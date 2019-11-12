#include <iio.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define MAX_SIZE 1000000

int main(void)
{
    struct iio_context *ctx;
    struct iio_device *dev_phy;
    struct iio_device *tx;
    struct iio_channel *ch;
    struct iio_channel *tx0_i;
    struct iio_channel *tx0_q;
    struct iio_buffer *txbuf;
    ssize_t ret;
    int indx;
    ssize_t nbytes_tx;
    char *p_dat, *p_end;
    ptrdiff_t p_inc;
    FILE *fp;
    int linect = 0;
    char buf[128];


    // ctx = iio_create_default_context();
    ctx = iio_create_context_from_uri("local:");
    dev_phy = iio_context_find_device(ctx, "ad9361-phy");
    ch = iio_device_find_channel(dev_phy, "voltage0", true);
    tx = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    tx0_i = iio_device_find_channel(tx, "voltage0", true);
    tx0_q = iio_device_find_channel(tx, "voltage1", true);

    ret = iio_channel_attr_write(ch, "sampling_frequency", "61440000");
    if (ret<0)
        printf("Writing sampling_frequency failed");
    ret = iio_channel_attr_write(ch, "rf_bandwidth", "56000000");
    if (ret<0)
        printf("Writing rf_bandwidth failed");
    ret = iio_channel_attr_write(ch, "hardwaregain", "-10");
    if (ret<0)
        printf("Writing hardwaregain failed");
    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);


    // Set LED
    FILE* fd;
    fd = fopen("/sys/class/leds/led0:green/trigger", "w");
    fprintf(fd, "none");
    fclose(fd);

    // Read data in
    int *i_data = (int*) malloc(sizeof(int)* MAX_SIZE);
    int *q_data = (int*) malloc(sizeof(int)* MAX_SIZE);

    fp = fopen("data.bin", "r");
    if(fp == NULL) {
        fprintf(stderr,"Cannot open file for reading");
        exit(EXIT_FAILURE);
    }
    while( fgets(buf,sizeof(buf),fp) != NULL ) {
        sscanf(buf, "%d, %d", &i_data[linect], &q_data[linect]);
        printf("Set %d - 1st: %d, 2nd: %d\n", linect, i_data[linect], q_data[linect]);
        linect++;
        if ((linect+1)>=MAX_SIZE) {
            fprintf(stderr,"Max data size exceeded");
            exit(EXIT_FAILURE);
        }

    }
    fclose(fp);


    // Create buffer
    txbuf = iio_device_create_buffer(tx, linect, true);
    if (!txbuf) {
        fprintf(stderr,"Could not create TX buffer");
        exit(EXIT_FAILURE);
    }
    // Fill buffer
    p_inc = iio_buffer_step(txbuf);
    p_end = iio_buffer_end(txbuf);
    indx = 0;
    for (p_dat = (char *)iio_buffer_first(txbuf, tx0_i); p_dat < p_end;
         p_dat += p_inc) {
        ((int16_t*)p_dat)[0] = i_data[indx]; // Real (I)
        indx++;
    }
    indx = 0;
    for (p_dat = (char *)iio_buffer_first(txbuf, tx0_q); p_dat < p_end;
         p_dat += p_inc) {
        ((int16_t*)p_dat)[0] = q_data[indx]; // Real (I)
        indx++;
    }
    nbytes_tx = iio_buffer_push(txbuf);
    if (nbytes_tx < 0) {
        fprintf(stderr,"Error pushing buf %d\n", (int) nbytes_tx);
        exit(EXIT_FAILURE);
    }

    // Set LED again
    fd = fopen("/sys/class/leds/led0:green/trigger", "w");
    fprintf(fd, "default-on");
    fclose(fd);

    while (1)
        sleep(30);

    iio_context_destroy(ctx);
    return EXIT_SUCCESS;


}
