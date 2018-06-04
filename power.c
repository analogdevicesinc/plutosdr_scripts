/*
 * power - AD9361 IIO streaming example which calculates channel power from DDS
 *
 * Copyright (C) 2014 IABG mbH
 * Author: Michael Feilen <feilen_at_iabg.de>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 **/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <getopt.h>

#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif

#define SAMPLES (1024*1024)

#define MY_NAME "power"

#ifdef _WIN32
#define snprintf sprintf_s
#endif

static const struct option options[] = {
    {"help", no_argument, 0, 'h'},
    {"dds", required_argument, 0, 'd'},
    {"tx", required_argument, 0, 't'},
    {"rx", required_argument, 0, 'r'},
    {"lo", required_argument, 0, 'l'},
    {"fs", required_argument, 0, 'f'},
    {0, 0, 0, 0},
};

static const char *options_descriptions[] = {
    "Show this help and quit.",
    "Set DDS tone frequency in Hz.",
    "Set RX gain in dB.",
    "Set TX gain in dB.",
    "Set LO frequency in MHz.",
    "Set sampling frequency in MHz.",
};

static void usage(void)
{
    unsigned int i;

    printf("Usage:\n\t" MY_NAME " -d 10000 -t -10 -r 10 -l 1000 -f 20\n\t"
           "\nMeasure power from DDS at a specific frequency in loopback\n"
           "\nOptions:\n");
    for (i = 0; options[i].name; i++)
        printf("\t-%c, --%s\n\t\t\t%s\n",
               options[i].val, options[i].name,
               options_descriptions[i]);
}


/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))
#define HZ(x) ((long long)(x*1.0 + .5))

#define ASSERT(expr) { \
	if (!(expr)) { \
		(void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
		(void) abort(); \
	} \
}

/* RX is input, TX is output */
enum iodev { RX, TX };

/* common RX and TX streaming params */
struct stream_cfg {
    long long bw_hz; // Analog banwidth in Hz
    long long fs_hz; // Baseband sample rate in Hz
    long long lo_hz; // Local oscillator frequency in Hz
    const char* rfport; // Port name
};

/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;

static bool stop;

/* cleanup and exit */
static void shutdown()
{
    printf("* Destroying buffers\n");
    if (rxbuf) {
        iio_buffer_destroy(rxbuf);
    }
    if (txbuf) {
        iio_buffer_destroy(txbuf);
    }

    printf("* Disabling streaming channels\n");
    if (rx0_i) {
        iio_channel_disable(rx0_i);
    }
    if (rx0_q) {
        iio_channel_disable(rx0_q);
    }
    if (tx0_i) {
        iio_channel_disable(tx0_i);
    }
    if (tx0_q) {
        iio_channel_disable(tx0_q);
    }

    printf("* Destroying context\n");
    if (ctx) {
        iio_context_destroy(ctx);
    }
    exit(0);
}

static void handle_sig(int sig)
{
    printf("Waiting for process to finish...\n");
    stop = true;
}

/* check return value of attr_write function */
static void errchk(int v, const char* what)
{
    if (v < 0) {
        fprintf(stderr,
                "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what);
        shutdown();
    }
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
    errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what,
                      const char* str)
{
    errchk(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)
{
    snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
    return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device* get_ad9361_phy(struct iio_context *ctx)
{
    struct iio_device *dev =  iio_context_find_device(ctx, "ad9361-phy");
    ASSERT(dev && "No ad9361-phy found");
    return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d,
                                  struct iio_device **dev)
{
    switch (d) {
    case TX:
        *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
        return *dev != NULL;
    case RX:
        *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
        return *dev != NULL;
    default:
        ASSERT(0);
        return false;
    }
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(struct iio_context *ctx, enum iodev d,
                                 struct iio_device *dev, int chid, struct iio_channel **chn)
{
    *chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
    if (!*chn)
        *chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
    return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid,
                         struct iio_channel **chn)
{
    switch (d) {
    case RX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage",
                                       chid), false);
        return *chn != NULL;
    case TX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage",
                                       chid), true);
        return *chn != NULL;
    default:
        ASSERT(0);
        return false;
    }
}

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(struct iio_context *ctx, enum iodev d,
                        struct iio_channel **chn)
{
    switch (d) {
    // LO chan is always output, i.e. true
    case RX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage",
                                       0), true);
        return *chn != NULL;
    case TX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage",
                                       1), true);
        return *chn != NULL;
    default:
        ASSERT(0);
        return false;
    }
}

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg,
                             enum iodev type, int chid)
{
    struct iio_channel *chn = NULL;

    // Configure phy and lo channels
    // printf("* Acquiring AD9361 phy channel %d\n", chid);
    if (!get_phy_chan(ctx, type, chid, &chn)) {
        return false;
    }
    wr_ch_str(chn, "rf_port_select",     cfg->rfport);
    wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
    wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);

    // Configure LO channel
    // printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
    if (!get_lo_chan(ctx, type, &chn)) {
        return false;
    }
    wr_ch_lli(chn, "frequency", cfg->lo_hz);
    return true;
}

double channel_power(int16_t *i, int16_t *q)
{
    // Calculate the squared RMS of signal
    double a,b,c,d,sum = 0;
    for (int k=0; k<SAMPLES; k++) {
        a = ((double)i[k]);
        b = ((double)q[k]);
        c = ((double)i[k]);
        d = (-1.0*(double)q[k]);
        sum = sum + a*c - b*d;
    }
    sum = sum/SAMPLES;
    return sum;
}

/* simple configuration and streaming */
int main (int argc, char **argv)
{
    // Test configuration
    int32_t dds_freq_hz = 10000;
    int16_t txgain = -10;
    int16_t rxgain = 10;
    long lo_mhz = 2400;
    long fs_mhz = 20;

    int c, option_index = 0;

    while ((c = getopt_long(argc, argv, "+hd:t:r:l:f:",
                            options, &option_index)) != -1) {
        switch (c) {
        case 'h':
            usage();
            return EXIT_SUCCESS;
        case 'd':
            dds_freq_hz = (int32_t) atoi(optarg);
            break;
        case 'r':
            rxgain = (int16_t) atoi(optarg);
            break;
        case 't':
            txgain = (int16_t) atoi(optarg);
            break;
        case 'l':
            lo_mhz = (long) atoi(optarg);
            break;
        case 'f':
            fs_mhz = (long) atoi(optarg);
            break;
        case '?':
            return -1;
        }
    }
    if (argc != 11) {
        fprintf(stderr, "Incorrect number of arguments.\n\n");
        usage();
        return -1;
    }


    // Streaming devices
    struct iio_device *tx;
    struct iio_device *rx;

    // RX sample counters
    size_t nrx = 0;

    // Stream configurations
    struct stream_cfg rxcfg;
    struct stream_cfg txcfg;

    // Listen to ctrl+c and ASSERT
    signal(SIGINT, handle_sig);

    printf("\n\n\n");
    printf("* Configuring transceiver at %ld MHz (lo) %ld MHz (fs) \n",lo_mhz,fs_mhz);
    // RX stream config
    rxcfg.bw_hz = MHZ(fs_mhz);   // 2 MHz rf bandwidth
    rxcfg.fs_hz = MHZ(fs_mhz);   // 2.5 MS/s rx sample rate
    rxcfg.lo_hz = MHZ(lo_mhz); // 2.5 GHz rf frequency
    rxcfg.rfport = "A_BALANCED"; // port A (select for rf freq.)

    // TX stream config
    txcfg.bw_hz = MHZ(fs_mhz); // 1.5 MHz rf bandwidth
    txcfg.fs_hz = MHZ(fs_mhz);   // 2.5 MS/s tx sample rate
    txcfg.lo_hz = MHZ(lo_mhz); // 2.5 GHz rf frequency
    txcfg.rfport = "A"; // port A (select for rf freq.)

    // printf("* Acquiring IIO context\n");
    struct iio_scan_context *sctx = iio_create_scan_context("usb", 0);
    if (sctx==NULL) {
        fprintf(stderr, "Unable to create scan context\n");
        abort();
    }
    struct iio_context_info **info;
    int ret = iio_scan_context_get_info_list(sctx, &info);
    if (ret < 0) {
        iio_scan_context_destroy(sctx);
        fprintf(stderr, "Unable to scan for Pluto devices\n");
        abort();
    }

    if (ret == 0) {
        iio_context_info_list_free(info);
        iio_scan_context_destroy(sctx);
        fprintf(stderr, "No Pluto device found\n");
    }

    if (ret > 1) {
        printf("More than one Pluto found:\n");

        for (unsigned int i = 0; i < (size_t) ret; i++) {
            printf("\t%d: %s [%s]\n", i,
                   iio_context_info_get_description(info[i]),
                   iio_context_info_get_uri(info[i]));
        }

        printf("We will use the first one.\n");
    }

    const char* uri = iio_context_info_get_uri(info[0]);
    // iio_context_info_list_free(info);
    iio_scan_context_destroy(sctx);

    ASSERT((ctx = iio_create_context_from_uri(uri)) && "No context");
    ASSERT(iio_context_get_devices_count(ctx) > 0 && "No devices");

    // printf("* Acquiring AD9361 streaming devices\n");
    ASSERT(get_ad9361_stream_dev(ctx, TX, &tx) && "No tx dev found");
    ASSERT(get_ad9361_stream_dev(ctx, RX, &rx) && "No rx dev found");

    // printf("* Configuring AD9361 for streaming\n");
    ASSERT(cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0) && "RX port 0 not found");
    ASSERT(cfg_ad9361_streaming_ch(ctx, &txcfg, TX, 0) && "TX port 0 not found");

    // printf("* Initializing AD9361 IIO streaming channels\n");
    ASSERT(get_ad9361_stream_ch(ctx, RX, rx, 0, &rx0_i) && "RX chan i not found");
    ASSERT(get_ad9361_stream_ch(ctx, RX, rx, 1, &rx0_q) && "RX chan q not found");
    ASSERT(get_ad9361_stream_ch(ctx, TX, tx, 0, &tx0_i) && "TX chan i not found");
    ASSERT(get_ad9361_stream_ch(ctx, TX, tx, 1, &tx0_q) && "TX chan q not found");

    // printf("* Enabling IIO streaming channels\n");
    iio_channel_enable(rx0_i);
    iio_channel_enable(rx0_q);
    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);

    // printf("* Creating non-cyclic IIO buffers\n");
    rxbuf = iio_device_create_buffer(rx, SAMPLES, false);
    if (!rxbuf) {
        perror("Could not create RX buffer");
        shutdown();
    }

    printf("* Setting manual gains to: %d dB (TX) %d dB (RX)\n", txgain, rxgain);
    struct iio_device *phy = iio_context_find_device(ctx, "ad9361-phy");
    ret = iio_channel_attr_write(iio_device_find_channel(phy, "voltage0", false),
                                 "gain_control_mode", "manual");
    if (ret < 0) {
        fprintf(stderr, "Failed to set AGC to manual mode: %d\n", ret);
        shutdown();
    }
    ret = iio_channel_attr_write_longlong(iio_device_find_channel(phy, "voltage0",
                                          false), "hardwaregain", rxgain);
    if (ret < 0) {
        fprintf(stderr, "Failed to set RX gain: %d\n", ret);
        shutdown();
    }
    ret = iio_channel_attr_write_longlong(iio_device_find_channel(phy, "voltage0",
                                          true), "hardwaregain", txgain);
    if (ret < 0) {
        fprintf(stderr, "Failed to set TX gain: %d\n", ret);
        shutdown();
    }

    printf("* Configuring DDS at %d Hz\n",dds_freq_hz);
    struct iio_device *dds = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    ret = iio_channel_attr_write_bool(iio_device_find_channel(dds, "altvoltage0",
                                      true), "raw", 1);
    if (ret < 0) {
        fprintf(stderr, "Failed to toggle DDS: %d\n", ret);
        shutdown();
    }
    ret = iio_channel_attr_write_double(iio_device_find_channel(dds, "altvoltage0",
                                        true), "frequency", dds_freq_hz);
    if (ret < 0) {
        fprintf(stderr, "Failed to set DDS frequency: %d\n", ret);
        shutdown();
    }
    ret = iio_channel_attr_write_double(iio_device_find_channel(dds, "altvoltage0",
                                        true), "scale", 1);
    if (ret < 0) {
        fprintf(stderr, "Failed to set DDS frequency: %d\n", ret);
        shutdown();
    }

    int16_t real_data[SAMPLES];
    int16_t imag_data[SAMPLES];
    double mean_channel_power = 0.0;

    printf("* Starting IO streaming (press CTRL+C to cancel)\n");
    while (!stop) {
        ssize_t nbytes_rx;
        char *p_dat, *p_end;
        ptrdiff_t p_inc;

        // Refill RX buffer
        nbytes_rx = iio_buffer_refill(rxbuf);
        if (nbytes_rx < 0) {
            printf("Error refilling buf %d\n",(int) nbytes_rx);
            shutdown();
        }

        // READ: Get pointers to RX buf and read IQ from RX buf port 0
        p_inc = iio_buffer_step(rxbuf);
        p_end = iio_buffer_end(rxbuf);
        int index = 0;
        for (p_dat = (char *)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end;
             p_dat += p_inc) {
            // Example: swap I and Q
            real_data[index] = ((int16_t*)p_dat)[0]; // Real (I)
            imag_data[index] = ((int16_t*)p_dat)[1]; // Imag (Q)
            index++;
        }
        mean_channel_power = (mean_channel_power + channel_power(real_data,
                              imag_data))/2;

        // Sample counter increment and status output
        nrx += nbytes_rx / iio_device_get_sample_size(rx);
        printf("Power %8.4f dB (Average)| Samples processed %8.2f MSmp\n",
               log10(mean_channel_power)*20, nrx/1e6);
    }

    iio_context_info_list_free(info);
    shutdown();

    return 0;
}
