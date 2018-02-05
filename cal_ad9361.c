/*
 * Copyright (C) 2014,2017 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *         Robin Getz <robin.getz@analog.com>
 *
 * Licensed under the GPL-2.
 *
 * */

#include <errno.h>
#include <getopt.h>
#include <iio.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <values.h>
#include <complex.h>
#include <fftw3.h>

#define SAMPLES_PER_READ 1048576

static const struct option options[] = {
	{"help", no_argument, 0, 'h'},
	{"uri", required_argument, 0, 'u'},
	{"buffer-size", required_argument, 0, 'b'},
	{"samples", required_argument, 0, 's' },
	{"sample-rate", required_argument, 0, 'S'},
	{"rxlo-freq", required_argument, 0, 'r'},
	{"txlo-freq", required_argument, 0, 't'},
	{"external-tone", required_argument, 0, 'e'},
	{"gps-pps", no_argument, 0, 'g'},
	{"timeout", required_argument, 0, 'T'},
	{"auto", no_argument, 0, 'a'},
	{"data-file", no_argument, 0, 'f'},
	{ NULL, no_argument, NULL, 0 }
};

static const char *options_descriptions[] = {
	"Show this help and quit.",
	"Use the context with the provided URI.",
	"Size of capture buffers. Default is 1048576.",
	"Number of buffers to capture, 0 = infinite. Default is 1",
	"Sample rate. Default is 30720000.",
	"Rx LO frequency in Hz. Default is 0. 0 is off",
	"Tx LO frequency in Hz. Default is 0. 0 is off",
	"External Tone in Hz. Default is 0. 0 is off",
	"Buffer timeout in milliseconds. 0 = no timeout",
	"Scan for available contexts and if only one is available use it.",
	"Save captured data to a file.",
};

static void usage(const char *name)
{
	unsigned int i;

	printf("Usage:\n\t %s [-T <timeout-ms>] [-b <buffer-size>] [-s <samples>] "
			"<iio_device> [<channel> ...]\n\nOptions:\n", name);
	for (i = 0; options[i].name; i++)
		printf("\t-%c, --%s\n\t\t\t%s\n",
					options[i].val, options[i].name,
					options_descriptions[i]);
}

static struct iio_context *ctx;
static struct iio_buffer *buffer;
static size_t num_samples = 1;

static volatile sig_atomic_t app_running = true;
static int exit_code = EXIT_SUCCESS;

static void quit_all(int sig)
{
	exit_code = sig;
	app_running = false;
	if (buffer)
		iio_buffer_cancel(buffer);
}

#ifdef _WIN32

#include <windows.h>

BOOL WINAPI sig_handler_fn(DWORD dwCtrlType)
{
	/* Runs in its own thread */

	switch (dwCtrlType) {
	case CTRL_C_EVENT:
	case CTRL_CLOSE_EVENT:
		quit_all(SIGTERM);
		return TRUE;
	default:
		return FALSE;
	}
}

static void setup_sig_handler(void)
{
	SetConsoleCtrlHandler(sig_handler_fn, TRUE);
}

#elif NO_THREADS

static void sig_handler(int sig)
{
	/*
	 * If the main function is stuck waiting for data it will not abort. If the
	 * user presses Ctrl+C a second time we abort without cleaning up.
	 */
	if (!app_running)
		exit(sig);
	app_running = false;
}

static void set_handler(int sig)
{
	struct sigaction action;

	sigaction(sig, NULL, &action);
	action.sa_handler = sig_handler;
	sigaction(sig, &action, NULL);
}

static void setup_sig_handler(void)
{
	set_handler(SIGHUP);
	set_handler(SIGPIPE);
	set_handler(SIGINT);
	set_handler(SIGSEGV);
	set_handler(SIGTERM);
}

#else

#include <pthread.h>

static void * sig_handler_thd(void *data)
{
	sigset_t *mask = data;
	int ret, sig;

	/* Blocks until one of the termination signals is received */
	do {
		ret = sigwait(mask, &sig);
	} while (ret == EINTR);

	quit_all(ret);

	return NULL;
}

static void setup_sig_handler(void)
{
	sigset_t mask, oldmask;
	pthread_t thd;
	int ret;

	/*
	 * Async signals are difficult to handle and the IIO API is not signal
	 * safe. Use a seperate thread and handle the signals synchronous so we
	 * can call iio_buffer_cancel().
	 */

	sigemptyset(&mask);
	sigaddset(&mask, SIGHUP);
	sigaddset(&mask, SIGPIPE);
	sigaddset(&mask, SIGINT);
	sigaddset(&mask, SIGSEGV);
	sigaddset(&mask, SIGTERM);

	pthread_sigmask(SIG_BLOCK, &mask, &oldmask);

	ret = pthread_create(&thd, NULL, sig_handler_thd, &mask);
	if (ret) {
		fprintf(stderr, "Failed to create signal handler thread: %d\n", ret);
		pthread_sigmask(SIG_SETMASK, &oldmask, NULL);
	}
}

#endif

static struct iio_context *scan(void)
{
	struct iio_scan_context *scan_ctx;
	struct iio_context_info **info;
	struct iio_context *ctx = NULL;
	unsigned int i;
	ssize_t ret;

	scan_ctx = iio_create_scan_context(NULL, 0);
	if (!scan_ctx) {
		fprintf(stderr, "Unable to create scan context\n");
		return NULL;
	}

	ret = iio_scan_context_get_info_list(scan_ctx, &info);
	if (ret < 0) {
		char err_str[1024];
		iio_strerror(-ret, err_str, sizeof(err_str));
		fprintf(stderr, "Scanning for IIO contexts failed: %s\n", err_str);
		goto err_free_ctx;
	}

	if (ret == 0) {
		printf("No IIO context found.\n");
		goto err_free_info_list;
	}

	if (ret == 1) {
		ctx = iio_create_context_from_uri(iio_context_info_get_uri(info[0]));
	} else {
		fprintf(stderr, "Multiple contexts found. Please select one using --uri:\n");

		for (i = 0; i < (size_t) ret; i++) {
			fprintf(stderr, "\t%d: %s [%s]\n", i,
				iio_context_info_get_description(info[i]),
				iio_context_info_get_uri(info[i]));
		}
	}

err_free_info_list:
	iio_context_info_list_free(info);
err_free_ctx:
	iio_scan_context_destroy(scan_ctx);

	return ctx;
}

struct write_l {
	char *device;
	char *channel;
	bool in_out;
	char *attribute;
	char *value;
};

struct write_l write_log[128];
static int wl_index = 0;

static bool iio_set_attribute(char * device, char * channel, bool in_out,
		char * attribute, char * buffer, bool log)
{
	struct iio_device *dev;
	struct iio_channel *chan;
	const char * attr;
	ssize_t ret;
	char buf[1024];

	dev = iio_context_find_device(ctx, device);
	if (!dev) {
		fprintf(stderr, "Device %s not found\n", device);
		iio_context_destroy(ctx);
		return false;
	}

	chan = iio_device_find_channel(dev, channel, in_out);
	if (!chan) {
		fprintf(stderr, "%s channel not found in %s\n", channel, device);
		iio_context_destroy(ctx);
		return false;
	}

	attr = iio_channel_find_attr(chan, attribute);
	if (!attr) {
		fprintf(stderr, "%s attribute not found in %s:%s\n", attribute, device, channel);
		iio_context_destroy(ctx);
		return false;
	}

	ret = iio_channel_attr_read(chan, attribute, buf, sizeof(buf));
	if (ret > 0) {
		if (log) {
			write_log[wl_index].device = strdup(device);
			write_log[wl_index].channel = strdup(channel);
			write_log[wl_index].in_out = in_out;
			write_log[wl_index].attribute = strdup(attribute);
			write_log[wl_index].value = strdup(buf);
			wl_index++;
		}
	}

	if (buffer) {
		ret = iio_channel_attr_write(chan, attribute, buffer);
		if (ret < 0) {
			printf("write '%s' failed to %s:%s:%s\n", buffer, device, channel, attribute);
			iio_context_destroy(ctx);
			return false;
		}
	}
	return true;

}

static double win_hanning(int j, int n)
{
	double a = 2.0 * M_PI / (n - 1), w;
	w = 0.5 * (1.0 - cos(a * j));
	return (w);
}

static double lo_frequency (uint32_t xo)
{
	uint32_t tmp1, tmp2, tmp3;
	double int_portion, fract_portion, vco_div;

	/* registers are 8-bits*/
	/* 0x231 is lower Rx synthesizer word */
	iio_device_reg_read(iio_context_find_device(ctx, "ad9361-phy"), 0x231, &tmp1);
	/* lower two bits in 0x232 are bits 10:8 */
	iio_device_reg_read(iio_context_find_device(ctx, "ad9361-phy"), 0x232, &tmp2);
	int_portion = ((tmp2 & 7) << 8) | tmp1;

	iio_device_reg_read(iio_context_find_device(ctx, "ad9361-phy"), 0x233, &tmp1);
	iio_device_reg_read(iio_context_find_device(ctx, "ad9361-phy"), 0x234, &tmp2);
	iio_device_reg_read(iio_context_find_device(ctx, "ad9361-phy"), 0x235, &tmp3);
	fract_portion = ((tmp3 & 0x7F) << 16) | (tmp2 << 8) | tmp1;

	iio_device_reg_read(iio_context_find_device(ctx, "ad9361-phy"), 0x5, &tmp1);
	vco_div = (tmp1 & 7);

	if (xo < 41000000)
		xo = xo * 2;

	return ((xo * fract_portion) + (xo * int_portion * 8388593)) /
			(pow(2, (vco_div + 1)) * 8388593);
}

#define IN false
#define OUT true

int main(int argc, char **argv)
{
	unsigned int i, nb_channels, tx_lo = 0,
			sample_rate = 30720000;
	uint64_t rx_lo = 0, external_tone = 0;
	unsigned int buffer_size = SAMPLES_PER_READ;
	long long xo = 0, new_xo = 0;
	int c, option_index = 0, arg_index = 0, uri_index = 0;
	struct iio_device *dev;
	size_t sample_size;
	int timeout = -1;
	bool scan_for_context = false, save_data = false, gps = false;
	char buf[256];
	fftw_complex *in_c, *out;
	fftw_plan plan_forward;
	double *win, exact_rx_lo, mult_fs, mult_lo;
	struct iio_channel *rx_i;
	FILE * fd = NULL;

	while ((c = getopt_long(argc, argv, "+hfu:b:s:T:aS:r:t:e:g",
					options, &option_index)) != -1) {
		switch (c) {
		case 'h':
			usage(argv[0]);
			return EXIT_SUCCESS;
		case 'f':
			arg_index += 1;
			save_data = true;
			break;
		case 'u':
			arg_index += 2;
			uri_index = arg_index;
			break;
		case 'a':
			arg_index += 1;
			scan_for_context = true;
			break;
		case 'b':
			arg_index += 2;
			buffer_size = atoi(argv[arg_index]);
			break;
		case 's':
			arg_index += 2;
			num_samples = atoi(argv[arg_index]);
			break;
		case 'T':
			arg_index += 2;
			timeout = atoi(argv[arg_index]);
			break;
		case 'S':
			arg_index += 2;
			sample_rate = atoi(argv[arg_index]);
			break;
		case 'r':
			arg_index += 2;
			rx_lo = strtoll(argv[arg_index], NULL, 10);
			if (tx_lo || external_tone) {
				fprintf(stderr, "-e -r -t are not compatible\n");
				return EXIT_FAILURE;
			}
			if (rx_lo >= 6000000000) {
				fprintf(stderr, "rx_out of range\n");
				return EXIT_FAILURE;
			}
			break;
		case 't':
			arg_index += 2;
			tx_lo = atoi(argv[arg_index]);
			if (rx_lo || external_tone) {
				fprintf(stderr, "-e -r -t are not compatible\n");
				return EXIT_FAILURE;
			}
			break;
		case 'e':
			arg_index += 2;
			external_tone = strtoll(argv[arg_index], NULL, 10);
			if (rx_lo || tx_lo) {
				fprintf(stderr, "-e -r -t are not compatible\n");
				return EXIT_FAILURE;
			}
			break;
		case 'g':
			arg_index += 1;
			gps = true;
			break;
		case '?':
			return EXIT_FAILURE;
		}
	}

	num_samples = num_samples * buffer_size;

	if (arg_index >= argc) {
		fprintf(stderr, "Incorrect number of arguments.\n\n");
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	if (!rx_lo && tx_lo) {
		rx_lo = tx_lo;
	}

	if (!rx_lo && external_tone) {
		rx_lo = external_tone - sample_rate/3;
	}

	setup_sig_handler();

	if (scan_for_context)
		ctx = scan();
	else if (uri_index)
		ctx = iio_create_context_from_uri(argv[uri_index]);
	else
		ctx = iio_create_default_context();

	if (!ctx) {
		fprintf(stderr, "Unable to create IIO context\n");
		return EXIT_FAILURE;
	}

	if (timeout >= 0)
		iio_context_set_timeout(ctx, timeout);

        if (gps) {
		long long val = 0;
		int ret;
		struct iio_device *dev;
		struct iio_channel *pps;
		dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
		if (!dev) {
			printf("no device cf-ad9361-lpc\n");
			return EXIT_FAILURE;
		}
		pps = iio_device_find_channel(dev, "voltage0", 0);
		if (!pps) {
			printf("no channel voltage0\n");
			return EXIT_FAILURE;
		}
		sprintf(buf, "%u", sample_rate);
		if (!iio_set_attribute("ad9361-phy", "voltage0", OUT,
				"sampling_frequency", buf, true))
			return EXIT_FAILURE;
		ret = iio_channel_attr_read_longlong(pps, "in_voltage_samples_pps", &val);
		if (ret) {
			printf("no in_voltage_samples_pps\n");
			return EXIT_FAILURE;
		}

		iio_device_attr_read_longlong(iio_context_find_device(ctx, "ad9361-phy"), "xo_correction", &xo);
		printf("pps: %llu (%f ppm off) xo was: %llu ", val, (double)(val - sample_rate)/(double)sample_rate * 1e6, xo);
		new_xo = (long long)((double)(1.0 + (double)(val - sample_rate)/(double)sample_rate) * xo);
		if (xo == new_xo) {
			printf("\n");
		} else {
			printf("now %llu\n", new_xo);
			iio_device_attr_write_longlong(
				iio_context_find_device(ctx, "ad9361-phy"),
				"xo_correction", new_xo);
		}
		sleep(2);
		quit_all(SIGTERM);
		return EXIT_SUCCESS;
	}


	sprintf(buf, "%lu", rx_lo);
	if (!iio_set_attribute("ad9361-phy", "RX_LO", OUT, "frequency", buf, true))
		return EXIT_FAILURE;
	iio_channel_attr_read(
			iio_device_find_channel(
				iio_context_find_device(ctx, "ad9361-phy"),
			"RX_LO", OUT),
		"frequency", buf, sizeof(buf));
	rx_lo = strtoll(buf, NULL, 10);

	iio_device_attr_read_longlong(iio_context_find_device(ctx, "ad9361-phy"), "xo_correction", &xo);

	exact_rx_lo=lo_frequency(xo);
	mult_lo = exact_rx_lo / (double)xo;
	mult_fs = sample_rate / (double)xo;

	sprintf(buf, "%u", sample_rate);
	if (!iio_set_attribute("ad9361-phy", "voltage0", OUT, "sampling_frequency", buf, true))
		return EXIT_FAILURE;
	if (!iio_set_attribute("ad9361-phy", "voltage0", IN, "gain_control_mode", "slow_attack", true))
		return EXIT_FAILURE;

	rx_i = iio_device_find_channel(iio_context_find_device(ctx, "cf-ad9361-lpc"), "voltage0", 0);

	if (tx_lo) {
		sprintf(buf, "%u", tx_lo);
		if (!iio_set_attribute("ad9361-phy", "TX_LO", OUT, "powerdown", "0", true) ||
		    !iio_set_attribute("ad9361-phy", "TX_LO", OUT, "frequency", buf, true))
			return EXIT_FAILURE;
		sprintf(buf, "%u", sample_rate / 3);
		if (!iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F1", OUT, "frequency" , buf, true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F2", OUT, "frequency" , buf, true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F1", OUT, "frequency" , buf, true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F2", OUT, "frequency" , buf, true))
			return EXIT_FAILURE;
		else {
			iio_channel_attr_read(
					iio_device_find_channel(
						iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc"),
						"TX1_I_F1", OUT),
					"frequency", buf, sizeof(buf));
			printf("got %s\n", buf);
		}
		if (!iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F1", OUT, "scale" , "0.4", true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F2", OUT, "scale" , "0.4", true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F1", OUT, "scale" , "0.4", true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F2", OUT, "scale" , "0.4", true))
			return EXIT_FAILURE;
		if (!iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F1", OUT, "phase" , "90000", true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F2", OUT, "phase" , "90000", true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F1", OUT, "phase" , "0", true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F2", OUT, "phase" , "0", true))
			return EXIT_FAILURE;
		if (!iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F1", OUT, "raw" , "1", true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F2", OUT, "raw" , "1", true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F1", OUT, "raw" , "1", true) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F2", OUT, "raw" , "1", true))
			return EXIT_FAILURE;
	} else {
		if (!iio_set_attribute("ad9361-phy", "TX_LO", OUT, "powerdown", "1", true))
			return EXIT_FAILURE;
	}
	/* pause for RSSI to stablize */
	usleep (100000);
	iio_channel_attr_read(
			iio_device_find_channel(
				iio_context_find_device(ctx, "ad9361-phy"),
			"voltage0", false),
		"rssi", buf, sizeof(buf));
	i=atoi(buf);
	if (i > 75) {
		printf("signal too weak (rssi = %i), adjust transmitter\n", i);
		quit_all(EXIT_FAILURE);
	}
	iio_device_attr_read(iio_context_find_device(ctx, "ad9361-phy"), "xo_correction", buf, sizeof(buf));
	xo=atoi(buf);

	dev = iio_context_find_device(ctx, "cf-ad9361-lpc");

	if (!dev) {
		fprintf(stderr, "Device %s not found\n", argv[arg_index + 1]);
		iio_context_destroy(ctx);
		return EXIT_FAILURE;
	}

	nb_channels = iio_device_get_channels_count(dev);
	printf("enabled %i channels\n", nb_channels);

	/* Enable all channels */
	for (i = 0; i < nb_channels; i++)
		iio_channel_enable(iio_device_get_channel(dev, i));

	sample_size = iio_device_get_sample_size(dev);

	buffer = iio_device_create_buffer(dev, buffer_size, false);
	if (!buffer) {
		char buf[256];
		iio_strerror(errno, buf, sizeof(buf));
		fprintf(stderr, "Unable to allocate buffer: %s\n", buf);
		iio_context_destroy(ctx);
		return EXIT_FAILURE;
	}

	win = fftw_malloc(sizeof(double) * buffer_size);
	in_c = fftw_malloc(sizeof(fftw_complex) * buffer_size);
	out = fftw_malloc(sizeof(fftw_complex) * (buffer_size + 1));
	plan_forward = fftw_plan_dft_1d(buffer_size, in_c, out, FFTW_FORWARD, FFTW_ESTIMATE);
	for (i = 0; i < buffer_size; i ++)
		win[i] = win_hanning(i, buffer_size);

	while (app_running) {
		int ret;
		unsigned int j, k, cnt, bin[5];
		double mag[5], peak[5], side[2];

		ret = iio_buffer_refill(buffer);
		if (ret < 0) {
			if (app_running) {
				char buf[256];
				iio_strerror(-ret, buf, sizeof(buf));
				fprintf(stderr, "Unable to refill buffer: %s\n", buf);
			}
			break;
		}

		/* If there are only the samples we requested, we don't need to
		 * demux */
		if (iio_buffer_step(buffer) == (ptrdiff_t)sample_size) {
			void *data, *end;
			ptrdiff_t inc;
			double actual_bin;

			end = iio_buffer_end(buffer);
			inc = iio_buffer_step(buffer);

			for (cnt = 0, data = iio_buffer_first(buffer, rx_i); data < end; data += inc, cnt++) {
				const int16_t real = ((int16_t*)data)[0]; // Real (I)
				const int16_t imag = ((int16_t*)data)[1]; // Imag (Q)

				in_c[cnt] = (real * win[cnt] + I * imag * win[cnt]) / 2048;
			}

			fftw_execute(plan_forward);

			for (j = 0; j <= 2; j++) {
				peak[j] = -FLT_MAX;
				bin[j] = 0;
			}

			if (save_data)
				fd = fopen("./dat.bin", "w");

			for (i = 1; i < buffer_size; ++i) {
				mag[2] = mag[1];
				mag[1] = mag[0];
				mag[0] = 10 * log10((creal(out[i]) * creal(out[i]) + cimag(out[i]) * cimag(out[i])) /
						((unsigned long long)buffer_size * (unsigned long long)buffer_size));
				if (fd)
					fprintf(fd, "%f\n", mag[0]);
				if (i < 2)
					continue;
				for (j = 0; j <= 2; j++) {
					if  ((mag[1] > peak[j]) &&
							((!((mag[2] > mag[1]) && (mag[1] > mag[0]))) &&
							 (!((mag[2] < mag[1]) && (mag[1] < mag[0]))))) {
						for (k = 2; k > j; k--) {
							peak[k] = peak[k - 1];
							bin[k] = bin[k - 1];
						}
						peak[j] = mag[1];
						bin[j] = i - 1;
						if (j == 0) {
							side[0] = mag[0];
							side[1] = mag[2];
						}
						break;
					}
				}
			}
			if (fd)
				fclose(fd);
			fd = NULL;

			printf("peaks at ");
			for (j = 0; j <= 2; j++) 
				printf("%f (%i); ", peak[j], bin[j]);
			printf("\nSFDR = %f\n", peak[0] - peak[1]);

			if (peak[0] < (peak[1] + 20)) {
				printf("can't find strong signal, fix signal source\n");
				quit_all(EXIT_FAILURE);
			}
			/* based on
			 * https://ccrma.stanford.edu/~jos/sasp/Quadratic_Interpolation_Spectral_Peaks.html
			 */
			actual_bin = bin[0] + (side[0] - side[1])/(2.0 * (side[0] - 2*peak[0] + side[1]));
			printf("reading : %f\n", actual_bin * sample_rate / buffer_size + exact_rx_lo);
			printf("error   : %f\n", fabs((actual_bin * sample_rate / buffer_size + exact_rx_lo) -
						(double)external_tone));

			printf("guess : %lf\n", actual_bin);
			/* fftw folds things, so:
			 * sample 0 is LO, buffer_size/2 is +FS/2
			 * (buffer_size/2)+1 is -FS/2, and buffer_size is LO
			 */
			if (bin[0] < (buffer_size/2)) {
				double ppm;

				ppm = (double)(external_tone * buffer_size) /
					(xo * (mult_fs * actual_bin + mult_lo * buffer_size));
				printf("error = %2.4f ppm\n", (1-ppm) * 1000000);
				printf("old xo : %lli\n", xo);
				if (abs(1 - ppm) * 1000000 <= 25)
					xo = (int)round(xo * ppm);
				else
					printf("calculated ppm value too high %f\n", (1 - ppm) * 1000000);
				printf("new xo : %lli\n", xo);
			} else {
			}
			iio_device_attr_write_longlong(iio_context_find_device(ctx, "ad9361-phy"), "xo_correction", xo);
//			iio_buffer_foreach_sample(buffer, print_sample, NULL);
		}
		app_running = false;
	}

	for (c = wl_index - 1; c >= 0; c--) {
		iio_set_attribute(write_log[c].device, write_log[c].channel, write_log[c].in_out,
				write_log[c].attribute, write_log[c].value, false);
	}
	iio_buffer_destroy(buffer);
	iio_context_destroy(ctx);
	return exit_code;
}
