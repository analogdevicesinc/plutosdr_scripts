/*
 * Copyright (C) 2014,2017 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *         Robin Getz <robin.getz@analog.com>
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
#include <values.h>
#include <complex.h>
#include <fftw3.h>


#define MY_NAME "cal_ad9361"

#define SAMPLES_PER_READ 1048576

static const struct option options[] = {
	{"help", no_argument, 0, 'h'},
	{"uri", required_argument, 0, 'u'},
	{"buffer-size", required_argument, 0, 'b'},
	{"samples", required_argument, 0, 's' },
	{"sample-rate", required_argument, 0, 'S'},
	{"Rx LO frequency", required_argument, 0, 'r'},
	{"Tx LO frequency", required_argument, 0, 't'},
	{"timeout", required_argument, 0, 'T'},
	{"auto", no_argument, 0, 'a'},
	{0, 0, 0, 0},
};

static const char *options_descriptions[] = {
	"Show this help and quit.",
	"Use the context with the provided URI.",
	"Size of capture buffers. Default is 1048576.",
	"Number of buffers to capture, 0 = infinite. Default is 1",
	"Sample rate. Default is 30720000.",
	"Rx LO frequency. Default is 2000000000 (2GHz). 0 is off",
	"Tx LO frequency. Default is 0. 0 is off",
	"Buffer timeout in milliseconds. 0 = no timeout",
	"Scan for available contexts and if only one is available use it.",
};

static void usage(void)
{
	unsigned int i;

	printf("Usage:\n\t" MY_NAME "[-T <timeout-ms>] [-b <buffer-size>] [-s <samples>] "
			"<iio_device> [<channel> ...]\n\nOptions:\n");
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

static bool iio_set_attribute(char * device, char * channel, char * attribute, char * buffer)
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

	chan = iio_device_find_channel(dev, channel, true);
	if (!chan)
		chan = iio_device_find_channel(dev, channel, false);
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
	if (ret > 0)
		printf("%s %s %s was : %s\n", device, channel, attribute, buf);

	ret = iio_channel_attr_write(chan, attribute, buffer);
	if (ret < 0) {
		printf("write '%s' failed to %s:%s:%s\n", buffer, device, channel, attribute);
		iio_context_destroy(ctx);
		return false;
	}
	return true;

}

static ssize_t demux_sample(const struct iio_channel *chn,
		void *sample, size_t size, void *d)
{
	const struct iio_data_format *format = iio_channel_get_data_format(chn);
/*
	if (size == 1) {
		int8_t val;

		iio_channel_convert(chn, &val, sample);
		if (format->is_signed)
			*() = (float) val;
		else
			*() = (float) (uint8_t)val;
	} else if (size == 2) {
		int16_t val;

		iio_channel_convert(chn, &val, sample);
		if (format->is_signed)
			*() = (float) val;
		else
			*() = (float) (uint16_t)val;
	} else {
		int32_t val;

		iio_channel_convert(chn, &val, sample);
		if (format->is_signed)
			*() = (float) val;
		else
			*() = (float) (uint32_t)val;
	}
*/
	return size;
}

static double win_hanning(int j, int n)
{
	double a = 2.0 * M_PI / (n - 1), w;
	w = 0.5 * (1.0 - cos(a * j));
	return (w);
}


int main(int argc, char **argv)
{
	unsigned int i, nb_channels, rx_lo = 2000000000, tx_lo = 0, sample_rate = 30720000;
	unsigned int buffer_size = SAMPLES_PER_READ;
	int c, option_index = 0, arg_index = 0, ip_index = 0, uri_index = 0;
	struct iio_device *dev;
	struct iio_channel *chan;
	size_t sample_size;
	int timeout = -1;
	bool scan_for_context = false;
	char buf[256];

	while ((c = getopt_long(argc, argv, "+hu:b:s:T:aS:r:t:",
					options, &option_index)) != -1) {
		switch (c) {
		case 'h':
			usage();
			return EXIT_SUCCESS;
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
			rx_lo = atoi(argv[arg_index]);
			break;
		case 't':
			arg_index += 2;
			tx_lo = atoi(argv[arg_index]);
			break;
		case '?':
			return EXIT_FAILURE;
		}
	}

	num_samples = num_samples * buffer_size;

	if (arg_index >= argc) {
		fprintf(stderr, "Incorrect number of arguments.\n\n");
		usage();
		return EXIT_FAILURE;
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

	sprintf(buf, "%u", rx_lo);
	if (!iio_set_attribute("ad9361-phy", "RX_LO", "frequency", buf))
		return EXIT_FAILURE;
	sprintf(buf, "%u", sample_rate);
	if (!iio_set_attribute("ad9361-phy", "voltage0", "sampling_frequency", buf))
		return EXIT_FAILURE;
	if (tx_lo) {
		sprintf(buf, "%u", tx_lo);
		if (!iio_set_attribute("ad9361-phy", "TX_LO", "powerdown", "0") ||
		    !iio_set_attribute("ad9361-phy", "TX_LO", "frequency", buf))
			return EXIT_FAILURE;
		sprintf(buf, "%u", sample_rate / 3);
		if (!iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F1", "frequency" , buf) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F2", "frequency" , buf) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F1", "frequency" , buf) ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F2", "frequency" , buf))
			return EXIT_FAILURE;
		if (!iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F1", "scale" , "0.4") ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F2", "scale" , "0.4") ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F1", "scale" , "0.4") ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F2", "scale" , "0.4"))
			return EXIT_FAILURE;
		if (!iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F1", "phase" , "90000") ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F2", "phase" , "90000") ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F1", "phase" , "0") ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F2", "phase" , "0"))
			return EXIT_FAILURE;
		if (!iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F1", "raw" , "1") ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_I_F2", "raw" , "1") ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F1", "raw" , "1") ||
		    !iio_set_attribute("cf-ad9361-dds-core-lpc", "TX1_Q_F2", "raw" , "1"))
			return EXIT_FAILURE;
	} else {
		if (!iio_set_attribute("ad9361-phy", "TX_LO", "powerdown", "1"))
			return EXIT_FAILURE;
	}

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

	while (app_running) {
		int ret = iio_buffer_refill(buffer);
		double *win;
		unsigned int j, fft_size, cnt, bin1, bin2;
		fftw_complex *in_c, *out;
		fftw_plan plan_forward;
		float *in_data, *in_data_c, mag, peak1, peak2;

		if (ret < 0) {
			if (app_running) {
				char buf[256];
				iio_strerror(-ret, buf, sizeof(buf));
				fprintf(stderr, "Unable to refill buffer: %s\n", buf);
			}
			break;
		}

		win = fftw_malloc(sizeof(double) * fft_size);
		in_c = fftw_malloc(sizeof(fftw_complex) * fft_size);
		out = fftw_malloc(sizeof(fftw_complex) * (fft_size + 1));
		plan_forward = fftw_plan_dft_1d(fft_size, in_c, out, FFTW_FORWARD, FFTW_ESTIMATE);
		for (i = 0; i < fft_size; i ++)
			win[i] = win_hanning(i, fft_size);

		/* If there are only the samples we requested, we don't need to
		 * demux */
		if (iio_buffer_step(buffer) == sample_size) {
			void *start = iio_buffer_start(buffer);
			size_t read_len, len = (intptr_t) iio_buffer_end(buffer)
				- (intptr_t) start;

			if (num_samples && len > num_samples * sample_size)
				len = num_samples * sample_size;

			iio_buffer_foreach_sample(buffer, demux_sample, NULL);

			/* normalization and scaling see fft_corr */
			for (cnt = 0, i = 0; cnt < fft_size; cnt++) {
				in_c[cnt] = in_data_c[i] * win[cnt] + I * in_data_c[i] * win[cnt];
				i++;
			}
			fftw_execute(plan_forward);

			peak2 = peak1 = FLT_MAX;
			bin2 = bin1 = 0;

			for (i = 0; i < fft_size; ++i) {
				mag = 10 * log10((creal(out[j]) * creal(out[j]) + cimag(out[j]) * cimag(out[j])) /
						((unsigned long long)fft_size * fft_size));
				if (mag > peak1) {
					peak2 = peak1;
					bin2 = bin1;
					peak1 = mag;
					bin1 = i;
				}
			}

			printf("peaks at %d (%i) and %d (%i)\n", peak2, bin2, peak1, bin1);

			if (num_samples) {
				num_samples -= read_len / sample_size;
				if (!num_samples)
					quit_all(EXIT_SUCCESS);
			}
		} else {
//			iio_buffer_foreach_sample(buffer, print_sample, NULL);
		}
	}

err_destroy_buffer:
	iio_buffer_destroy(buffer);
	iio_context_destroy(ctx);
	return exit_code;
}
