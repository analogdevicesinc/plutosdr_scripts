#!/usr/bin/env python
#
# Copyright (C) 2018 Analog Devices, Inc.
# Author: Travis Collins <travis.collins@analog.com>
#
# Licensed under the GPL-2.

import sys
import numpy as np
try:
    import iio
except:
    # By default the iio python bindings are not in path
    sys.path.append('/usr/lib/python2.7/site-packages/')
    import iio

import numpy as np
try:
    import matplotlib.pyplot as plt
    do_plots = True
except:
    print("To view plots install matplotlib")
    do_plots = False

# User configurable
TXLO = 1000000000
TXBW = 5000000
TXFS = 3000000
RXLO = TXLO
RXBW = TXBW
RXFS = TXFS

# Setup contexts
try:
    ctx = iio.Context('ip:192.168.2.1')
except:
    print("No device found")
    sys.exit(0)

ctrl = ctx.find_device("ad9361-phy")
txdac = ctx.find_device("cf-ad9361-dds-core-lpc")
rxadc = ctx.find_device("cf-ad9361-lpc")

# Configure transceiver settings
ctrl.channels[0].attrs["frequency"].value = str(int(RXLO))
ctrl.channels[1].attrs["frequency"].value = str(int(TXLO))
ctrl.channels[4].attrs["rf_bandwidth"].value = str(int(RXBW))
ctrl.channels[5].attrs["rf_bandwidth"].value = str(int(TXBW))
ctrl.channels[4].attrs["sampling_frequency"].value = str(int(RXFS))
ctrl.channels[5].attrs["sampling_frequency"].value = str(int(TXFS))
ctrl.channels[4].attrs['gain_control_mode'].value = 'slow_attack'
ctrl.channels[5].attrs['hardwaregain'].value = '-30'

# Enable all IQ channels
rxadc.channels[0].enabled = True
rxadc.channels[1].enabled = True
txdac.channels[4].enabled = True
txdac.channels[5].enabled = True

# Force DAC to use DMA not DDSs
txdac.channels[0].attrs['raw'].value = str(0)
txdac.channels[1].attrs['raw'].value = str(0)
txdac.channels[2].attrs['raw'].value = str(0)
txdac.channels[3].attrs['raw'].value = str(0)

# Create buffer for RX data
rxbuf = iio.Buffer(rxadc, 2**15, False)

# Create cyclic buffer for TX data
N = 2**15
txbuf = iio.Buffer(txdac, N/2, True)

# Create a sinewave waveform
fc = 10000
ts = 1/float(RXFS)
t = np.arange(0, N*ts, ts)
i = np.sin(2*np.pi*t*fc) * 2**14
q = np.cos(2*np.pi*t*fc) * 2**14
iq = np.empty((i.size + q.size,), dtype=i.dtype)
iq[0::2] = i
iq[1::2] = q
iq = np.int16(iq)

# Send data to buffer
txbuf.write(iq)
txbuf.push()

# Collect data
reals = np.array([])
imags = np.array([])

for k in range(10):
  rxbuf.refill()
  data = rxbuf.read()
  x = np.frombuffer(data,dtype=np.int16)
  reals = np.append(reals,x[::2])
  imags = np.append(imags,x[1::2])

# Plot
if do_plots:
    f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
    ax1.plot(reals)
    ax1.plot(imags)
    ax1.set_xlabel("Samples")
    ax1.set_ylabel("Amplitude [dbFS]")
    ax1.set_title('Received')
    ax2.plot(i)
    ax2.plot(q)
    ax2.set_xlabel("Samples")
    ax2.set_ylabel("Amplitude")
    ax2.set_title('Transmitted')
    plt.show()
