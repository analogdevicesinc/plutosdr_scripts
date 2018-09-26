#!/usr/bin/env python
#
# Copyright (C) 2018 Analog Devices, Inc.
# Author: Travis Collins <travis.collins@analog.com>
#
# Licensed under the GPL-2.

import sys
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
txdac.channels[0].enabled = True
txdac.channels[1].enabled = True

# Create buffer for RX data
rxbuf = iio.Buffer(rxadc, 2**15, False)

# Enable single tone DDS
txdac.channels[0].attrs['raw'].value = str(1)
txdac.channels[0].attrs['frequency'].value = str(100000)
txdac.channels[0].attrs['scale'].value = str(0.9)
txdac.channels[0].attrs['phase'].value = str(90000)
txdac.channels[2].attrs['raw'].value = str(1)
txdac.channels[2].attrs['frequency'].value = str(100000)
txdac.channels[2].attrs['scale'].value = str(0.9)
txdac.channels[2].attrs['phase'].value = str(0)

# Collect data
reals = np.array([])
imags = np.array([])

for i in range(10):
  rxbuf.refill()
  data = rxbuf.read()
  x = np.frombuffer(data,dtype=np.int16)
  reals = np.append(reals,x[::2])
  imags = np.append(imags,x[1::2])

# Plot
if do_plots:
    plt.plot(reals)
    plt.plot(imags)
    plt.xlabel("Samples")
    plt.ylabel("Amplitude [dbFS]")
    plt.show()
