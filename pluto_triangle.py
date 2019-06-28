#!/usr/bin/env python
#
# Copyright (C) 2019 Analog Devices, Inc.
# Author: Travis Collins <travis.collins@analog.com>
#
# Licensed under the GPL-2.

# Triangle example: This example demonstrates how to verify data that is sent to the DMA
# by using digital loopback and a triangle signal

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

ctrl.debug_attrs['loopback'].value = '1'

# Configure transceiver settings
ctrl.find_channel('RX_LO').attrs["frequency"].value = str(int(RXLO))
ctrl.find_channel('TX_LO').attrs["frequency"].value = str(int(TXLO))
ctrl.find_channel('voltage0').attrs["rf_bandwidth"].value = str(int(RXBW))
ctrl.find_channel('voltage0',True).attrs["rf_bandwidth"].value = str(int(TXBW))
ctrl.find_channel('voltage0').attrs["sampling_frequency"].value = str(int(RXFS))
ctrl.find_channel('voltage0',True).attrs["sampling_frequency"].value = str(int(TXFS))
ctrl.find_channel('voltage0').attrs['gain_control_mode'].value = 'slow_attack'
ctrl.find_channel('voltage0',True).attrs['hardwaregain'].value = '-30'

# Enable all IQ channels
rxadc.find_channel("voltage0").enabled = True
rxadc.find_channel("voltage1").enabled = True
txdac.find_channel("voltage0",True).enabled = True
txdac.find_channel("voltage1",True).enabled = True

# Force DAC to use DMA not DDS
txdac.find_channel('TX1_I_F1',True).attrs['raw'].value = str(0)
txdac.find_channel('TX1_Q_F1',True).attrs['raw'].value = str(0)
txdac.find_channel('TX1_I_F2',True).attrs['raw'].value = str(0)
txdac.find_channel('TX1_Q_F2',True).attrs['raw'].value = str(0)

# Create buffer for RX data
rxbuf = iio.Buffer(rxadc, 2**12, False)

# Create ramp
N = 2**10 # Dont exceed 2**11-1
a = np.int16(range(int(N)))
a = np.concatenate((a, a[::-1]), axis=0)
a = a << 4; # Shift to prevent interface core cast scale reduction
iq = np.empty((a.size*2,), dtype=np.int16)
iq[0::2] = a
iq[1::2] = a
iq = np.int16(iq)

# Create cyclic buffer for TX data
samples_per_channel = iq.size//2
txbuf = iio.Buffer(txdac, samples_per_channel, True)

# Send data to buffer
txbuf.write(bytearray(iq))
txbuf.push()

# Collect data
reals = np.array([])
imags = np.array([])

for k in range(10):
  rxbuf.refill()
  data = rxbuf.read()
  x = np.frombuffer(data,dtype=np.int16)
  reals = x[::2]
  imags = x[1::2]

# Plot
if do_plots:
    plt.plot(reals)
    plt.plot(a >> 4)
    plt.xlabel("Samples")
    plt.ylabel("Amplitude")
    plt.show()
