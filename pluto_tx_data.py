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
rxbuf = iio.Buffer(rxadc, 2**15, False)

# Create cyclic buffer for TX data
N = 2**15
txbuf = iio.Buffer(txdac, int(N/2), True)

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
