import adi
import matplotlib.pyplot as plt
import numpy as np

# Create radio
sdr = adi.Pluto()

# Configure properties
sdr.rx_rf_bandwidth = 4000000
sdr.rx_lo = 2000000000

# Read properties
print("RX LO %s" % (sdr.rx_lo))

# Collect data
sig = sdr.rx()
sig = sdr.rx()
plt.plot(np.real(sig))
plt.plot(np.imag(sig))
plt.xlabel("Samples")
plt.ylabel("Amplitude [dbFS]")
plt.show()
