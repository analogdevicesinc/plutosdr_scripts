import numpy as np

import sys
try:
    import iio
except:
    # By default the iio python bindings are not in path
    sys.path.append('/usr/lib/python2.7/site-packages/')
    import iio


class Pluto:

  def __init__(self, uri=None, rx_lo=1000000000, tx_lo=1000000000, \
    sample_rate=5000000, rx_rf_bandwidth=3000000, tx_rf_bandwidth=3000000, \
    rx_hardwaregain=30, tx_hardwaregain=-10, gain_control_mode='slow_attack'):
      self.uri = uri

      # Initialize context
      self.ctx = None
      try:
          if uri:
              self.ctx = iio.Context(self.uri)
          else:
              contexts = iio.scan_contexts()
              for uri in contexts:
                  if 'PlutoSDR' in contexts[uri]:
                      self.ctx = iio.Context(uri)
              if self.ctx==None:
                  raise Exception()
      except:
          print("No PlutoSDR device found")
          sys.exit(0)

      self.ctrl = self.ctx.find_device("ad9361-phy")
      self.txdac = self.ctx.find_device("cf-ad9361-dds-core-lpc")
      self.rxadc = self.ctx.find_device("cf-ad9361-lpc")

      self.rx_lo = rx_lo
      self.tx_lo = tx_lo
      # self.sample_rate = sample_rate
      self.rx_rf_bandwidth = rx_rf_bandwidth
      self.tx_rf_bandwidth = tx_rf_bandwidth

      self.gain_control_mode = gain_control_mode
      self.rx_hardwaregain = rx_hardwaregain
      self.tx_hardwaregain = tx_hardwaregain

      self.rxbuf = None

  def set_iio_attr_str(self,channel_name,attr_name,output,value):
      channel = self.ctrl.find_channel(channel_name, output)
      try:
          channel.attrs[attr_name].value = str(value)
      except Exception as ex:
          raise ex

  def set_iio_attr(self,channel_name,attr_name,output,value):
      channel = self.ctrl.find_channel(channel_name, output)
      try:
          channel.attrs[attr_name].value = str(int(value))
      except Exception as ex:
          raise ex

  def get_iio_attr(self,channel_name,attr_name,output):
      channel = self.ctrl.find_channel(channel_name, output)
      return channel.attrs[attr_name].value

  @property
  def gain_control_mode(self):
      return self.get_iio_attr("voltage0","gain_control_mode",False)

  @gain_control_mode.setter
  def gain_control_mode(self,value):
      self.set_iio_attr_str("voltage0","gain_control_mode",False,value)

  @property
  def rx_hardwaregain(self):
      return self.get_iio_attr("voltage0","hardwaregain",False)

  @rx_hardwaregain.setter
  def rx_hardwaregain(self,value):
      if self.gain_control_mode == 'manual':
          self.set_iio_attr("voltage0","hardwaregain",False,value)

  @property
  def tx_hardwaregain(self):
      return self.get_iio_attr("voltage0","hardwaregain",True)

  @tx_hardwaregain.setter
  def tx_hardwaregain(self,value):
      self.set_iio_attr("voltage0","hardwaregain",True,value)

  @property
  def rx_rf_bandwidth(self):
      return self.get_iio_attr("voltage0","rf_bandwidth",False)

  @rx_rf_bandwidth.setter
  def rx_rf_bandwidth(self,value):
      self.set_iio_attr("voltage0","rf_bandwidth",False,value)

  @property
  def tx_rf_bandwidth(self):
      return self.get_iio_attr("voltage0","rf_bandwidth",True)

  @tx_rf_bandwidth.setter
  def tx_rf_bandwidth(self,value):
      self.set_iio_attr("voltage0","rf_bandwidth",True,value)

  @property
  def sample_rate(self):
      return self.get_iio_attr("voltage0","sampling_frequency",False)

  @sample_rate.setter
  def sample_rate(self,value):
      self.set_iio_attr("voltage0","sampling_frequency",False,value)

  @property
  def rx_lo(self):
      return self.get_iio_attr("altvoltage0","frequency",True)

  @rx_lo.setter
  def rx_lo(self,value):
      self.set_iio_attr("altvoltage0","frequency",True,value)

  @property
  def tx_lo(self):
      return self.get_iio_attr("altvoltage1","frequency",True)

  @tx_lo.setter
  def tx_lo(self,value):
      self.set_iio_attr("altvoltage1","frequency",True,value)

  def rx(self):
      if not self.rxbuf:
          # Enable all IQ channels
          v0 = self.rxadc.find_channel("voltage0")
          v1 = self.rxadc.find_channel("voltage1")
          v0.enabled = True
          v1.enabled = True
          self.rxbuf = iio.Buffer(self.rxadc, 2**15, False)
      self.rxbuf.refill()
      data = self.rxbuf.read()
      x = np.frombuffer(data,dtype=np.int16)
      sig = x[::2] + 1j * x[1::2]
      return sig

if  __name__ == "__main__":
    sdr = Pluto()
    print("RX LO %s" % (sdr.rx_lo))
    sdr.rx_lo = 2000000000
    sig = sdr.rx()
