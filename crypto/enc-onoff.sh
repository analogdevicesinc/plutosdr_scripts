#!/bin/bash

# "enables/disables crypto", this just assigns the macsec0 device the
# IP address that the modem used so that traffic will flow through it 
# instead of adi_radio when enabling and changes it back when disabling
# This was done this way to minimize impact on the rest of the demo.

ip=$(</tmp/modem-ip)
macsecen=$(</tmp/enc-onoff)


if [ "$macsecen" = "1" ]; then
   sudo ifconfig macsec0 $ip
   sudo ifconfig adi_radio 192.168.24.1
else
   sudo ifconfig adi_radio $ip
   sudo ifconfig macsec0 192.168.24.1
fi
