#!/bin/bash

ip=$(</tmp/modem-ip)
macsecen=$(</tmp/enc-onoff)


if [ "$macsecen" = "1" ]; then
   sudo ifconfig macsec0 $ip
   sudo ifconfig adi_radio 192.168.24.1
else
   sudo ifconfig adi_radio $ip
   sudo ifconfig macsec0 192.168.24.1
fi
