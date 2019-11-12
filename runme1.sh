#!/bin/sh

cd /media/sda1/

iio_attr -a -c ad9361-phy altvoltage0 frequency 1000000000
iio_attr -a -c ad9361-phy altvoltage1 frequency 1000000000

./pluto_tx_file
