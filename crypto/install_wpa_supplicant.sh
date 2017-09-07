#!/bin/bash

#install iproute2
cd /usr/local/src
git clone --depth 1 git://git.kernel.org/pub/scm/linux/kernel/git/shemminger/iproute2.git
cd iproute2/
./configure
make
make install

#install libnl
cd /usr/local/src
wget https://github.com/thom311/libnl/releases/download/libnl3_3_0/libnl-3.3.0.tar.gz
tar -xvf libnl-3.3.0.tar.gz
cd libnl-3.3.0
./configure
make
make install
cd ..
rm libnl-3.3.0.tar.gz

#install wpa_supplicant
cd /usr/local/src
#NEED TO REPLACE THIS WITH A DIFFERENT REPO NOT IN MY PRIVATE REPO
git clone https://github.com/emaclean-adi/wpa_supplicant.git
cd wpa_supplicant
cp ./wpa_supplicant.conf /etc/wpa_supplicant.conf
cd ./hostap/wpa_supplicant/
make
cp wpa_cli wpa_supplicant /usr/local/bin
cd /lib/arm-linux-gnueabihf/
rm libnl-3.so.200
ln -s /usr/local/lib/libnl-3.so.200 libnl-3.so.200
rm libnl-genl-3.so.200
ln -s /usr/local/lib/libnl-genl-3.so.200 libnl-genl-3.so.200
cd /usr/lib/arm-linux-gnueabihf/
rm libnl-route-3.so.200
ln -s /usr/local/lib/libnl-route-3.so.200 libnl-route-3.so.200
