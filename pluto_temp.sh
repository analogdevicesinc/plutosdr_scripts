#!/bin/sh

if [ -z ${1} ] ; then
  #Find USB?
  uri=$(iio_info -s | grep PLUTO | \
	sed -e 's/ /\n/g' -e 's:\[::g' -e 's:\]::g' | grep usb)
  if [ -z ${uri} ] ; then
    #Not USB, try zeroconf
    command avahi-resolve  >/dev/null 2>&1
    if [ "$?" -eq "1" ] ; then
      ip=$(avahi-resolve --name pluto.local | awk '{print $2}')
      uri=$(echo "-u ip:${ip}")
    fi
  else
    uri=$(echo "-u ${uri}")
  fi
else
  if [ "${1}" = "-h" ] ; then
    echo $0 [uri] [number loops] [delay in seconds]
    exit
  fi
  uri=$(echo "-u ${1}")
fi
if [ -z "${uri}" ] ; then
  echo "could not find plutosdr"
  echo "use $0 ip:192.168.2.1 [number loops] [delay in seconds]"
  exit
else
  iio_info $uri > /dev/null 2>&1
  if [ "$?" -eq "1" ] ; then
    echo Did not find a uri at \'${uri}\'
    echo "use $0 ip:192.168.2.1 [number loops] [delay in seconds]"
    exit
  fi
  echo using uri $uri
fi

device=$(iio_attr -${uri} -C | grep PlutoSDR)
if [ -z "${device}" ] ; then
  echo "not a PlutoSDR at ${uri}"
  exit
fi

if [ -z "${2}" ] ; then
  i=1
else
  i=$2
fi

if [ -z "${3}" ] ; then
  delay=1
else
  delay=$3
fi

while [ $i -ne 0 ] ; do
  pluto_temp=$(iio_attr -q ${uri} -c ad9361-phy temp0 input)
  pluto=$(awk "BEGIN {printf(\"%.1f\", ${pluto_temp} / 1000)}")

  xadc_raw=$(iio_attr -q ${uri} -c xadc temp0 raw)
  xadc_offset=$(iio_attr -q ${uri} -c xadc temp0 offset)
  xadc_scale=$(iio_attr -q ${uri} -c xadc temp0 scale)
  xadc=$(awk "BEGIN {printf(\"%.1f\", (${xadc_raw} + ${xadc_offset}) * ${xadc_scale}/1000)}")

  echo "pluto: ${pluto} °C      zynq: ${xadc} °C"
  i=$(expr $i - 1)
  if [ $i -ne 0 ] ; then
    sleep ${delay}
  fi
done
