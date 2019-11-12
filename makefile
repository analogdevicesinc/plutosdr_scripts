all:
	gcc -std=gnu99 -g -o power power.c -liio -lm -Wall -Wextra
	gcc -std=gnu99 -g -o cal_ad9361 cal_ad9361.c -lfftw3 -lpthread -liio -lm -Wall -Wextra
arm:
	# source /opt/Xilinx/SDK/2017.2/settings64.sh
	# arm-xilinx-linux-gnueabi-gcc -mfloat-abi=soft --sysroot=sysroot -I./include -Lsysroot/lib -std=gnu99 -g -o power_check power_check.c -lfftw3 -lpthread -liio -lm -Wall -Wextra
	# source /opt/Xilinx/SDK/2018.2/settings64.sh
	arm-linux-gnueabihf-gcc -mfloat-abi=hard  --sysroot=sysroot -std=gnu99 -g -o pluto_stream pluto_tx_file.c -liio -lm -Wall -Wextra
