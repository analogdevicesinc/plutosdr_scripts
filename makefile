all:
	gcc -g -o power power.c -liio -lm -Wall -Wextra
	gcc -g -o cal_ad9361 cal_ad9361.c -lfftw3 -lpthread -liio -lm -Wall -Wextra

