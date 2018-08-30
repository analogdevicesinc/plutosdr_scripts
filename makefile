all:
	gcc -std=gnu99 -g -o power power.c -liio -lm -Wall -Wextra
	gcc -std=gnu99 -g -o cal_ad9361 cal_ad9361.c -lfftw3 -lpthread -liio -lm -Wall -Wextra

