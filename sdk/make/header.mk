SRC = 
LIB = 
OBJ = 

CFLAGS += -Wall -pedantic -Wextra -g -DPRINT_LOG
INCLUDES = -I. -I$(SDK_ROOT)/mmitss/include/libj2735
LIBS = -L$(SDK_ROOT)/mmitss/lib/

.PHONY: all linux asd rse locomate cobalt clean

linux: CC=gcc
linux: LD=ld
linux: AR=ar
linux: RANLIB=ranlib
linux: DEVICE=linux
linux: LIBS+=-lj2735-linux

asd: TOOLCHAIN=$(SDK_ROOT)/savari/asd/toolchain/
asd: DEVICE=asd
asd: CC=$(TOOLCHAIN)/bin/i386-linux-uclibc-gcc
asd: LD=$(TOOLCHAIN)/bin/i386-linux-uclibc-ld
asd: AR=$(TOOLCHAIN)/bin/i386-linux-uclibc-ar
asd: RANLIB=$(TOOLCHAIN)/bin/i386-linux-uclibc-ranlib
asd: INCLUDES+=-I$(TOOLCHAIN)/include -I$(TOOLCHAIN)/usr/include
asd: LIBS+=-L$(TOOLCHAIN)/lib -L$(TOOLCHAIN)/usr/lib -luClibc++ -lj2735-asd

rse: TOOLCHAIN=$(SDK_ROOT)/savari/rse/toolchain/
rse: DEVICE=rse
rse: CC=$(TOOLCHAIN)/bin/i386-linux-uclibc-gcc
rse: LD=$(TOOLCHAIN)/bin/i386-linux-uclibc-ld
rse: AR=$(TOOLCHAIN)/bin/i386-linux-uclibc-ar
rse: RANLIB=$(TOOLCHAIN)/bin/i386-linux-uclibc-ranlib
rse: INCLUDES+=-I$(TOOLCHAIN)/include -I$(TOOLCHAIN)/usr/include
rse: LIBS+=-L$(TOOLCHAIN)/lib -L$(TOOLCHAIN)/usr/lib -luClibc++ -lj2735-rse

locomate: TOOLCHAIN=/opt/buildroot-2013.11/output/host/usr
locomate: DEVICE=locomate
locomate: CC=$(TOOLCHAIN)/bin/mips-linux-gcc
locomate: LD=$(TOOLCHAIN)/bin/mips-linux-ld
locomate: AR=$(TOOLCHAIN)/bin/mips-linux-ar
locomate: RANLIB=$(TOOLCHAIN)/bin/mips-linux-ranlib
locomate: INCLUDES+=-I$(TOOLCHAIN)/include -I$(TOOLCHAIN)/usr/include -I/usr/local/include/
locomate: LIBS+=-L$(TOOLCHAIN)/lib -L$(TOOLCHAIN)/usr/lib -lj2735-locomate
	export LD_LIBRARY_PATH=/opt/buildroot-2013.11/output/host/usr/lib

cobalt: TOOLCHAIN=$(SDK_ROOT)/econolite/cobalt
cobalt: DEVICE=cobalt
cobalt: CC=$(TOOLCHAIN)/bin/powerpc-econolite_e300c2-linux-gnu-gcc
cobalt: LD=$(TOOLCHAIN)/bin/powerpc-econolite_e300c2-linux-gnu-ld
cobalt: AR=$(TOOLCHAIN)/bin/powerpc-econolite_e300c2-linux-gnu-ar
cobalt: RANLIB=$(TOOLCHAIN)/bin/powerpc-econolite_e300c2-linux-gnu-ranlib
cobalt: INCLUDES+=-I$(TOOLCHAIN)/powerpc-econolite_e300c2-linux-gnu/include -I$(TOOLCHAIN)/powerpc-econolite_e300c2-linux-gnu/usr/include
cobalt: LIBS+=-L$(TOOLCHAIN)/powerpc-econolite_e300c2-linux-gnu/lib -L$(TOOLCHAIN)/powerpc-econolite_e300c2-linux-gnu/usr/lib -lj2735-econolite

imx: TOOLCHAIN=$(SDK_ROOT)/econolite/imx
imx: DEVICE=imx
imx: CC=$(TOOLCHAIN)/bin/arm-none-linux-gnueabi-gcc
imx: LD=$(TOOLCHAIN)/bin/arm-none-linux-gnueabi-ld
imx: AR=$(TOOLCHAIN)/bin/arm-none-linux-gnueabi-ar
imx: RANLIB=$(TOOLCHAIN)/bin/arm-none-linux-gnueabi-ranlib
imx: INCLUDES+=-I$(TOOLCHAIN)/arm-none-linux-gnueabi/libc/usr/include -I$(TOOLCHAIN)/arm-none-linux-gnueabi/include/c++
imx: LIBS+= -L$(TOOLCHAIN)/arm-none-linux-gnueabi/libc/usr/lib -L$(TOOLCHAIN)/arm-none-linux-gnueabi/libc/lib -L$(TOOLCHAIN)/arm-none-linux-gnueabi/lib -lj2735-imx
