SRC = $(wildcard *.c)

LIB_NAME=libspatasn

CFLAGS = -Wall -DPRINT_LOG
INCLUDES = -I. -I../../include/${LIB_NAME}
OBJ = $(patsubst %.c, %.o, ${SRC})
.PHONY: all linux asd rse ${LIB_NAME} clean

linux: CC=gcc
linux: LD=ld
linux: AR=ar
linux: RANLIB=ranlib
linux: DEVICE=linux

asd: TOOLCHAIN=../../../asd/toolchain/
asd: DEVICE=asd

rse: TOOLCHAIN=../../../rse/toolchain/
rse: DEVICE=rse

asd rse: CC=$(TOOLCHAIN)/bin/i386-linux-uclibc-gcc
asd rse: LD=$(TOOLCHAIN)/bin/i386-linux-uclibc-ld
asd rse: AR=$(TOOLCHAIN)/bin/i386-linux-uclibc-ar
asd rse: RANLIB=$(TOOLCHAIN)/bin/i386-linux-uclibc-ranlib

TGT=${LIB_NAME}-${DEVICE}.a

all: 
	@echo "Usage: make [linux|asd|rse|clean]"

linux asd rse: clean ${LIB_NAME}

${LIB_NAME}: $(OBJ)
	${AR} ruv $(TGT) $(OBJ)
	${RANLIB} $(TGT)
	cp -v $(TGT) ../../lib/

%.o: %.c
	${CC} $(INCLUDES) $(CFLAGS) -c -o $@ $<

clean:
	-rm -f $(LIB_NAME)*.a $(OBJ) *~ *.o
