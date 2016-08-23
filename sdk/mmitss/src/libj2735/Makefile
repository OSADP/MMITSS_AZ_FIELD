SDK_ROOT=../../../
MAKE_ROOT=$(SDK_ROOT)/make
include $(MAKE_ROOT)/header.mk

SRC = $(wildcard *.c)
LIB_NAME=libj2735
TGT=${LIB_NAME}-${DEVICE}.a

OBJ = $(patsubst %.c, %.o, ${SRC})


include $(MAKE_ROOT)/rules.mk

${TGT}: $(OBJ)
	${AR} ruv $(TGT) $(OBJ)
	${RANLIB} $(TGT)
	cp -v $(TGT) ../../lib/
