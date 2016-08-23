SDK_ROOT=../../../sdk
MAKE_ROOT=$(SDK_ROOT)/make
include $(MAKE_ROOT)/header.mk

SRC += cgpxlogger.cpp ReqEntry.cpp
TGT:=rsu_priorityWebserver
LIBS+= -lnetsnmp -lstdc++ -lm -lwme -leloop -lconffile -luClibc++ 

OBJ = $(patsubst %.cpp, %.o, ${SRC})

include $(MAKE_ROOT)/rules.mk

$(TGT): $(OBJ)
	${CC} ${CFLAGS} ${INCLUDES} $(LDFLAGS) $(OBJ) -o $(TGT) $(LIBS)
