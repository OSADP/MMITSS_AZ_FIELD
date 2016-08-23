
SDK_ROOT=../../../sdk
MAKE_ROOT=$(SDK_ROOT)/make
include $(MAKE_ROOT)/header.mk

SRC  = cgpxlogger.cpp ReqEntry.cpp ListHandle.cpp
TGT  = OBU_WebServer_v2_ack
LIBS+= -lstdc++ -luClibc++ -leloop -lgps -lgpsapi -lsavarimath 
OBJ = $(patsubst %.cpp, %.o, ${SRC})

include $(MAKE_ROOT)/rules.mk

$(TGT): $(OBJ)
	${CC} ${CFLAGS} ${INCLUDES} $(LDFLAGS) $(OBJ) -o $(TGT) $(LIBS)
