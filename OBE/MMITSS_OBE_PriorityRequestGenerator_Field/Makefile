SRC =   
SRC += MMITSS_OBE_PriorityRequestGenerator.cpp GeoCoord.cpp NMAP.cpp AccelerationSet4Way.cpp BasicVehicle.cpp Motion.cpp PositionLocal3D.cpp Brakes.cpp ReqEntry.cpp MAPEntry.cpp

TGT:=MMITSS_OBE_PriorityRequestGenerator_Field
TOOLCHAIN=../../../sdk/asd/toolchain

SAVARI_CC=$(TOOLCHAIN)/bin/i386-linux-uclibc-g++
SAVARI_LD=$(TOOLCHAIN)/bin/i386-linux-uclibc-ld
CFLAGS:= -Wall -DPRINT_LOG
LDFLAGS:=
INCLUDES:=-I. -I$(TOOLCHAIN)/include -I$(TOOLCHAIN)/usr/include
LIBS =-L$(TOOLCHAIN)/lib -L$(TOOLCHAIN)/usr/lib -lstdc++ -luClibc++ -leloop -lnetsnmp -lj2735 -lsae -lm -lgps -lgpsapi -lsavarimath

OBJ = $(patsubst %.cpp, %.o, ${SRC})

LIBS +=
all:	$(TGT)
$(TGT): $(OBJ)
	${SAVARI_CC}  $(LDFLAGS) $(OBJ) -o $(TGT) $(LIBS)

%.o: %.cpp
	${SAVARI_CC} $(INCLUDES) $(CFLAGS) -c -o $@ $<

clean:
	-rm -f $(TGT) $(OBJ) *~ *.o
