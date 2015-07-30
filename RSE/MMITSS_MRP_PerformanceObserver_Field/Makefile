SRC =   
SRC += MMITSS_rsu_PerformanceObserver.cpp AccelerationSet4Way.cpp BasicVehicle.cpp Brakes.cpp GeoCoord.cpp NMAP.cpp PositionLocal3D.cpp Array.cpp Signal.cpp Config.cpp

TGT:=MMITSS_MRP_PerformanceObserver_Field
TOOLCHAIN=../../../sdk/rse/toolchain

SAVARI_CC=$(TOOLCHAIN)/bin/i386-linux-uclibc-g++
SAVARI_LD=$(TOOLCHAIN)/bin/i386-linux-uclibc-ld
CFLAGS:= -Wall -DPRINT_LOG
LDFLAGS:=
INCLUDES:=-I. -I$(TOOLCHAIN)/include -I$(TOOLCHAIN)/usr/include
LIBS =-L$(TOOLCHAIN)/lib -L$(TOOLCHAIN)/usr/lib -lstdc++ -luClibc++ -leloop -lnetsnmp -lgps -lgpsapi

OBJ = $(patsubst %.cpp, %.o, ${SRC})

LIBS +=
all:	$(TGT)
$(TGT): $(OBJ)
	${SAVARI_CC}  $(LDFLAGS) $(OBJ) -o $(TGT) $(LIBS)

%.o: %.cpp
	${SAVARI_CC} $(INCLUDES) $(CFLAGS) -c -o $@ $<

clean:
	-rm -f $(TGT) $(OBJ) *~ *.o
