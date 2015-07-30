SRC =   
SRC += main.cpp bsmd.cpp srmd.cpp wme.cpp gps.cpp

TGT:=MMITSS_OBE_BSMData_Transmitter
TOOLCHAIN=../../../sdk/asd/toolchain

SAVARI_CC=$(TOOLCHAIN)/bin/i386-linux-uclibc-g++
SAVARI_LD=$(TOOLCHAIN)/bin/i386-linux-uclibc-ld
CFLAGS:= -Wall -DPRINT_LOG
LDFLAGS:=
INCLUDES:=-I. -I$(TOOLCHAIN)/include -I$(TOOLCHAIN)/usr/include
LIBS =-L$(TOOLCHAIN)/lib -L$(TOOLCHAIN)/usr/lib -luClibc++ -leloop -lwme -lradio -lstdc++ -lj2735 -lsae -lm -lgps -lgpsapi -lsavarimath

OBJ = $(patsubst %.cpp, %.o, ${SRC})

LIBS +=
all:	$(TGT)

$(TGT): $(OBJ)
	${SAVARI_CC}  $(LDFLAGS) $(OBJ) -o $(TGT) $(LIBS)

%.o: %.cpp
	${SAVARI_CC} $(INCLUDES) $(CFLAGS) -c -o $@ $<

clean:
	-rm -f $(TGT) $(OBJ) *~ *.o
