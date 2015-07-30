SRC =   
SRC += MMITSS_PED_MAP_BROADCAST.cpp  NMAP.cpp

TGT:=MMITSS_PED_MAP_BROADCAST
TOOLCHAIN=../../../sdk/rse/toolchain
MMITSS=../../../sdk/mmitss

SAVARI_CC=$(TOOLCHAIN)/bin/i386-linux-uclibc-g++
SAVARI_LD=$(TOOLCHAIN)/bin/i386-linux-uclibc-ld
CFLAGS:=  -DPRINT_LOG -g
LDFLAGS:=
INCLUDES:=-I. -I$(TOOLCHAIN)/include -I$(TOOLCHAIN)/usr/include -I$(MMITSS)/include
LIBS =-L$(TOOLCHAIN)/lib -L$(TOOLCHAIN)/usr/lib -lnetsnmp -lstdc++ -luClibc++ -leloop -lm -lj2735 -lsae -lgps -lgpsapi
LIBS+=-L$(MMITSS)/lib -lspatasn-rse

OBJ = $(patsubst %.cpp, %.o, ${SRC})

LIBS +=
all:	$(TGT)
$(TGT): $(OBJ)
	${SAVARI_CC}  $(LDFLAGS) $(OBJ) -o $(TGT) $(LIBS)

%.o: %.cpp
	${SAVARI_CC} $(INCLUDES) $(CFLAGS) -c -o $@ $<

clean:
	-rm -f $(TGT) $(OBJ) *~ *.o
