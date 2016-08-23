all: 
	@echo "Usage: make [linux|asd|rse|locomate|cobalt|imx|clean]"

linux     : $(TGT)
rse       : $(TGT)
asd       : $(TGT)
locomate  : $(TGT)
cobalt    : $(TGT)
imx       : $(TGT)

%.o: %.c
	${CC} ${CFLAGS} ${INCLUDES} $(LDFLAGS) $(LIBS) -c -o $@ $<

%.o: %.cpp
	${CC} $(INCLUDES) $(CFLAGS) $(LDFLAGS) $(LIBS) -c -o $@ $<

clean:
	-rm -f $(TGT) $(OBJ) *~ *.o
