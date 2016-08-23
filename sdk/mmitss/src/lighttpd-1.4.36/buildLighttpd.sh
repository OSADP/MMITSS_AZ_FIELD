export LD=arm-none-linux-gnueabi-ld
export AR=arm-none-linux-gnueabi-ar
export RANLIB=arm-none-linux-gnueabi-ranlib
export STRIP=arm-none-linux-gnueabi-strip
export PATH=$PATH:/home/obesim/azv2i/MMITSS_Refreshed/sdk/imx/bin
export CFLAGS=-I/home/obesim/azv2i/MMITSS_Refreshed/sdk/imx/include
export LIBS=-L/home/obesim/azv2i/MMITSS_Refreshed/sdk/imx/lib
export CC="arm-none-linux-gnueabi-gcc" # $CFLAGS $LIBS"
./configure \
	--host=arm-none-linux-gnueabi \
	--prefix=/home/obesim/azv2i/MMITSS_Refreshed/sdk/imx/arm-none-linux-gnueabi/libc/usr/ \
	--exec-prefix=/usr/ \
	--without-zlib --without-bzip2 && \
make clean && make -j16
