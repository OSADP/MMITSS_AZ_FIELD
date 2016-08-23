export PATH=$PATH:/home/obesim/azv2i/MMITSS_Refreshed/sdk/imx/bin
export CC=arm-none-linux-gnueabi-gcc
export LD=arm-none-linux-gnueabi-ld
export AR=arm-none-linux-gnueabi-ar
export RANLIB=arm-none-linux-gnueabi-ranlib
export STRIP=arm-none-linux-gnueabi-strip
./configure \
	--host=arm-none-linux-gnueabi \
	--build=x86_64-linux \
	--prefix=/home/obesim/azv2i/MMITSS_Refreshed/sdk/imx/arm-none-linux-gnueabi/libc/usr/ \
make clean && make -j16
