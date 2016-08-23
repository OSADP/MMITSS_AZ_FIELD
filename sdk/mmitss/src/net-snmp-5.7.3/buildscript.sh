./configure \
	--prefix=/home/obesim/azv2i/MMITSS_Refreshed/sdk/mmitss \
	--with-default-snmp-version=3 \
	--with-sys-contact=@@no.where \
	--with-sys-location=Unknown \
	--with-logfile=/var/log/snmpd.log \
	--with-persistent-directory=/var/net-snmp \
	--with-cc=/home/obesim/azv2i/MMITSS_Refreshed/sdk/CodeSourcery/bin/powerpc-linux-gnu-gcc \
	--with-cflags=-DARP_SCAN_FOUR_ARGUMENTS \
	--with-ar=/home/obesim/azv2i/MMITSS_Refreshed/sdk/CodeSourcery/bin/powerpc-linux-gnu-ar \
	--with-ld=/home/obesim/azv2i/MMITSS_Refreshed/sdk/CodeSourcery/bin/powerpc-linux-gnu-ld \
	--build=ppc-linux \
	--host=powerpc

sleep 5 \ 

make -j4 && make install
