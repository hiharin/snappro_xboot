#!/bin/bash
make CROSS_COMPILE=arm-none-linux-gnueabi- distclean && make CROSS_COMPILE=arm-none-linux-gnueabi- dm3730logic_config && make CROSS_COMPILE=arm-none-linux-gnueabi- MLO && cp -v MLO /tftpboot/rootfs/
