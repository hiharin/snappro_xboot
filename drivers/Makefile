#
# (C) Copyright 2000
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

# CFLAGS += -DET_DEBUG -DDEBUG

LIB	= libdrivers.a
OBJS	= serial.o ns16550.o gpio.o omap3_dss.o lcd.o

ifeq ($(BOARD), omap3430sdp)
OBJS 	+= k9f1g08r0a.o onenand.o
endif

ifeq ($(BOARD), omap3430labrador)
OBJS 	+= k9f1g08r0a.o onenand.o
endif

ifeq ($(BOARD), omap3evm)
OBJS 	+= k9f1g08r0a.o onenand.o
endif

ifeq ($(BOARD), dm3730logic)
OBJS 	+= k9f1g08r0a.o nand_bch.o
endif

ifeq ($(BOARD), am3517evm)
OBJS    += k9f1g08r0a.o onenand.o
endif

ifeq ($(BOARD), omap2420h4)
OBJS 	+= k9k1216.o onenand.o
endif

ifeq ($(BOARD), omap2430sdp)
OBJS    += k9k1216.o onenand.o
endif

ifeq ($(BOARD), omap1710h3)
OBJS    += k9f5616.o onenand.o
endif


## Disabled for now:
##	  cs8900.o ct69000.o dataflash.o dc2114x.o ds1722.o \
##	  lan91c96.o mw_eeprom.o natsemi.o \
##	  smc91111.o smiLynxEM.o spi_eeprom.o sym53c8xx.o \
##

all:	$(LIB)

$(LIB):	$(OBJS)
	$(AR) crv $@ $(OBJS)

#########################################################################

.depend:	Makefile $(OBJS:.o=.c)
		$(CC) -M $(CFLAGS) $(OBJS:.o=.c) > $@

sinclude .depend

#########################################################################
