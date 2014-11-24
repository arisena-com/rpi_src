#*****************************************************************************
# Makefile for i2s_test test application
#  Cross compile make file. 
#
#  Copyright (C)  07 Nov 2014 Simon Derek Hughes
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along
#  with this program; if not, write to the Free Software Foundation, Inc.,
#  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
#  For more information, contact Simon Hughes at:
#    Email:    simon.d.hughes@arisena.com
#    Address:  Arisena, 11 Gough Way, Cambridge, CB3 9LN, UK.
#
#*****************************************************************************
# Notes
# This should be invoked after sourcing setup.sh to correctly define:
#	PATH				to include path to cross tools
#   CCPREFIX			prefix to canonical tool name
#   KERNEL_SRC_ROOT		path to kernel srcs (optional)

PROG=i2s_test
SRC=$(PROG).c

# remove no to enable debug trace
noCFLAGS+=-D_DEBUG -Dlinux

# Include the directory that contains the top level sources
KERNEL_INC=-I.
ifneq ($(strip $(KERNEL_SRC_ROOT)),)
KERNEL_INC+=-I$(KERNEL_SRC_ROOT)

# KERNEL_SRC_ROOT_DEF
#  define for c files to determine whether a path to the kernel src has 
#  been specified as an include dir.
KERNEL_SRC_ROOT_DEF=-DKERNEL_SRC_ROOT
endif

all:
	$(CCPREFIX)gcc $(KERNEL_INC) $(KERNEL_SRC_ROOT_DEF) $(CFLAGS) $(SRC) -o $(PROG)
	
clean:
	-rm $(PROG)
	
