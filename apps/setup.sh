#!/bin/bash
#*****************************************************************************
# setup.sh file for setting up the cross compile building environment for 
#  building rpi kernel and the apps.
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
# 
# This file should be sourced to put into the environment variables for the 
# cross-compile build. 
# 
# the path to the cross compile tools is added to the PATH env var
# todo: change this path to something meaningful for you.
export PATH=$PATH:/media/arisena/datastore/public/jobs/yr2014/2200/proj/tools-master/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/

# the gcc prefix is defined as follows
export CCPREFIX=bcm2708-

# Cross Compiling & Building on host development machine with RPI kernel sources 
#
# Define KERNEL_SRC_ROOT to point to your RPI kernel sources and then follow the
# instructions below if you also want to build the kernel. If KERNEL_SRC_ROOT
# is defined then it i2s_test.c will use it to include the platform.h header 
# file, which has some useful defines. Otherwise, it uses copies from platform.h
# pasted into i2s_test.c 
#
# export the root to the kernel sources. The setup.sh script lives in the git_repo root and the sources
# are in git_repo/linux. Hence linux is appended to the current dir
# export KERNEL_SRC_ROOT=$(pwd)/linux

# from a running rpi you can get the .config file by doing:  zcat /proc/config.gz > .config

# move into the top dir of the kernel src tree
# this is the command clean
# make mrproper

# if any options have been changed then remake according to the config file
# make ARCH=arm CROSS_COMPILE=${CCPREFIX} oldconfig

# now build the kernel image
# make ARCH=arm CROSS_COMPILE=${CCPREFIX}

# now make the modules
# make ARCH=arm CROSS_COMPILE=${CCPREFIX} modules

# now install the modules to a directory outside the kernel tree
# setup a directory to some convenient location. This will create
#   $MODULES_TEMP/lib/modules/<kernel_build_ver>/build
# which is used in building modules.
# export MODULES_TEMP=<path_to_my_dir>/sdh_modules
# make ARCH=arm CROSS_COMPILE=${CCPREFIX} INSTALL_MOD_PATH=${MODULES_TEMP} modules_install

