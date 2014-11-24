#!/bin/bash

#*****************************************************************************
# i2s_test4.sh
#  test script to document some command line examples for v1.0 of i2s_test 
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
# Versions
#  v0.1    20141107    SDH   first version    
#*****************************************************************************

PROG=i2s_test

# This is more for documentation purposes rather than running, but could be converted into a 
# useful script. Uncomment/recommend lines as your interest dictates.

# Get help. Uncomment next line to see help message
${PROG} -h

# see test_vector_table[] in src file for {src, mash, divi, divf} tuples for different clock 
# sources

# Generate I2S_BCLK (aka PCM_CLK) at 512KHz
# clock src = PLLD(500MHz), num mash stages = 1, integer divider = 1951, fractional divider=2176
#${PROG} -s 6 -m 1 -i 1951 -f 2176

# Test vector 6 is equivalent to the previous command line options
#${PROG} -t 6

 
# Generate I2S_BCLK (aka PCM_CLK) at 512KHz
# clock src = PLLD(500MHz), num mash stages = 1, integer divider = 974, fractional divider=2624
#${PROG} -s 6 -m 1 -i 974 -f 2624

# Test vector 7 is equivalent to the previous command line options
#${PROG} -t 7
 

# Generate I2S_BCLK (aka PCM_CLK) at 1.536MHz
# clock src = PLLD(500MHz), num mash stages = 1, integer divider = 323, fractional divider=2581
#${PROG} -s 6 -m 1 -i 323 -f 2581

# Test vector 8 is equivalent to the previous command line options
#${PROG} -t 8
