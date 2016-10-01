#!/bin/bash

##
#	Beluga VRP Solver
#	Copyright (c) 2005-2006 Claudio Procida. All rights reserved.
#	http://www.emeraldion.it/
#
#	This program is free software; you can redistribute it and/or modify
#	it under the terms of the GNU General Public License as published by
#	the Free Software Foundation; either version 2 of the License, or
#	(at your option) any later version.
#
#	This program is distributed in the hope that it will be useful,
#	but WITHOUT ANY WARRANTY; without even the implied warranty of
#	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#	GNU General Public License for more details.
#
#	Bug fixes, suggestions and comments should be sent to:
#	claudio@emeraldion.it
##

echo "This is Beluga installer script"

# Headers directory
INCLUDEDIR=/usr/local/include/
# Libraries directory
LIBDIR=/usr/local/lib/
COPY="cp -R"
HEADERS="include/*"
LIBRARIES="include/*"
MAKE=make

echo "This is Beluga installer script"

echo "Copying headers..."
# Copy headers to headers directory
$COPY $HEADERS $INCLUDEDIR
echo "done"

echo "Copying libraries..."
# Copy libraries to libraries directory
$COPY $LIBRARIES $LIBDIR
echo "done"

echo "Compiling Beluga..."
$MAKE
echo "done"

echo "Beluga installed successfully"
exit 0
