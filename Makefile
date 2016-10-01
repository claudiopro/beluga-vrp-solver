#
# Beluga VRP Solver
# Copyright (c) 2005-2006 Claudio Procida. All rights reserved.
# http://www.emeraldion.it/
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# Bug fixes, suggestions and comments should be sent to:
# claudio@emeraldion.it
#
SOURCES=beluga.c optwriter.c binpacking.c capconloc.c datautils.c getdata.c
HEADERS=beluga.h
LIBRARIES=/usr/local/lib/libglpk.a /usr/local/lib/concorde.a /usr/local/lib/qsopt.a
CFLAGS=-O2
DEBUGFLAGS=-DDEBUG
OUTFILE=beluga

default: ${SOURCES} ${HEADERS}
	gcc -o $(OUTFILE) $(CFLAGS) ${SOURCES} $(LIBRARIES)

debug: ${SOURCES} ${HEADERS}
	gcc -o $(OUTFILE) $(DEBUGFLAGS) ${SOURCES} $(LIBRARIES)

clean:
	rm -rf *.o *.tmp *.exe
	rm -rf *.sol *.mipsol *.dat *.lp
	rm -rf *.mas *.sav *.pul
