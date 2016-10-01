/**
 *  Beluga VRP Solver
 *	Copyright (c) 2005-2006 Claudio Procida. All rights reserved.
 *	http://www.emeraldion.it/
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	Bug fixes, suggestions and comments should be sent to:
 *	claudio@emeraldion.it
 */

/** grapher.h
 *
 *  header file for Beluga Grapher utility
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../beluga.h"

#define WIDTH 600 //!< Output SVG canvas width
#define HEIGHT 600 //!< Output SVG canvas height
#define PAD 20 //!< Output SVG canvas padding
#define NODE_RADIUS 14 //!< Radius of the node bullets
#define STROKE_WIDTH 7 //!< Thickness of the route path
#define NODE_STROKE "black" //!< Color of node disc
#define DEPOT_COLOR "red" //!< Color of a depot node bullet
#define CUSTOMER_COLOR "white" //!< Color of a customer node bullet
#define LABEL_TEXT "white"  //!< Color of the text in node labels
#define LABEL_BG NODE_STROKE  //!< Background color of node labels
#define LABEL_FONT "Courier New" //!< Font used in node labels

#ifndef MIN
#define MIN(x,y) (((x)<(y))?(x):(y)) //!< Macro to return the minimum between two quantities
#endif

/* Represent the VRP instance solution as a SVG file. */

void BEL_PrintSVG(BEL_VRPData *data, BEL_VRPSolution *solution);

