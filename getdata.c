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

#include "beluga.h"

#define MAX_CAPACITY 100
#define MIN_CAPACITY 50
#define MAX_DEMAND 25
#define MIN_DEMAND 1

/** Creates the data to generate edge lengths in the dat structure.
 *
 *	The calling routine should be sure that dat points to
 *  a structure. If datname is NULL then random entries will be
 *  generated
 *
 *  @param datname The name of the datfile or the matrix file, if NULL
 *      then random data will be generated, according to the norm type.
 *      For D2 and D3 norms, the coordinates will be uniform between 0
 *      and ncount -1 (GEOGRAPHIC and GEOM norms have x between -90 and 90
 *      and y between -180 and 180). (For D2, the points will be distinct.)
 *      For MATRIX norms, the entries will be
 *      uniform between 0 and MATRAND_SCALE * ncount - 1 (currently
 *      10*ncount - 1. For CRYSTAL norms, a random matrix and bounds in
 *      range of the TSPLIB problems is generated - the wavelength is
 *      chosen to be 1.0, 1.35, or 1.70 depending on the ncount (but the
 *      problem will not be very close to hitting ncount.).
 *	@param binary_in Should be 1 if the datname file is in binary integers,
 *      or 2 if the datname file is in binary doubles.
 *  @param innorm The norm.
 *  @param ncount Will return the number of nodes. If datname is NULL, then
 *      ncount should be passed in with the number of nodes to be used in
 *      the random problem generation.
 *  @param dat Will contain the info to call the edgelen function.
 *  @param gridsize Specifies the size of the square grid random points are
 *      drawn from.
 *  @param allow_dups Indicates whether or not to allow duplicate points in
 *      the random point set.
 *  @param verbose Be verbose.
 *  @return 0 on success, 1 on failure.
 */

int BEL_VRPGetData(char *datname, int binary_in, int innorm, int *ncount, BEL_VRPData *data,
	int gridsize, int allow_dups, CCrandstate *rstate, int verbose)
{
		srand(utime());
    if (datname == (char *) NULL && *ncount == 0) {
        fprintf (stderr, "BEL_VRPGetData needs a datfile or a nodecount\n");
        return 1;
    }
    
    // Fill CCdatagroup
	int rval = CCutil_getdata (datname, binary_in, innorm,
						ncount, data->dat, gridsize, allow_dups, rstate);
						
	if (rval)
	{
		fprintf(stderr, "CCutil_getdata returned an error: %d\n", rval);
		return 1;
	}
	if (verbose)
 		printf("Generating %d random demands\n", *ncount);
	data->capacity = MIN_CAPACITY + (rand() % (MAX_CAPACITY - MIN_CAPACITY));
	data->dimension = *ncount;
	data->ndepots = 1;
	data->ncustomers = *ncount - 1;

	data->demand = (int *)calloc(*ncount, sizeof(int));
	data->isadepot = (int *)calloc(*ncount, sizeof(int));
	data->depots = (int *)calloc(1, sizeof(int));

	int i;
	data->demand[0] = 0;
	data->isadepot[0] = 1;
	data->depots[0] = 0;
#ifdef DEBUG
	printf("i:%d, demand:%d, isadepot:%d\n", 0, data->demand[0], data->isadepot[0]);
#endif
	for (i = 1; i < *ncount; i++)
	{
		data->demand[i] = MIN_DEMAND + (rand() % (MAX_DEMAND - MIN_DEMAND));
		data->depots[i] = 0;
#ifdef DEBUG
		printf("i:%d, demand:%d, isadepot:%d\n", i, data->demand[i], data->isadepot[i]);
#endif
	}

	
	return 0;
}
