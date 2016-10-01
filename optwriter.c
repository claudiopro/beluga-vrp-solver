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

/** Prints a VRP solution to file
 *
 *  Prints the given BEL_VRPSolution to file as a standard tourfile.
 *  The solution is written as a list of ordered customer sets to be visited
 *  by each vehicle and the total travel cost of the routes.
 *
 *  This is an example optimal tour file:
 *
 *  <code>Route #1: 14 1 15 7 6 2 18 4</code><br>
 *  <code>Route #2: 13 11 17 9 5</code><br>
 *  <code>Route #3: 10 3 16 19 8 12</code><br>
 *  <code>cost 103</code>
 *
 *  @param sol  The BEL_VRPSolution to be printed
 *  @param optfname The name of the output tourfile
 *  @param verbose  Be verbose
 */

void BEL_PrintVRPSolution(BEL_VRPSolution *sol, char *optfname, int verbose)
{
  FILE *tourfile;
  if (!(tourfile = fopen(optfname, "w")))
  {
    printf("Error. Can't open file for writing. Aborting.\n");
    exit(1);
  }

  /* Output routes */
  int routes = sol->nvehicles;
  if (verbose)
    printf("Found %d routes\n", routes);
  int i, j, routelen;
  for (i = 0; i < routes; i++)
  {
    routelen = sol->routelen[i];
    if (verbose)
      printf("Route #%d has length %d\n", i, routelen);

    fprintf(tourfile, "Route #%d:", i + 1);
    for (j = 0; j < routelen; j++)
    {
	    fprintf(tourfile, " %d", sol->routes[i][j]);
	  }
    fprintf(tourfile, "\n");
  }
  fprintf(tourfile, "cost %d\n", sol->cost);
  fflush(tourfile);
  fclose(tourfile);
}
