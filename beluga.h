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

/** beluga.h
 *
 *  header file for Beluga VRP solver
 *
 */

#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <glpk.h>
#include <concorde.h>

/* How many nonzero elements we allow for a MIP instance */
#define MAX_NONZEROES 100000
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define BEL_VRP_SOLVED                (1)
#define BEL_VRP_INFEASIBLE            (2)
#define BEL_VRP_NOT_ENOUGH_VEHICLES   (6)


/**
 * Data Types
 */

typedef char BOOL; //!< Boolean type

#define TRUE 1	//!< Boolean TRUE
#define FALSE 0	//!< Boolean FALSE

/** A structure to hold the solution of VRP problem.
 *
 *	A solution is described by the ordered list of nodes to be visited
 *	by each vehicle, the number of vehicles and the total cost.
 *
 */

typedef struct BEL_VRPSolution {

	int **routes;		//!< A variable length array of routes. A route is an array of customers.
	int *routelen;	//!< An array that holds the length of the corresponding route.
	int cost;				//!< The total cost of the solution.
	int nvehicles;	//!< Number of vehicles needed.
	
} BEL_VRPSolution;

/** A structure to hold VRP Problem data.
 *
 *	This is an extension of the data structure used by Concorde,
 *  to accommodate additional data of a VRP problem instance.
 *
 */

typedef struct BEL_VRPData {

	char *name;				//!< The name of the instance. Used to save intermediate and output files.
	char *comment;		//!< A brief description of this instance.
	CCdatagroup *dat;	//!< This is the original node allocation structure from Concorde.
	int *demand;			//!< Array of demand for actual customers.
	int *isadepot;		//!< Array of flags. A 1 at position x denotes that node x is a depot.
	int *depots;			//!< Array of ndepots nodes that are depots.
	int capacity;			//!< Vehicle capacity for CVRP.
	int dimension;		//!< The dimension of the instance, that is the number of nodes.
	int ndepots;			//!< Number of depots.
	int ncustomers;		//!< Number of customers (just dimension - ndepots).
	int nvehicles;		//!< Number of available vehicles (usually not set).

} BEL_VRPData;


/* VRP Data handling */

/* Initializes a BEL_VRPData structure */
void BEL_InitVRPData(BEL_VRPData *data);

/* Release the memory allocated by a BEL_VRPData structure */
void BEL_FreeVRPData(BEL_VRPData *data);

/* Creates the data to generate edge lengths in the dat structure */
int BEL_VRPGetData(char *datname, int binary_in, int innorm, int *ncount, BEL_VRPData *data,
	int gridsize, int allow_dups, CCrandstate *rstate, int verbose);


/* Solution handling */

/* Initializes a BEL_VRPSolution structure */
void BEL_InitVRPSolution(BEL_VRPSolution *sol);

/* Release the memory allocated by a BEL_VRPData structure */
void BEL_FreeVRPSolution(BEL_VRPSolution *sol);

/* Prints a BEL_VRPSolution to a file */
void BEL_PrintVRPSolution(BEL_VRPSolution *sol, char *optfname, int verbose);

/* Reads a VRP solution from file in standard tourfile format */
int BEL_VRPReadSolution(char *datfile, BEL_VRPSolution *solution, int nodes, int verbose);


/* Problem Solving */

/* Verify wheter this instance of VRP Problem is feasible */
BOOL BEL_VRPProblemIsFeasible(BEL_VRPData *data, int *errorCode, int verbose);

/* Solve an instance of VRP Problem */
int BEL_SolveVRPProblem(BEL_VRPData *data, BEL_VRPSolution *sol);

/* Solves a TSP instance calling Concorde TSP solver */
int *BEL_TSPSolve(int ncount, CCdatagroup *dat, char *probname);

/* Solve an instance of Bin Packing Problem */
int BEL_BPPSolve(int bins, int capacity, int items, int volume[],
	int *min_bins, int verbose);

/* Solve an instance of Capacitated Concentrator Location Problem */
int BEL_CCLPSolve(int items, int cost[items][items], int weight[items], int seeds,
	int seed_cost[items], int capacity, int assignments[items], int verbose);


/* TSPLIB format utilities */

/* Reads a TSPLIB file to a BEL_VRPData structure */
int BEL_VRPReadTSPLIB(char *datfile, BEL_VRPData *data, int verbose);

/* Writes a VRP instance to a file in standard TSPLIB format */
int BEL_VRPWriteTSPLIB(char *datfile, BEL_VRPData *data);


/* Misc utilities */

/* Calculate the number of vehicles needed */
int BEL_VRPVehiclesLB(BEL_VRPData *data);

