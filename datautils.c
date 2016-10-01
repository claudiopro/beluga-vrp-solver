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

/**
 *	datautils.c
 *
 *	Data utilities for Beluga VRP Solver
 *
 */

#include "beluga.h"
#include <concorde.h>

#define MATRIX_LOWER_DIAG_ROW  0
#define MATRIX_UPPER_ROW       1
#define MATRIX_UPPER_DIAG_ROW  2
#define MATRIX_FULL_MATRIX     3

void print_matrix(int, int, int **, char *);
void print_array(int, int *, char *);

/** Initializes a BEL_VRPData structure
 *
 *  Call this function to initialize a BEL_VRPData structure before its use.
 *  Initializes and allocates the memory for members.
 *
 *  @param data  The BEL_VRPData structure to be initialized
 */

void BEL_InitVRPData(BEL_VRPData *data)
{
	data->dat = (CCdatagroup *)malloc(sizeof(CCdatagroup));
	if (!data->dat)
	{
		fprintf(stderr, "BEL_InitVRPData: Error during allocation of CCdatagroup. Aborting");
		exit(1);
	}
	data->name = (char *) NULL;
	data->comment = (char *) NULL;

	data->demand = (int *) NULL;
	data->depots = (int *) NULL;
	CCutil_init_datagroup(data->dat);
}

/** Releases memory allocated by a BEL_VRPData struct
 *
 *  Recursively calls <code>free</code> on all pointer members of the
 *  BEL_VRPData structure. Calls Concorde's <code>CCutil_freedatagroup</code> on
 *  <code>CCdatagroup dat</code> member.
 *
 *  @param data The structure to be released.
 */

void BEL_FreeVRPData(BEL_VRPData *data)
{
  CCutil_freedatagroup(data->dat);
  free(data->name);
  free(data->comment);
  free(data->demand);
  free(data->depots);
  free(data);
}

/** Initializes a BEL_VRPSolution structure
 *
 *  Call this function to initialize a BEL_VRPSolution structure before its use.
 *  It initializes and allocates the memory for members.
 *
 *  @param sol  The BEL_VRPSolution structure to be initialized
 */
 
void BEL_InitVRPSolution(BEL_VRPSolution *sol)
{
	sol->routes = (int **) NULL;
	sol->routelen = (int *) NULL;
	sol->cost = 0;
  sol->nvehicles = 0;
}

/** Releases memory allocated by a BEL_VRPSolution struct
 *
 *  Recursively calls <code>free</code> on all pointer members of the
 *  BEL_VRPSolution structure.
 *
 *  @param sol The structure to be released.
 */

void BEL_FreeVRPSolution(BEL_VRPSolution *sol)
{
  free(sol->routes);
  free(sol->routelen);
}

/** Reads a TSPLIB file to a BEL_VRPData structure
 *
 *  This function reads a TSPLIB file and creates a BEL_VRPData structure
 *	representing the given problem instance.
 *  Returns 1 on failure, 0 otherwise.
 *
 *  @param datfile  The name of the TSPLIB file
 *  @param data     The target BEL_VRPData structure to load data into.
 *  @param verbose  Turns on lots of messages.
 *  @return 1 on failure, 0 otherwise.
 */

int BEL_VRPReadTSPLIB(char *datfile, BEL_VRPData *data, int verbose)
{
	FILE *in;
	char buffer[256], key[256], field[256];
	char *p;
	int norm = -1;
	int matrixform = MATRIX_LOWER_DIAG_ROW;
	int ncount = -1;
	int ndepot = 0;
	int i, j;
	
	if ((in = fopen(datfile, "r")) == NULL)
	{
		fprintf(stderr, "Cannot open file %s for reading. Aborting\n", datfile);
		exit(1);
	}
	// Read the file line by line
	while (fgets(buffer, 256, in) != NULL)
	{
		p = buffer;
		while (*p != '\0')
		{
			if (*p == ':')
			{
				*p = ' ';			
			}
			p++;
		}
		p = buffer;
		if (sscanf(p, "%s", key) != EOF)
		{
			p += strlen(key);
			while (*p == ' ')
			{
				p++;
			}
			if (!strcmp(key, "NAME"))
			{
        /**
         *  We cannot directly strcpy p over data->name, for it's NULL.
         *  We have to malloc the room for p and then strcpy it.
         */
        free(data->name);
        data->name = (char *)calloc(strlen(p)-1, sizeof(char));
        strncpy(data->name, p, strlen(p)-1);
			}
			else if (!strcmp(key, "TYPE"))
			{
				if (verbose)
					printf("Problem type: %s", p);
			}
			else if (!strcmp(key, "COMMENT"))
			{
				if (verbose)
					printf("%s", p);
			}
			else if (!strcmp(key, "DIMENSION"))
			{
                if (sscanf (p, "%s", field) == EOF) {
                    fprintf (stderr, "ERROR in DIMENSION line\n");
                    return FALSE;
                }
                ncount = atoi (field);
                data->dimension = ncount;
                if (verbose)
                	printf ("Number of Nodes: %d\n", data->dimension);
			}
			else if (!strcmp(key, "EDGE_WEIGHT_TYPE"))
			{
          if (sscanf (p, "%s", field) == EOF) {
                    fprintf (stderr, "ERROR in EDGE_WEIGHT_TYPE line\n");
                    return FALSE;
                }
                if (!strcmp (field, "EXPLICIT")) {
                    norm = CC_MATRIXNORM;
                    if (verbose)
                    	printf ("Explicit Lengths (CC_MATRIXNORM)\n");
                } else if (!strcmp (field, "EUC_2D")) {
                    norm = CC_EUCLIDEAN;
                    if (verbose)
                    	printf ("Rounded Euclidean Norm (CC_EUCLIDEAN)\n");
                } else if (!strcmp (field, "EUC_3D")) {
                    norm = CC_EUCLIDEAN_3D;
                    if (verbose)
                    	printf ("Rounded Euclidean 3D Norm (CC_EUCLIDEAN_3D)\n");
                } else if (!strcmp (field, "MAX_2D")) {
                    norm = CC_MAXNORM;
                    if (verbose)
                    	printf ("Max Norm (CC_MAXNORM)\n");
                } else if (!strcmp (field, "MAN_2D")) {
                    norm = CC_MANNORM;
                    if (verbose)
                    	printf ("Max Norm (CC_MAXNORM)\n");
                } else if (!strcmp (field, "GEO")) {
                    norm = CC_GEOGRAPHIC;
                    if (verbose)
                    	printf ("Geographical Norm (CC_GEOGRAPHIC)\n");
                } else if (!strcmp (field, "GEOM")) {
                    norm = CC_GEOM;
                    if (verbose)
                    	printf ("Geographical Norm in Meters (CC_GEOM)\n");
                } else if (!strcmp (field, "ATT")) {
                    norm = CC_ATT;
                    if (verbose)
                    	printf ("ATT Norm (CC_ATT)\n");
                } else if (!strcmp (field, "CEIL_2D")) {
                    norm = CC_EUCLIDEAN_CEIL;
                    if (verbose)
                    	printf ("Rounded Up Euclidean Norm (CC_EUCLIDEAN_CEIL)\n");
                } else if (!strcmp (field, "DSJRAND")) {
                    norm = CC_DSJRANDNORM;
                    if (verbose)
                    	printf ("David Johnson Random Norm (CC_DSJRANDNORM)\n");
                } else {
                    fprintf (stderr, "ERROR: Not set up for norm %s\n", field);
                    return FALSE;
                }
                if (CCutil_dat_setnorm (data->dat, norm)) {
                    fprintf (stderr, "ERROR: Couldn't set norm %d\n", norm);
                    return FALSE;
                }
			}
			else if (!strcmp(key, "EDGE_WEIGHT_FORMAT"))
			{
                if (sscanf (p, "%s", field) == EOF) {
                    fprintf (stderr, "ERROR in EDGE_WEIGHT_FORMAT line\n");
                    return FALSE;
                }
                if (!strcmp (field, "LOWER_DIAG_ROW")) {
                    matrixform = MATRIX_LOWER_DIAG_ROW;
                } else if (!strcmp (field, "UPPER_ROW")) {
                    matrixform = MATRIX_UPPER_ROW;
                } else if (!strcmp (field, "UPPER_DIAG_ROW")) {
                    matrixform = MATRIX_UPPER_DIAG_ROW;
                } else if (!strcmp (field, "FULL_MATRIX")) {
                    matrixform = MATRIX_FULL_MATRIX;
                } else if (strcmp (field, "FUNCTION")) {
                    fprintf (stderr, "Cannot handle format: %s\n", field);
                    return FALSE;
                }
			}
			else if (!strcmp(key, "NODE_COORD_SECTION"))
			{
                if (ncount <= 0) {
                    fprintf (stderr, "ERROR: Dimension not specified\n");
                    return FALSE;
                }
                if (data->dat->x != (double *) NULL) {
                    fprintf (stderr, "ERROR: A second NODE_COORD_SECTION?\n");
                    BEL_FreeVRPData(data);
                    return FALSE;
                }
                if ((norm & CC_NORM_SIZE_BITS) == CC_D2_NORM_SIZE) {
                    data->dat->x = CC_SAFE_MALLOC (ncount, double);
                    if (!data->dat->x) {
                        BEL_FreeVRPData(data);
                        return FALSE;
                    }
                    data->dat->y = CC_SAFE_MALLOC (ncount, double);
                    if (!data->dat->y) {
                        BEL_FreeVRPData(data);
                        return FALSE;
                    }
                    for (i = 0; i < ncount; i++) {
                        fscanf (in, "%*d %lf %lf", &(data->dat->x[i]), &(data->dat->y[i]));
                    }
                } else if ((norm & CC_NORM_SIZE_BITS) == CC_D3_NORM_SIZE) {
                    data->dat->x = CC_SAFE_MALLOC (ncount, double);
                    if (!data->dat->x) {
                        BEL_FreeVRPData(data);
                        return FALSE;
                    }
                    data->dat->y = CC_SAFE_MALLOC (ncount, double);
                    if (!data->dat->y) {
                        BEL_FreeVRPData(data);
                        return FALSE;
                    }
                    data->dat->z = CC_SAFE_MALLOC (ncount, double);
                    if (!data->dat->z) {
                        BEL_FreeVRPData(data);
                        return FALSE;
                    }
                    for (i = 0; i < ncount; i++) {
                        fscanf (in, "%*d %lf %lf %lf",
                               &(data->dat->x[i]), &(data->dat->y[i]), &(data->dat->z[i]));
                    }
                } else {
                    fprintf (stderr, "ERROR: Node coordinates with norm %d?\n",
                                 norm);
                    return FALSE;
                }
			}
			else if (!strcmp(key, "EDGE_WEIGHT_SECTION"))
			{
	                if (ncount <= 0) {
	                    fprintf (stderr, "ERROR: Dimension not specified\n");
                     return FALSE;
	                }
	                if (data->dat->adj != (int **) NULL) {
	                    fprintf (stderr, "ERROR: A second NODE_COORD_SECTION?\n");
	                    CCutil_freedatagroup (data->dat);
                     return FALSE;
	                }
	                if ((norm & CC_NORM_SIZE_BITS) == CC_MATRIX_NORM_SIZE) {
	                    data->dat->adj = CC_SAFE_MALLOC (ncount, int *);
	                    data->dat->adjspace = CC_SAFE_MALLOC (ncount*(ncount+1)/2,
	                                                    int);
	                    if (data->dat->adj == (int **) NULL ||
	                        data->dat->adjspace == (int *) NULL) {
	                        CCutil_freedatagroup (data->dat);
                         return FALSE;
	                    }
	                    for (i = 0, j = 0; i < ncount; i++) {
	                        data->dat->adj[i] = data->dat->adjspace + j;
	                        j += (i+1);
	                    }
	                    if (matrixform == MATRIX_LOWER_DIAG_ROW) {
	                        for (i = 0; i < ncount; i++) {
	                            for (j = 0; j <= i; j++)
	                                fscanf (in, "%d", &(data->dat->adj[i][j]));
	                        }
	                    } else if (matrixform == MATRIX_UPPER_ROW ||
	                               matrixform == MATRIX_UPPER_DIAG_ROW ||
	                               matrixform == MATRIX_FULL_MATRIX) {
	                        int **tempadj = (int **) NULL;
	                        int *tempadjspace = (int *) NULL;
	                        tempadj = CC_SAFE_MALLOC (ncount, int *);
	                        tempadjspace = CC_SAFE_MALLOC (ncount * ncount,
	                                                       int);
	                        if (tempadj == (int **) NULL ||
	                            tempadjspace == (int *) NULL) {
	                            CC_IFFREE (tempadj, int *);
	                            CC_IFFREE (tempadjspace, int);
	                            CCutil_freedatagroup (data->dat);
                             return FALSE;
	                        }
	                        for (i = 0; i < ncount; i++) {
	                            tempadj[i] = tempadjspace + i * ncount;
	                            if (matrixform == MATRIX_UPPER_ROW) {
	                                tempadj[i][i] = 0;
	                                for (j = i + 1; j < ncount; j++)
	                                    fscanf (in, "%d", &(tempadj[i][j]));
	                            } else if (matrixform == MATRIX_UPPER_DIAG_ROW) {
	                                for (j = i; j < ncount; j++)
	                                    fscanf (in, "%d", &(tempadj[i][j]));
	                            } else {
	                                for (j = 0; j < ncount; j++)
	                                    fscanf (in, "%d", &(tempadj[i][j]));
	                            }
	                        }
	                        for (i = 0; i < ncount; i++) {
	                            for (j = 0; j <= i; j++)
	                                data->dat->adj[i][j] = tempadj[j][i];
	                        }
	                        CC_FREE (tempadjspace, int);
	                        CC_FREE (tempadj, int *);
	                    }
	                } else {
	                    fprintf (stderr, "ERROR: Matrix with norm %d?\n",
	                             norm);
                     return FALSE;
	                }
			}
			else if (!strcmp(key, "FIXED_EDGES_SECTION"))
			{
                fprintf (stderr, "ERROR: Not set up for fixed edges\n");
                return FALSE;
			}
			else if (!strcmp(key, "CAPACITY"))
			{
                if (sscanf (p, "%s", field) == EOF) {
                    fprintf (stderr, "ERROR in DIMENSION line\n");
                    return FALSE;
                }
                data->capacity = atoi (field);
                if (verbose)
                	printf ("Vehicle capacity: %d\n", data->capacity);
			}
			else if (!strcmp(key, "DEMAND_SECTION"))
			{
				int demand;
                if (ncount <= 0) {
                    fprintf (stderr, "ERROR: Dimension not specified\n");
                    return FALSE;
                }
                if (data->demand != (int *) NULL) {
                    fprintf (stderr, "ERROR: A second DEMAND_SECTION?\n");
                    BEL_FreeVRPData(data);
                    return FALSE;
                }
                data->demand = CC_SAFE_MALLOC (ncount, int);
                if (!data->demand) {
                    BEL_FreeVRPData(data);
                    return FALSE;
                }
                for (i = 0; i < ncount; i++) {
                    fscanf (in, "%d %d", &j, &demand);
                    if (i != j - 1)
                    {
                      fprintf (stderr, "ERROR: Malformed DEMAND_SECTION. Found %d, expecting %d.\n", j - 1, i);
                      BEL_FreeVRPData(data);
                      return FALSE;
                    }
                    if (demand == 0)
                    {
                      // This entry is a depot
                      ndepot++;
                    }
                    data->demand[i] = demand;
                }
                data->ncustomers = data->dimension - ndepot;
                data->ndepots = ndepot;
			}
			else if (!strcmp(key, "DEPOT_SECTION"))
			{
             	int dep;
                if (ncount <= 0) {
                    fprintf (stderr, "ERROR: Dimension not specified\n");
                    return FALSE;
                }
                if (!data->demand) {
                    fprintf (stderr, "ERROR: Missing DEMAND_SECTION?\n");
                    BEL_FreeVRPData(data);
                    return FALSE;
                }
                if (data->depots != (int *) NULL) {
                    fprintf (stderr, "ERROR: A second DEPOT_SECTION?\n");
                    BEL_FreeVRPData(data);
                    return FALSE;
                }
                data->isadepot = CC_SAFE_MALLOC (ncount, int);
                if (!data->isadepot) {
                    BEL_FreeVRPData(data);
                    return FALSE;
                }
								data->depots = CC_SAFE_MALLOC (ndepot, int);
                if (!data->depots) {
                    BEL_FreeVRPData(data);
                    return FALSE;
                }
                int k = 0;
                do {
                    fscanf (in, "%d", &dep);
                    if (dep != -1)
                    {
											// Assign 1 to identify this node as a depot
											data->isadepot[dep - 1] = 1;
											
											// Put this node in the depots list
											data->depots[k++] = dep - 1;
                    }
                } while (dep != -1);
			}
		}
	}
	fclose(in);

#ifdef DEBUG
		for (i = 0; i < ncount; i++)
		{
			printf("Coordinates: %.4lf, %.4lf, Demand: %d, Depot: %d\n", data->dat->x[i], data->dat->y[i], data->demand[i], data->isadepot[i]);
		}
#endif

	return 0;
}

/** Writes a VRP instance to a file in standard TSPLIB format.
 *
 *  Gets a problem instance as a BEL_VRPData structure and writes
 *  to a file in standard TSPLIB format. At the moment, only instances
 *	with norm EUC_2D are supported.
 *
 *  @param datfile  The name of the output TSPLIB file.
 *  @param data The BEL_VRPData structure that holds the instance to be written.
 *  @return 1 on failure, 0 otherwise.
 */

int BEL_VRPWriteTSPLIB(char *datfile, BEL_VRPData *data)
{
	/**
	 *  TSPLIB file format\n
	 *  ------------------\n
	 *	NAME : <instance name>\n
	 *	COMMENT : <comments here>\n
	 *	TYPE : CVRP\n
	 *	DIMENSION : <dimension>\n
	 *	EDGE_WEIGHT_TYPE : <norm>\n
	 *	CAPACITY : <capacity>\n
	 *	NODE_COORD_SECTION\n
	 *	 1 X1 Y1\n
	 *	 ...\n
	 *	 N XN YN\n
	 *	DEMAND_SECTION\n
	 *	1 0\n
	 *	2 D2\n
	 *	...\n
	 *	N DN\n
	 *	DEPOT_SECTION\n
	 *	 1\n
	 *	 -1\n
	 *	EOF\n
	 */
	 
	FILE *out;
	int i;
  if (!(out = fopen(datfile, "w")))
  {
    printf("Error. Can't open file for writing. Aborting.\n");
    exit(1);
  }

	// Preamble
  fprintf(out, "NAME : %s\n", data->name);
  fprintf(out, "COMMENT : %s\n", data->comment);
  fprintf(out, "TYPE : CVRP\n");
  fprintf(out, "DIMENSION : %d\n", data->dimension);
  fprintf(out, "EDGE_WEIGHT_TYPE : EUC_2D\n"); // Questo va dentro uno switch...case
  fprintf(out, "CAPACITY : %d\n", data->capacity);

	// Node coordinates section
  fprintf(out, "NODE_COORD_SECTION\n");
	for (i = 0; i < data->dimension; i++)
	{
    fprintf(out, "%2d %3d %3d\n", i + 1, (int)data->dat->x[i], (int)data->dat->y[i]);
	}
	// Demand section
  fprintf(out, "DEMAND_SECTION\n");
	for (i = 0; i < data->dimension; i++)
	{
    fprintf(out, "%2d %3d\n", i + 1, data->demand[i]);
	}
	// Depot section
  fprintf(out, "DEPOT_SECTION\n");
	for (i = 0; i < data->dimension; i++)
	{
		if (data->isadepot[i])
    	fprintf(out, "%2d\n", i + 1);
	}
	fprintf(out, "-1\n");
	fprintf(out, "EOF");

  fflush(out);
  fclose(out);
}

/** Reads a VRP solution from file in standard tourfile format.
 *
 *  Reads VRP solution from file and returns a BEL_VRPSolution structure
 *  that holds the vehicle routes and the cost of the given solution.
 *
 *  @param datfile  The name of the input tourfile.
 *  @param solution The BEL_VRPSolution structure to be filled with tour data.
 *  @param nodes  The number of nodes in this VRP instance.
 *  @param verbose  Turns on lots of messages.
 *  @return 1 on failure, 0 otherwise.
 */

int BEL_VRPReadSolution(char *datfile, BEL_VRPSolution *solution, int nodes, int verbose)
{
	FILE *in;
	char buffer[256], key[256], field[256];
	char *p;
	int nvehicles = 0;
	int cost = 0;
  int routelist[nodes][nodes];

	if ((in = fopen(datfile, "r")) == NULL)
	{
		fprintf(stderr, "Cannot open file %s for reading. Aborting\n", datfile);
		exit(1);
	}
	// Read the file line by line
	while (fgets(buffer, 256, in) != NULL)
	{
		p = buffer;
		while (*p != '\0')
		{
			if (*p == ':')
			{
				*p = ' ';
			}
			p++;
		}
		p = buffer;
		if (sscanf(p, "%s", key) != EOF)
		{
			p += strlen(key);
			while (*p == ' ')
			{
				p++;
			}
			if (!strcmp(key, "cost"))
			{
                if (sscanf (p, "%s", field) == EOF) {
                    fprintf (stderr, "ERROR in COST line\n");
                    return 1;
                }
                cost = atoi (field);
                solution->cost = cost;
                if (verbose)
                  printf ("Cost: %d\n", solution->cost);
			}
			else if (!strcmp(key, "Route"))
			{
        int i = 1;
        if (sscanf (p, "#%s", field) != EOF)
        {
          fflush(stdout);
          int r = atoi(field);
          if (r != nvehicles + 1)
          {
            fprintf (stderr, "ERROR in Route line\n");
            return 1;
          }
          p += strlen(field) + 1;
			    while (*p == ' ')
			    {
				    p++;
			    }
        }
        while(sscanf (p, "%s", field) != EOF)
        {
          routelist[nvehicles][i++] = atoi(field);
#ifdef DEBUG
          printf("routelist[%d][%d] = %d\n", nvehicles, i - 1, atoi(field));
#endif
          p += strlen(field);
			    while (*p == ' ')
			    {
				    p++;
			    }
        }
        routelist[nvehicles][0] = i - 1; // Hack to know how long it is
        nvehicles++;
			}
		}
	}
	fclose(in);
	int i, j, routelen;
	solution->nvehicles = nvehicles;
	solution->routes = (int **)calloc(nvehicles, sizeof(int *));
	solution->routelen = (int *)calloc(nvehicles, sizeof(int *));
	for (i = 0; i < nvehicles; i++)
	{
    routelen = routelist[i][0];
    solution->routelen[i] = routelen;
		solution->routes[i] = (int *)calloc(routelen, sizeof(int));
		for (j = 0; j < routelen; j++)
		{
      solution->routes[i][j] = routelist[i][j + 1];
    }
	}
  return 0;
}

/** Flushes a matrix of integers to standard output for debug.
 *
 *  A commodity function to debug integer matrices used throughout the program.
 *
 *  @param rows  Matrix rows.
 *  @param cols Matrix cols.
 *  @param matrix The actual matrix data.
 *  @param name The name of the matrix.
 */

 void print_matrix(int rows, int cols, int **matrix, char *name)
{
	int i, j;
	int **mat;
	
	mat = matrix;
	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < cols; j++)
		{
			print("%s[%d][%d] = %-4d", name, i, j, mat[cols * i + j]);
		}
		print("\n");
	}
}

/** Flushes an array of integers to standard output for debug.
 *
 *  A commodity function to debug integer arrays used throughout the program.
 *
 *  @param rows  Array rows.
 *  @param array The actual array data.
 *  @param name The name of the array.
 */
 
 void print_array(int rows, int *array, char *name)
{
	int i;
	printf("Dumping %s...\n", name);
	for (i = 0; i < rows; i++)
	{
		print("%s[%d] = %-4d", name, i, array[i]);
	}
}
