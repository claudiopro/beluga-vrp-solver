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
 *  capconloc.c
 *
 *  Capacitated Concentrator Location Problem utilities for Beluga VRP solver
 *
 */

#include "beluga.h"
#include <glpk.h>

/** Bin Packing Problem solver routine
 *
 *  This routine tries to solve a standard Bin Packing Problem instance with our
 *  MIP solver.
 *
 *  @param items Number of items (nodes) to allocate
 *  @param  cost Bidimensional array of cost of allocation for the items
 *  @param weight Array of weights for the nodes
 *  @param seeds  Number of nodes to be chosen as seeds for clusters
 *  @param seed_cost  Array of cost for a node to become seed
 *  @param capacity  Cluster capacity
 *  @param assignments  Array of assignations. Indicates what seed the ith item is assigned to.
 *  @param verbose  Turns on lots of messages
 *  @return 1 on failure, 0 otherwise
 */
int BEL_CCLPSolve(int items, int cost[items][items], int weight[items], int seeds, int seed_cost[items], int capacity, int assignments[items], int verbose)
{
  /**
   * Here we use the GLPK LP solver library.
   * To assign customers to vehicles, we solve a capacitated concentrator
   * location problem.
   * We are interested in building K lists of customers, then the caller will
   * find an optimal TSP tour between them.
   *
   * A CCLP problem has the following structure:
   *
   * <ol><li>Variables:
   *
   *    <code>y<sub>ij</sub>: i-th item is assigned to j-th seed, i=1...N, j=1...N</code><br>
   *    <code>z<sub>j</sub>: j-th item is a seed, j=1...N</code></li>
   *
   * <li>Parameters:
   *
   *    <code>d<sub>ij</sub>: cost of assigning i-th item to j-th seed, i=1...N, j=1...N</code><br>
   *    <code>v<sub>j</sub>: cost of choosing j-th item to be seed, j=1...N</code><br>
   *    <code>a<sub>i</sub>: weight of i-th item, i=1...N</li></code><br>
   *
   * <li>Objective function
   *
   * <ul><li>Minimize the total cost
   *
   *    <code>min Sum<sub>i=1...N,j=1...N</sub>(d<sub>ij</sub>y<sub>ij</sub>) +
	 *		Sum<sub>j=1...N</sub>(v<sub>j</sub>z<sub>j</sub>) (1)</code></li></ul>
   *
   * <li>Constraints
   *
   * <ul><li>We can accept only K seeds:
   *
   *    <code>Sum<sub>j=1...N</sub>(z<sub>j</sub>) = K (2)</code></li>
   *
   * <li>An item must be assigned to one and only one seed:
   *
   *    <code>Sum<sub>j=1...N</sub>(y<sub>ij</sub>) = 1, i=1...N (3)</code></li>
   *
   * <li>Total weight of items associated with j-th seed cannot exceed cluster
   * capacity:
   *
   *    <code>Sum<sub>i=1...M</sub>(a<sub>i</sub>y<sub>ij</sub>) <= V*z<sub>j</sub>, j=1...N (4)</code>
   *
   * we write it as:
	 *
   *    <code>Sum<sub>i=1...M</sub>(a<sub>i</sub>y<sub>ij</sub>) - V*z<sub>j</sub> <= 0, j=1...N (4')</code></li>
   *
   *
   * <li>For performance issues, we also add the following constraints:
   *
   * We can assign i-th item to j-th item only if it is a seed
   *
   *    <code>y<sub>ij</sub> <= z<sub>j</sub>, i=1...N, j=1...N (5)</code>
   *
   * we write it as:
   *
   *    <code>y<sub>ij</sub> - z<sub>j</sub> <= 0, i=1...N, j=1...N (5')</code></li></ul></li></ol>
   *
   */
   
#ifdef DEBUG
	print_matrix(items, items, cost, "cost");
	print_array(items, weight, "weight");
	print_array(items, seed_cost, "seed_cost");
#endif

  LPX *lp;
  int ia[1 + MAX_NONZEROES], ja[1 + MAX_NONZEROES], rows, cols, nonzeroes;
  double ar[1 + MAX_NONZEROES];

  rows = (1)                 // (2)
         + (items)           // (3)
         + (items)           // (4)
         + (items * items);  // (5)
  cols = (items + 1) * items;
  nonzeroes = (1) * (items)
         + (items) * (items + 1)
         + (items) * (items)
         + (items * items) * (2);

	if (verbose)
	{
	  printf("Building Capacitated Concentrator Location Problem instance...\n");

		printf("Items: %d\n", items);
		printf("Seeds: %d\n", seeds);
		printf("Capacity: %d\n", capacity);

	  printf("Rows: %d\n", rows);
  	printf("Cols: %d\n", cols);
   	printf("Nonzeroes: %d\n", nonzeroes);
 	}

  // Create a problem
  lp = lpx_create_prob();

  // Set the problem's name
  lpx_set_prob_name(lp, "capconloc");

  // Set the problem's direction
  lpx_set_obj_dir(lp, LPX_MIN);

  // Add constraints
  lpx_add_rows(lp, rows);
  
  // Initialize rows
  int i, j, offset, row = 1, col = 0;
  char s[255];

  // Constraint (2)

	sprintf(s, "c2[%d]", row);
	lpx_set_row_name(lp, row, s);
	lpx_set_row_bnds(lp, row, LPX_FX, (float)MIN(seeds, items), (float)MIN(seeds, items));

  // Constraint (3)

  for (i = 1; i <= items; i++)
  {
    sprintf(s, "c3[%d]", i);
    lpx_set_row_name(lp, i + row, s);
    lpx_set_row_bnds(lp, i + row, LPX_UP, 0.0, 0.0);
  }
  row += items;
  
  // Constraint (4)
  
  for (i = 1; i <= items; i++)
  {
    sprintf(s, "c4[%d]", i);
    lpx_set_row_name(lp, i + row, s);
    lpx_set_row_bnds(lp, i + row, LPX_FX, 1.0, 1.0);
  }
  row += items;
  
  // Contraint (5)
  
  for (i = 1; i <= items; i++)
  {
    for (j = 1; j <= items; j++)
    {
      sprintf(s, "c5[%d]", i * items + j);
      lpx_set_row_name(lp, j + row, s);
      lpx_set_row_bnds(lp, j + row, LPX_UP, 0.0, 0.0);
    }
    row += items;
  }

  // Initialize cols
  lpx_add_cols(lp, cols);
  for (i = 1, row = 0; i <= items; i++)
  {
    for (j = 1; j <= items; j++)
    {
      sprintf(s, "y[%d][%d]", i, j);
      lpx_set_col_name(lp, j + col, s);
      lpx_set_col_bnds(lp, j + col, LPX_DB, 0.0, 1.0);
      lpx_set_obj_coef(lp, j + col, cost[i - 1][j - 1]);
    }
    col += items;
  }
  for (j = 1; j <= items; j++)
  {
    sprintf(s, "z[%d]", j);
    lpx_set_col_name(lp, j + col, s);
    lpx_set_col_bnds(lp, j + col, LPX_DB, 0.0, 1.0);
    lpx_set_obj_coef(lp, j + col, seed_cost[j - 1]);
  }

  // Initialize matrix
  row = 0;
  col = 0;
  offset = 0;
  for (i = 1; i <= items; i++)
  {
    offset++;
    ia[offset] = 1;
    ja[offset] = i + items * items + row;
    ar[offset] = 1.0;
#ifdef DEBUG
    printf("%d: a[%d][%d] = %f\n", offset, 1, i + items * items + row, 1.0);
#endif
  }
  row++;
  for (i = 1; i <= items; i++)
  {
    for (j = i; j <= i + items * (items - 1); j += items)
    {
      offset++;
      ia[offset] = i + row;
      ja[offset] = j;
      ar[offset] = weight[(j - 1) / items];
#ifdef DEBUG
      printf("%d: a[%d][%d] = weight[%d] = %d\n", offset, i + row, j, (j - 1) / items, weight[(j - 1) / items]);
#endif
    }
    offset++;
    ia[offset] = i + row;
    ja[offset] = i + (items * items);
    ar[offset] = -capacity;
  }
  row += items;

  for (i = 1; i <= items; i++)
  {
    for (j = (i - 1) * items + 1; j <= i * items; j++)
    {
      offset++;
      ia[offset] = i + row;
      ja[offset] = j;
      ar[offset] = 1.0;
#ifdef DEBUG
      printf("%d: a[%d][%d] = %d\n", offset, i + row, j, 1.0);
#endif
    }
  }
  row += items;

  for (i = 1; i <= items * items; i++)
  {
    offset++;
    ia[offset] = i + row;
    ja[offset] = i;
    ar[offset] = 1.0;
#ifdef DEBUG
    printf("%d: a[%d][%d] = %f\n", offset, i + row, i, 1.0);
#endif

    offset++;
    ia[offset] = i + row;
    ja[offset] = items * items + ((i - 1) % items) + 1;
    ar[offset] = -1.0;
#ifdef DEBUG
    printf("%d: a[%d][%d] = %f\n", offset, i + row, items * items + ((i - 1) % items) + 1, -1.0);
#endif
  }
  row += items * items;

  lpx_load_matrix(lp, offset, ia, ja, ar);
  
  // Write to a file
  lpx_write_cpxlp(lp, "capconloc.lp");

  lpx_set_class(lp, LPX_MIP);
  for (i = 1; i <= (items + 1) * items; i++)
  {
    lpx_set_col_kind(lp, i, LPX_IV);
  }
  if (verbose)
  	printf("Integer columns: %d\n", lpx_get_num_int(lp));

  // Launch the MIP solver
  lpx_intopt(lp);

  int mip_status = lpx_mip_status(lp);
  if (verbose)
  	printf("Status: %d\n", mip_status);
  double val;
  switch (mip_status)
  {
    default:
      for (i = 0; i < items; i++)
      {
        // Consider i-th item
        for (j = 0; j < items; j++)
        {
          // If y[ij] = 1 assign item i to seed j
          val = lpx_mip_col_val(lp, i * items + j + 1);
          if (val != 0)
            assignments[i] = j;
#ifdef DEBUG
          printf("%d: y[%d][%d] = %lf\n", i * items + j, i, j, lpx_mip_col_val(lp, i * items + j + 1));
#endif
        }
      }
  }

  // Write problem to a file
  lpx_print_prob(lp, "capconloc.dat");
  // Write output to a file
  lpx_print_sol(lp, "capconloc.sol");
  // Write output to a file
  lpx_print_mip(lp, "capconloc.mipsol");

  lpx_delete_prob(lp);

  return 0;
}
