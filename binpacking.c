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
 *  binpacking.c
 *
 *  Bin Packing Problem utilities for Beluga VRP solver
 *
 */

#include "beluga.h"
#include <glpk.h>

/** Bin Packing Problem solver routine
 *
 *  This routine tries to solve a standard Bin Packing Problem instance with our
 *  MIP solver.
 *
 *  @param bins Number of available bins
 *  @param capacity Bin capacity (assumed the same for all bins)
 *  @param items Number of items to allocate
 *  @param  volume Array of volumes of the items
 *  @param min_bins Minimum number of bins required
 *  @param verbose  Turns on lots of messages
 *  @return 1 on failure, 0 otherwise
 */

int BEL_BPPSolve(int bins, int capacity, int items, int volume[], int *min_bins, int verbose)
{
	/**
	 * We use here the GLPK LP solver library.
	 * We build a standard MIP problem and try to solve it.
	 * If the problem allows solution, we report it to the caller.
	 *
	 * A BPP problem has the following structure:
	 *
	 * <ul>
	 * <li>Variables:
	 *
	 *		<code>x<sub>ij</sub>: i-th item is assigned to j-th bin, i=1...N, j=1...M</code><br>
	 * 		<code>y<sub>j</sub>: j-th bin is being used, j=1...M</code></li>
	 *
	 * <li>Parameters:
	 *
	 *		<code>v<sub>i</sub>: vulume of i-th item, i=1...N</code></li>
	 *
	 * <li>Objective function
	 * <ul>
	 *	<li>Minimize the number of bins
	 *
	 * 	<code>min Sum<sub>j=1...M</sub>(y<sub>j</sub>) (1)</code></li>
	 *  </ul>
	 *	</li>
	 *
	 * <li>Constraints
	 *  <ul>
	 *  <li>Every item must be assigned to one and only one bin
	 *
	 * 		<code>Sum<sub>j=1...M</sub>(x<sub>ij</sub>) = 1, i=1...N (2)</code></li>
	 *
	 * 	<li>Total vulume of items cannot exceed bin's capacity
	 *
	 * 		<code>Sum<sub>i=1...N</sub>(v<sub>i</sub>*x<sub>ij</sub>) <= V*y<sub>j</sub>, j=1...M (3)</code><br>
	 * 		<code>Sum<sub>i=1...N</sub>(v<sub>i</sub>*x<sub>ij</sub>) - V*y<sub>j</sub> <= 0, j=1...M (3')</code></li>
	 *  </ul>
	 *  </li>
	 *
	 *
	 * <li>For performance issues, we also add the following constraints:
	 *  <ul>
	 * 	<li>We can only assign an item to an actually used bin
	 *
	 * 		<code>x<sub>ij</sub> <= y<sub>j</sub>, i=1...N, j=1...M (4)</code> <br>
	 * 		<code>x<sub>ij</sub> - y<sub>j</sub> <= 0, i=1...N, j=1...M (4')</code></li>
	 *
	 * 	<li>We use a bin only if its predecessor has been used
	 *
	 * 		<code>y<sub>j+1</sub> <= y<sub>j</sub> j=1...M-1 (5)</code><br>
	 * 		<code>y<sub>j+1</sub> - y<sub>j</sub> <= 0 j=1...M-1 (5')</code></li>
	 *  </ul>
	 *  </li>
	 *  </ul>
	 *
	 */

	LPX *lp;
	int ia[1 + MAX_NONZEROES], ja[1 + MAX_NONZEROES], required_bins, rows, cols, nonzeroes;
	double ar[1 + MAX_NONZEROES];

	rows = (items)            // (2)
	       + (bins)           // (3)
	       + (items * bins)   // (4)
	       + (bins - 1);      // (5)
	cols = (items + 1) * bins;

	nonzeroes = (items) * (bins)
	       + (items + 1) * (bins)
	       + (2) * (items * bins)
	       + (2) * (bins - 1);
	if (verbose)
	{
		// Tell what I'm doing here...
	  printf("Building Bin Packing Problem instance...\n");
	  printf("Rows: %d\n", rows);
	  printf("Cols: %d\n", cols);
	  printf("Nonzeroes: %d\n", nonzeroes);
	}
	
	// Create a problem
	lp = lpx_create_prob();

	// Set the problem's name
	lpx_set_prob_name(lp, "binpacking");

	// Set the problem's direction
	lpx_set_obj_dir(lp, LPX_MIN);

	// Add constraints
	lpx_add_rows(lp, rows);
	
	// Initialize rows
	int i, j, offset, row = 0, col = 0;
	char *s;
	for (i = 1; i <= items; i++)
	{
	  sprintf(s, "c2[%d]", i);
	  lpx_set_row_name(lp, i + row, s);
	  lpx_set_row_bnds(lp, i + row, LPX_FX, 1.0, 1.0);
	}
	row += items;
	for (j = 1; j <= bins; j++)
	{
	  sprintf(s, "c3[%d]", j);
	  lpx_set_row_name(lp, j + row, s);
	  lpx_set_row_bnds(lp, j + row, LPX_UP, 0.0, 0.0);
	}
	row += bins;
	for (i = 1; i <= items; i++)
	{
	  for (j = 1; j <= bins; j++)
	  {
	    sprintf(s, "c4[%d]", i * j);
	    lpx_set_row_name(lp, j + row, s);
	    lpx_set_row_bnds(lp, j + row, LPX_UP, 0.0, 0.0);
	  }
	  row += bins;
	}
	for (j = 1; j < bins; j++)
	{
	  sprintf(s, "c5[%d]", j);
	  lpx_set_row_name(lp, j + row, s);
	  lpx_set_row_bnds(lp, j + row, LPX_UP, 0.0, 0.0);
	}

	// Initialize cols
	lpx_add_cols(lp, cols);
	for (i = 1, row = 0; i <= items; i++)
	{
	  for (j = 1; j <= bins; j++)
	  {
	    sprintf(s, "x[%d][%d]", i, j);
	    lpx_set_col_name(lp, j + row, s);
	    lpx_set_col_bnds(lp, j + row, LPX_DB, 0.0, 1.0);
	    lpx_set_obj_coef(lp, j + row, 0.0);
	  }
	  row += bins;
	}
	for (j = 1; j <= bins; j++)
	{
	  sprintf(s, "j[%d]", j);
	  lpx_set_col_name(lp, j + row, s);
	  lpx_set_col_bnds(lp, j + row, LPX_DB, 0.0, 1.0);
	  lpx_set_obj_coef(lp, j + row, 1.0);
	}

	// Initialize matrix
	row = 0;
	col = 0;
	offset = 0;
	for (i = 1; i <= items; i++)
	{
	  for (j = (i - 1) * bins + 1; j <= i * bins; j++)
	  {
	    offset++;
	    ia[offset] = i + row;
	    ja[offset] = j;
	    ar[offset] = 1.0;
#ifdef DEBUG
	    printf("%d: a[%d][%d] = %f\n", offset, i + row, j, 1.0);
#endif
	  }
	}
	row += items;

	for (i = 1; i <= bins; i++)
	{
	  for (j = i; j <= items * bins; j += bins)
	  {
	    offset++;
	    ia[offset] = i + row;
	    ja[offset] = j;
	    ar[offset] = volume[(j - 1) / bins];
#ifdef DEBUG
	    printf("%d: a[%d][%d] = %d\n", offset, i + row, j, volume[(j - 1) / bins]);
#endif
	  }
	  offset++;
	  ia[offset] = i + row;
	  ja[offset] = items * bins + i;
	  ar[offset] = -capacity;
#ifdef DEBUG
	  printf("%d: a[%d][%d] = %d\n", offset, i + row, items * bins + i, -capacity);
#endif
	}
	row += bins;

	for (i = 1; i <= items * bins; i++)
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
	  ja[offset] = items * bins + ((i - 1) % bins) + 1;
	  ar[offset] = -1.0;
#ifdef DEBUG
	  printf("%d: a[%d][%d] = %f\n", offset, i + row, items * bins + ((i - 1) % bins) + 1, -1.0);
#endif
	}
	row += items * bins;

	for (i = 1; i < bins; i++)
	{
	  offset++;
	  ia[offset] = i + row;
	  ja[offset] = items * bins + ((i - 1) % bins) + 1;
	  ar[offset] = -1.0;
#ifdef DEBUG
	  printf("%d: a[%d][%d] = %f\n", offset, i + row, items * bins + ((i - 1) % bins) + 1, -1.0);
#endif

	  offset++;
	  ia[offset] = i + row;
	  ja[offset] = items * bins + ((i - 1) % bins) + 2;
	  ar[offset] = 1.0;
#ifdef DEBUG
	  printf("%d: a[%d][%d] = %f\n", offset, i + row, items * bins + ((i - 1) % bins) + 2, 1.0);
#endif
	}

	lpx_load_matrix(lp, offset, ia, ja, ar);
	
	// Write to a file
	lpx_write_cpxlp(lp, "binpacking.lp");
	
	lpx_set_class(lp, LPX_MIP);
	for (i = 1; i <= (items + 1) * bins; i++)
	{
	  lpx_set_col_kind(lp, i, LPX_IV);
	}
  if (verbose)
  	printf("Integer columns: %d\n", lpx_get_num_int(lp));
  	
  // Launch the MIP solver
  lpx_intopt(lp);

	// Write problem to a file
	lpx_print_prob(lp, "binpacking.dat");
	// Write output to a file
	lpx_print_sol(lp, "binpacking.sol");
	// Write output to a file
	lpx_print_mip(lp, "binpacking.mipsol");

	// Set the minimum number of bins
	*min_bins = lpx_mip_obj_val(lp);

	int mip_status = lpx_mip_status(lp);
	lpx_delete_prob(lp);

	return (mip_status == LPX_I_OPT || mip_status == LPX_I_FEAS);
}
