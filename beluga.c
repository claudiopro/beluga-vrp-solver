/**
 *	Beluga VRP Solver
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
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *	GNU General Public License for more details.
 *
 *	Bug fixes, suggestions and comments should be sent to:
 *	claudio@emeraldion.it
 */
 
/**
 *	BELUGA VRP Solver
 *
 *	A two-phase heuristic VRP solver based on Concorde TSP solver
 *
 *	Copyright 2005-2006 Claudio Procida
 */


#include "beluga.h"

#define CC_JUST_SUBTOUR (1) //!< See Concorde.
#define CC_JUST_BLOSSOM (2) //!< See Concorde.
#define CC_JUST_SUBTOUR_AND_BLOSSOM (3) //!< See Concorde.
#define CC_JUST_FAST_CUTS (4) //!< See Concorde.

/**
 *	Global static variables
 */

static char *optfname			= "tour.opt"; //!< Name of the optimal tour output file
static char *tsplibfname	= "instance.vrp"; //!< Name of the TSPLIB input file
static int silent					= 0; //!< Verbose feedback
static int curr_depot			= 0; //!< The depot we are considering.

static int norm						= CC_EUCLIDEAN; //!< Norm for node distances
static char *datfname			= (char *) NULL;
static char *edgegenfname = (char *) NULL;
static char *problname		= (char *) NULL;
static char *probfname		= (char *) NULL;
static char *edgefname		= (char *) NULL;
static char *fullfname		= (char *) NULL;
static char *tourfname		= (char *) NULL;
static char *poolfname		= (char *) NULL;
static char *restartfname	= (char *) NULL;
static char *xfname				= (char *) NULL;
static char *outfname			= (char *) NULL;
static char *filecutname	= (char *) NULL;
static int seed						= 0;
static int nnodes_want		= 0;
static int binary_in			= 0;
static int tsplib_in			= 1; //!< Input data should be read from a TSPLIB file
static int just_cuts			= 0;
static int dontcutroot		= 0;
static int usetighten			= 0;
static int usedominos			= 0;
static int maxchunksize		= 16;
static int multiple_chunker	= 0;
static int valid_edges		= 0;
static int dfs_branching	= 0;
static int bfs_branching	= 1;
static int simple_branching	= 0;
static int usebranchcliques	= 1;
static int tentative_branch_num = 0;
static int complete_price	= 0;
static int want_rcnearest	= 0;
static int output_tour_as_edges = 0;
static int run_silently		= 1;
static int be_nethost			= 0;
static int unlink_files		= 0;
static double initial_ub	= CCtsp_LP_MAXDOUBLE;
static unsigned short hostport = CCtsp_HOST_PORT;
static int eliminate_edges = -1; //!< Set to 1 to force elim, 0 to not elim
static int eliminate_sparse = 0;   //!<  Set to 1 to elim from full edge list.

static int longedge_branching = 1; //!<  Set to 0 to turn off.
static int save_proof			= 0; //!<  Set to 1 to save the proof.
static int standalone_branch = 0;  //!<  Set to 1 to do a manual branch .

/**
 *  Function prototypes
 */

static int
    handle_just_cuts (CCtsp_lp *lp, int the_cuts, CCrandstate *rstate,
       int silent),
    run_hk (int ncount, CCdatagroup *dat, int *hk_tour),
    build_edges (int *p_ecount, int **p_elist, int **p_elen,
        int ncount, int *ptour, CCdatagroup *dat, char *in_edgefname,
        char *in_edgegenfname, int in_just_cuts, int silent,
        CCrandstate *rstate),
    build_fulledges (int *p_excount, int **p_exlist, int **p_exlen,
        int ncount, int *ptour, char *in_fullfname),
    parseargs (int ac, char **av),
    find_tour (int ncount, CCdatagroup *dat, int *perm, double *ub,
            int trials, int silent, CCrandstate *rstate),
    getedges (CCdatagroup *dat, CCedgegengroup *plan, int ncount, int *ecount,
            int **elist, int **elen, int silent, CCrandstate *rstate),
    dump_rc (CCtsp_lp *lp, int count, char *pname, int usesparse);
static void
    adjust_upbound (double *bound, int ncount, CCdatagroup *dat),
    usage(char *);
    
/** Solves a TSP instance calling Concorde TSP solver.
 *
 *	This function accepts a CCdatagroup as input, the number of nodes in the tour 
 *	and the name to identify the TSP instance. Returns the optimal tour to the caller
 *	as a sequence of node indexes stored in an integer array.
 *
 *  @param ncount Number of nodes in the tour
 *  @param dat  TSP instance data
 *  @param probname A name describing this TSP instance
 *  @return	The optimal tour as a list of nodes
 */

int *BEL_TSPSolve(int ncount, CCdatagroup *dat, char *probname)
{
    int i, rval;
    int ecount = 0;
    int excount = 0;
    int bbcount = 0;
    int *elist = (int *) NULL;
    int *elen = (int *) NULL;
    int *ptour = (int *) NULL;
    int *exlist = (int *) NULL;
    int *exlen = (int *) NULL;
    int *besttour = (int *) NULL;

    int is_infeasible = 0;
    double szeit;
    double upbound = 0.0;
    double branchzeit = 0.0;
    char buf[1024];
    CCtsp_cutselect sel, tentativesel;
    CCrandstate rstate;
    CCtsp_lp *lp = (CCtsp_lp *) NULL;
    CCtsp_lpcuts *pool = (CCtsp_lpcuts *) NULL;
    CCtsp_lpcuts *dominopool = (CCtsp_lpcuts *) NULL;

    szeit = CCutil_zeit ();

    CCutil_printlabel ();
    CCutil_signal_init ();
    CCutil_sprand (seed, &rstate);
    
    if (!be_nethost) {
        hostport = 0;
    }

    CCtsp_init_cutselect (&sel);
    CCtsp_init_tentative_cutselect (&tentativesel);
    CCtsp_cutselect_tighten (&sel, usetighten);
    CCtsp_cutselect_tighten (&tentativesel, usetighten);
    CCtsp_cutselect_chunksize (&sel, maxchunksize);
    CCtsp_cutselect_dominos (&sel, usedominos);
    if (filecutname) CCtsp_cutselect_filecuts (&sel, filecutname);

    if (problname == (char *) NULL) {
        problname = probname;
    }
        /* Handle small instances */

        if (ncount < 3) {
            besttour = CC_SAFE_MALLOC (ncount, int);
            CCcheck_NULL (besttour, "out of memory for besttour");
            ptour = CC_SAFE_MALLOC (ncount, int);
            CCcheck_NULL (ptour, "out of memory for ptour");
            printf("I wish everything was that easy!!\n");
            for (i = 0; i < ncount; i++)
            {
              besttour[i] = i;
              ptour[i] = i;
            }
            goto CLEANUP;
        } else if (ncount < 10) {
            besttour = CC_SAFE_MALLOC (ncount, int);
            CCcheck_NULL (besttour, "out of memory for besttour");
            if (ncount == 3) {
                printf("This one is easy, baby!\n");
                for (i = 0; i < ncount; i++) besttour[i] = i;
            } else {
                rval = run_hk (ncount, dat, besttour);
                CCcheck_rval (rval, "run_hk failed");
            }
            ptour = CC_SAFE_MALLOC (ncount, int);
            CCcheck_NULL (ptour, "out of memory for ptour");
            for (i = 0; i < ncount; i++) ptour[i] = i;
            rval = CCtsp_dumptour (ncount, dat, ptour, probname, besttour,
                                   outfname, output_tour_as_edges, silent);
            CCcheck_rval (rval, "CCtsp_dumptour failed");
            printf ("Total Running Time: %.2f (seconds)\n",
                     CCutil_zeit () - szeit);
            fflush (stdout);
#ifdef DEBUG
    print_array(ncount, besttour, "besttour");
    print_array(ncount, ptour, "ptour");
#endif

            goto CLEANUP;
        }
        /***** Get the permutation tour and permute the data  *****/

        ptour = CC_SAFE_MALLOC (ncount, int);
        CCcheck_NULL (ptour, "out of memory for ptour");

        if (tourfname) {
            rval = CCutil_getcycle (ncount, tourfname, ptour, 0);
            CCcheck_rval (rval, "CCutil_getcycle failed");
        } else {
            double bnd;
            if (just_cuts > 0) {
                rval = find_tour (ncount, dat, ptour, &bnd, -1, silent,
                                  &rstate);
            } else if (initial_ub == CCtsp_LP_MAXDOUBLE) {
                rval = find_tour (ncount, dat, ptour, &bnd, 1, silent,
                                  &rstate);
            } else {
                if (!silent) {
                    printf ("Initial bnd %f - use short LK\n", initial_ub);
                    fflush (stdout);
                }
                rval = find_tour (ncount, dat, ptour, &bnd, 0, silent,
                                 &rstate);
            }
            CCcheck_rval (rval, "find_tour failed");
        }
        rval = CCutil_datagroup_perm (ncount, dat, ptour);
        CCcheck_rval (rval, "CCutil_datagroup_perm failed");

        sprintf (buf, "%s.mas", probname);
        rval = CCutil_putmaster (buf, ncount, dat, ptour);
        CCcheck_rval (rval, "CCutil_putmaster failed");
    adjust_upbound (&initial_ub, ncount, dat);
    if (!probfname && !restartfname) {
        rval = build_edges (&ecount, &elist, &elen, ncount, ptour,
                            dat, edgefname, edgegenfname, just_cuts,
                            silent, &rstate);
        CCcheck_rval (rval, "build_edges failed");
    }

    rval = build_fulledges (&excount, &exlist, &exlen, ncount, ptour,
                            fullfname);
    CCcheck_rval (rval, "build_fulledges failed");
    rval = CCtsp_init_cutpool (&ncount, poolfname, &pool);
    CCcheck_rval (rval, "CCtsp_init_cutpool failed");
#ifdef CCtsp_USE_DOMINO_CUTS
    rval = CCtsp_init_cutpool (&ncount, dominopoolfname, &dominopool);
    CCcheck_rval (rval, "CCtsp_init_cutpool failed for dominos");
#endif

    /***** Initialize besttour to be the permutation tour  ****/

    besttour = CC_SAFE_MALLOC (ncount, int);
#ifdef DEBUG
    print_array(ncount, besttour, "besttour");
    print_array(ncount, ptour, "ptour");
#endif
    CCcheck_NULL (besttour, "out of memory for besttour");
    for (i = 0; i < ncount; i++) {
        besttour[i] = i;
    }
    if (restartfname) {
        upbound  = initial_ub;
        bbcount = 0;

        rval = CCtsp_bfs_restart (problname, restartfname, &sel,
                &tentativesel, &upbound, &bbcount, usebranchcliques, dat,
                ptour, pool, ncount, besttour, hostport, &branchzeit,
                save_proof, tentative_branch_num, longedge_branching,
                (double *) NULL, (int *) NULL, silent, &rstate);
        CCcheck_rval (rval, "CCtsp_bfs_restart failed");
        goto DONE;
    }
    rval = CCtsp_dumptour (ncount, dat, ptour, probname, besttour,
                           (char *) NULL, 0, silent);
    CCcheck_rval (rval, "CCtsp_dumptour failed");
    rval = CCtsp_init_lp (&lp, problname, -1, probfname, ncount, dat,
                    ecount, elist, elen, excount, exlist, exlen, valid_edges,
                    ptour, initial_ub, pool, dominopool, silent, &rstate);
    if (rval == 2) {
        printf ("CCtsp_init_lp reports an infeasible LP\n");
        rval = CCtsp_verify_infeasible_lp (lp, &is_infeasible, silent);
        CCcheck_rval (rval, "CCtsp_verify_infeasible_lp failed");
        if (!is_infeasible) {
            printf ("Couldn't verify infeasible LP\n"); fflush (stdout);
            rval = 1; goto CLEANUP;
        }
        upbound = CCtsp_LP_MAXDOUBLE;
        bbcount = 1;
        goto DONE;
    } else if (rval) {
        fprintf (stderr, "CCtsp_init_lp failed\n"); goto CLEANUP;
    }
    CCutil_start_timer (&lp->stats.total);

    ecount = 0;
    CC_IFFREE (elist, int);
    CC_IFFREE (elen, int);
    excount = 0;
    CC_IFFREE (exlist, int);
    CC_IFFREE (exlen, int);
    if (0 && lp->full_edges_valid) {
        if (CCtsp_inspect_full_edges (lp)) {
            fprintf (stderr, "full edge set does not contain all LP edges\n");
            rval = 1; goto CLEANUP;
        }
    }
    if (standalone_branch) {
        rval = CCtsp_do_interactive_branch (lp, silent, &rstate);
        CCcheck_rval (rval, "CCtsp_do_interactive_branch failed");
        printf ("Total Running Time: %.2f (seconds)\n", CCutil_zeit () - szeit);
        goto CLEANUP;
    }

    if (just_cuts > 0) {
        rval = handle_just_cuts (lp, just_cuts, &rstate, silent);
        CCcheck_rval (rval, "handle_just_cuts failed");
        if (want_rcnearest) {
            rval = dump_rc (lp, want_rcnearest, probname, 0);
            CCcheck_rval (rval, "dump_rc failed");
        }
        if (xfname) {
            rval = CCtsp_dump_x (lp, xfname);
            CCcheck_rval (rval, "CCtsp_dump_x failed");
        }
        goto DONE;
    }

    rval = CCtsp_cutselect_set_tols (&sel, lp, 1, silent);
    CCcheck_rval (rval, "CCtsp_cutselect_set_tols failed");

    if (dontcutroot == 0) {
        if (multiple_chunker) {
            rval = CCtsp_cutting_multiple_loop (lp, &sel, 1, maxchunksize,
                                    1, silent, &rstate);
        } else {
            rval = CCtsp_cutting_loop (lp, &sel, 1, silent, &rstate);
        }
        if (rval == 2) {
            printf ("CCtsp_cutting_loop reports an infeasible LP\n");
            rval = CCtsp_verify_infeasible_lp (lp, &is_infeasible, silent);
            CCcheck_rval (rval, "CCtsp_verify_infeasible_lp failed");
            if (!is_infeasible) {
                printf ("Couldn't verify infeasibile LP\n");
                fflush (stdout);
                rval = 1; goto CLEANUP;
            }
            upbound = CCtsp_LP_MAXDOUBLE;
            bbcount = 1;
            CCutil_stop_timer (&lp->stats.total, 1);
            printf ("Final LP has %d rows, %d columns, %d nonzeros\n",
                    CClp_nrows (lp->lp), CClp_ncols (lp->lp),
                    CClp_nnonzeros (lp->lp));

            goto DONE;
        } else if (rval) {
            fprintf (stderr, "cutting_loop failed\n");
            goto CLEANUP;
        }
    }

        double tourval;
        CCutil_start_timer (&lp->stats.linkern);
        rval = CCtsp_call_x_heuristic (lp, &tourval, besttour, silent, &rstate);
        CCcheck_rval (rval, "CCtsp_call_x_heuristic failed");

        if (!silent) CCutil_stop_timer (&lp->stats.linkern, 1);
        else         CCutil_stop_timer (&lp->stats.linkern, 0);

        if (tourval < lp->upperbound) {
            printf ("New upperbound from x-heuristic: %.2f\n", tourval);
            lp->upperbound = tourval;
            rval = CCtsp_dumptour (ncount, dat, ptour, probname, besttour,
                                   (char *) NULL, 0, silent);
            CCcheck_rval (rval, "CCtsp_dumptour failed");
        }
        printf ("Final lower bound %f, upper bound %f\n", lp->lowerbound,
                                                          lp->upperbound);
        fflush (stdout);

    if (xfname) {
        rval = CCtsp_dump_x (lp, xfname);
        CCcheck_rval (rval, "CCtsp_dump_x failed");
    }
    if (want_rcnearest) {
        rval = dump_rc (lp, want_rcnearest, probname, 0);
        CCcheck_rval (rval, "dump_rc failed");
    }

    if (lp->graph.ncount < 100000 || complete_price) {
        CCbigguy bound;
        CCbigguy bupper;
        rval = CCtsp_exact_price (lp, &bound, complete_price, 0, silent);
        if (rval) {
            fprintf (stderr, "CCtsp_exact_price failed\n");
            goto CLEANUP;
        }
        lp->exact_lowerbound = bound;
        printf ("Exact lower bound: %.6f\n", CCbigguy_bigguytod (bound));
        if (1 || !silent) {
            printf ("DIFF: %f\n", lp->lowerbound - CCbigguy_bigguytod (bound));
            fflush (stdout);
        }

        bupper = CCbigguy_dtobigguy (lp->upperbound);
        CCbigguy_sub (&bupper, CCbigguy_ONE);

        if (CCbigguy_cmp (lp->exact_lowerbound, bupper) > 0) {
            upbound = lp->upperbound;
            bbcount = 1;
            if (!dfs_branching && !bfs_branching) {
                printf ("Optimal Solution: %.2f\n", upbound);
                printf ("Number of bbnodes: %d\n", bbcount);
                fflush (stdout);
            }
            if (!silent) {
                CCutil_stop_timer (&lp->stats.total, 1);
            } else {
                CCutil_stop_timer (&lp->stats.total, 0);
            }
            printf ("Final LP has %d rows, %d columns, %d nonzeros\n",
                    CClp_nrows (lp->lp), CClp_ncols (lp->lp),
                    CClp_nnonzeros (lp->lp));

            if (dat->ndepot > 0) {
                rval = CCtsp_depot_valid (lp, dat->ndepot, (int *) NULL);
                CCcheck_rval (rval, "CCtsp_depot_valid failed");
            }
            goto DONE;
        }

        if (dat->ndepot == 0 && eliminate_edges) {
            rval = CCtsp_eliminate_variables (lp, eliminate_sparse, silent);
            CCcheck_rval (rval, "CCtsp_eliminate_variables failed");
        }
    } else {
        printf ("During testing, do not exact price large problems\n");
        fflush (stdout);
        CCutil_stop_timer (&lp->stats.total, 1);
        printf ("Final LP has %d rows, %d columns, %d nonzeros\n",
                CClp_nrows (lp->lp), CClp_ncols (lp->lp),
                CClp_nnonzeros (lp->lp));

        goto DONE;
    }
#ifdef DEBUG
  	print_array(ncount, besttour, "besttour");
    print_array(ncount, ptour, "ptour");
#endif
    CCutil_stop_timer (&lp->stats.total, 1);
    printf ("Final LP has %d rows, %d columns, %d nonzeros\n",
            CClp_nrows (lp->lp), CClp_ncols (lp->lp),
            CClp_nnonzeros (lp->lp));
    fflush (stdout);

    if (dat->ndepot > 0) {
        rval = CCtsp_depot_valid (lp, dat->ndepot, (int *) NULL);
        CCcheck_rval (rval, "CCtsp_depot_valid failed");
        goto DONE;
    }

    if (dfs_branching) {
        upbound = lp->upperbound;
        bbcount = 0;

        if (simple_branching) CCtsp_init_simple_cutselect (&sel);
        rval = CCtsp_easy_dfs_brancher (lp, &sel, 0, &upbound, &bbcount,
                     usebranchcliques, besttour, longedge_branching,
                     simple_branching, silent, &rstate);
        CCcheck_rval (rval, "CCtsp_easy_dfs_brancher failed");
    } else if (bfs_branching) {
        double lowbound = lp->lowerbound;
        int id          = lp->id;

        upbound  = lp->upperbound;
        bbcount = 0;

        rval = CCtsp_write_probroot_id (problname, lp);
        CCcheck_rval (rval, "CCtsp_write_probroot_id failed");
        CCtsp_free_tsp_lp_struct (&lp);

        rval = CCtsp_bfs_brancher (problname, id, lowbound, &sel,
                &tentativesel, &upbound, &bbcount, usebranchcliques, dat,
                ptour, pool, ncount, besttour, hostport, &branchzeit,
                save_proof, tentative_branch_num, longedge_branching,
                (double *) NULL, (int *) NULL, silent, &rstate);
        CCcheck_rval (rval, "CCtsp_bfs_brancher failed");
    }

DONE:
#ifdef DEBUG
    print_array(ncount, besttour, "besttour");
    print_array(ncount, ptour, "ptour");
#endif
    if (dfs_branching || bfs_branching || restartfname) {
        printf ("Optimal Solution: %.2f\n", upbound);
        printf ("Number of bbnodes: %d\n", bbcount);
        fflush (stdout);
        rval = CCtsp_dumptour (ncount, dat, ptour, probname, besttour,
                               outfname, output_tour_as_edges, silent);
        CCcheck_rval (rval, "CCtsp_dumptour failed");
    } else {
        rval = CCtsp_write_probfile_sav (lp);
        CCcheck_rval (rval, "CCtsp_write_probfile_sav failed");
    }

    printf ("Total Running Time: %.2f (seconds)", CCutil_zeit () - szeit);
    if (branchzeit != 0.0) {
        printf ("  Branching Time: %.2f (seconds)", branchzeit);
    }
    printf ("\n"); fflush (stdout);

    /*  CCtsp_output_statistics (&lp->stats);  */

    if (pool && pool->cutcount) {
        if (!silent) {
            printf ("Final Pool: %d cuts\n", pool->cutcount); fflush (stdout);
        }
        sprintf (buf, "%s.pul", probname);
        rval = CCtsp_write_cutpool (ncount, buf, pool);
        CCcheck_rval (rval, "CCtsp_write_cutpool failed");
    }

#ifdef CCtsp_USE_DOMINO_CUTS
    if (dominopool && dominopool->cutcount) {
        if (1 || !silent) {
            printf ("Final Domino Pool: %d cuts\n", dominopool->cutcount);
            fflush (stdout);
        }
        sprintf (buf, "%s.dominopul", probname);
        rval = CCtsp_write_cutpool (ncount, buf, dominopool);
        CCcheck_rval (rval, "CCtsp_write_cutpool failed");
    }
#endif

    if (sel.remotepool && pool && pool->cutcount > pool->savecount) {
        rval = CCtsp_send_newcuts (ncount, pool, sel.remotehost,
                sel.remoteport);
        if (rval) {
            fprintf (stderr, "CCtsp_send_newcuts failed\n");
            rval = 0;
        }
    }

    rval = 0;

CLEANUP:

    if (unlink_files) {
        if (!run_silently) {
            printf ("Delete the temporary files: pul sav mas\n");
            fflush (stdout);
        }

        sprintf (buf, "%s.pul", probname);
        rval = unlink (buf);
        if (rval && !run_silently) {
            printf ("CCutil_sdelete_file failed for %s\n", buf);
        }

        sprintf (buf, "O%s.pul", probname);
        rval = unlink (buf);
        if (rval && !run_silently) {
            printf ("CCutil_sdelete_file failed for %s\n", buf);
        }

        sprintf (buf, "%s.sav", probname);
        rval = unlink (buf);
        if (rval && !run_silently) {
            printf ("CCutil_sdelete_file failed for %s\n", buf);
        }

        sprintf (buf, "O%s.sav", probname);
        rval = unlink (buf);
        if (rval && !run_silently) {
            printf ("CCutil_sdelete_file failed for %s\n", buf);
        }

        sprintf (buf, "%s.mas", probname);
        rval = unlink (buf);
        if (rval && !run_silently) {
            printf ("CCutil_sdelete_file failed for %s\n", buf);
        }

        sprintf (buf, "O%s.mas", probname);
        rval = unlink (buf);
        if (rval && !run_silently) {
            printf ("CCutil_sdelete_file failed for %s\n", buf);
        }
    }

    if (lp) CCtsp_free_tsp_lp_struct (&lp);
    if (pool) { CCtsp_free_cutpool (&pool); }
    if (dominopool) { CCtsp_free_cutpool (&dominopool); }

	int k, *tour = (int *) NULL;
	tour = (int *)calloc(ncount, sizeof(int));
	
    for (k = 0; k < ncount; k++)
    {
		tour[k] = ptour[besttour[k]];
    }
#ifdef DEBUG
	print_array(ncount, tour, "tour");
#endif

    CC_IFFREE (elist, int);
    CC_IFFREE (elen, int);
    CC_IFFREE (exlist, int);
    CC_IFFREE (exlen, int);
    CC_IFFREE (ptour, int);
    CC_IFFREE (besttour, int);

    return tour;
}

/** Main function
 *
 *  The main function parses the commandline arguments, creates a VRP instance,
 *	either reading it from a TSPLIB file or generating it randomly, solves it and
 *  outputs the solution to an optimal tour file.
 */

int main(int argc, char** argv)
{
	BEL_VRPSolution sol; //!< The solution we are going to find
	BEL_VRPData data; //!< Current VRP instance data
	int rval = 0;
	char *probname = (char *) NULL;
	int i, j, ncount, allow_dups, use_gridsize;
	CCrandstate rstate;
  seed = (int) CCutil_real_zeit();
	CCutil_sprand (seed, &rstate);


	// Parse the command line arguments
	if (parseargs (argc, argv))
	{
		fprintf(stderr, "Error: bad arguments. Aborting.\n");
		exit(1);
	}
	silent = run_silently;

	if (!silent)
  	printf ("Using random seed %d\n", seed); fflush (stdout);

  // Initialize data structures
	BEL_InitVRPData(&data);
	BEL_InitVRPSolution(&sol);
  
	// What data source are we using?
	if (tsplib_in && datfname != (char *) NULL)
	{
		// We are reading data from a TSPLIB file
		rval = BEL_VRPReadTSPLIB(datfname, &data, !silent);
		ncount = data.dimension;
	}
	else
	{
		// Data is being passed in the commandline
		ncount = nnodes_want;
		use_gridsize = nnodes_want;
    	allow_dups = 0;
    	rval = BEL_VRPGetData (datfname, binary_in, norm, &ncount, &data,
                           use_gridsize, allow_dups, &rstate, !silent);

  	}
	if (rval)
	{
		fprintf(stderr, "Error during data acquisition. Aborting.\n");
		exit(1);
	}
	
#ifdef DEBUG
	int adj[ncount][ncount];
  int k, l;
  for (k = 0; k < ncount; k++)
  {
    for (l = 0; l < ncount; l++)
    {
      adj[k][l] = ((&data)->dat->edgelen)(k, l, (&data)->dat);
    }
  }
  print_matrix(ncount, ncount, &adj, "adj");
#endif
	
	/**
	 *	If we have the number of vehicles, we have to verify if the problem is feasible
	 *	If we don't have it, we have to calculate it with a BPP. Either way, we call a
	 *	BPP solver.
	 */

	//Determine if this problem is a feasible VRP instance

	int errCode;
	if (!silent)
		printf("Determining problem feasibility...\n");
	if (!BEL_VRPProblemIsFeasible(&data, &errCode, !silent))
	{
		printf("Error: this is not a feasible instance of VRP (%d). Exiting.\n", errCode);
		exit(1);
	}

	/**
	 *  Attempts to solve the given VRP instance and write the solution to file. If
	 *  we cannot solve it, notify the user and then abort.
	 */
	 
	if (!BEL_SolveVRPProblem(&data, &sol))
	{
    BEL_PrintVRPSolution(&sol, optfname, !silent);
	}
	else
	{
  	fprintf(stderr, "I couldn't solve the current instance of VRP. Aborting.\n");
  	exit(1);
	}

	/**
	 *  If the user has chosen to output the VRP instance to a TSPLIB file, e.g. when
	 *  the instance is randomly generated, write <code>data</code> to <code>tsplibfname</code>.
	 */
	 
	if (tsplibfname != (char *)NULL)
	{
		BEL_VRPWriteTSPLIB(tsplibfname, &data);
	}
	
	// Sayonara
  return 0;
}

/** Verify if the given CVRP instance is feasible
 *
 *  Verify if the given CVRP instance is feasible solving a Bin Packing Problem
 *  instance, considering the vehicles as bins and the demand of customers as
 *  items. If the number of required bins is less or equal than the number of
 *  vehicles the CVRP instance is feasible. If the number of vehicles is not set,
 *  we just set it to the number of required bins and return TRUE.
 *
 *  @param errorCode  An error code denoting the reason of the infeasibility
 *  @param verbose  Be verbose.
 *  @return TRUE if feasible, FALSE otherwise
 *  @see BEL_BPPSolve
 */

BOOL BEL_VRPProblemIsFeasible(BEL_VRPData *data, int *errorCode, int verbose)
{
  /**
   * Let's verify if this instance allows solution.
   * How can we tell? If the number of vehicles is
   * given we solve a Bin Packing Problem and test
   * if we have enough vehicles to serve all customers.
   * If the number of vehicles is not set, it returns TRUE.
   */

  if (data != (BEL_VRPData *) NULL)
  {
    /**
     * Let's formulate an instance of BPP and feed it to our MIP solver
     */

    int items;
    int bins;
    int capacity;
    int min_bins;
    int *volume;
    
    items    = data->ncustomers;
    bins     = BEL_VRPVehiclesLB(data);
    capacity = data->capacity;
    volume   = CC_SAFE_MALLOC(items, int);

		if (verbose)
    {
			printf("Invoking MIP solver on a Bin Packing instance...\n");
    	printf("Bins: %d\n", bins);
    	printf("Capacity: %d\n", capacity);
			printf("Items: %d\n", items);
		}
	int i, j;
	j = 0;
	for (i = 0; i < data->dimension; i++)
	{
		if (!data->isadepot[i])
		{
			volume[j] = data->demand[i];
			j++;
		}
	}
	if (BEL_BPPSolve(bins, capacity, items, volume, &min_bins, verbose))
	{
		if (verbose)
			printf("Feasible :) Number of vehicles needed: %d/%d\n", min_bins, bins);
		data->nvehicles = min_bins;
		return TRUE;
    }
    else
    {
			if (verbose)
      	print("Infeasible :(\n");
      *errorCode = BEL_VRP_NOT_ENOUGH_VEHICLES;
      return FALSE;
    }
  }
  return FALSE;
}

/** Calculate the number of vehicles needed.
 *
 *  Calculate the approximate number of vehicles needed to serve all the customers
 *	of a given CVRP instance using the formula:
 *
 *  <code>Sum<sub>i</sub> a<sub>i</sub>/b</code>
 *
 *  Where <code>a<sub>i</sub></code> is the demand of node <code>i</code> and <code>b</code> is the capacity of the vehicles.
 *
 *  @param data The VRP instance
 *  @return The number of vehicles needed
 */

int BEL_VRPVehiclesLB(BEL_VRPData *data)
{
	int total_demand;
	int capacity;
	int i;
	
	capacity = data->capacity;
	total_demand = 0;
	for (i = 0; i < data->dimension; i++)
		total_demand += data->demand[i];
#ifdef DEBUG
	printf("Total demand: %d, capacity: %d, LB: %d (%f)\n", total_demand, capacity, (int) ceil(total_demand / (float) capacity), total_demand / (float) capacity);
#endif
	return (int) ceil(total_demand / (float) capacity);
}


/**	Solve an instance of VRP Problem
 *
 *  This function solves an instance of VRP splitting the solution in two phases. The first
 *	phase is an assignment of customers to vehicles, solving a Capacitated Concentrator
 *  Location Problem (see Bramel & Simchi-Levi). The second phase is the resolution
 *  of <code>nvehicles</code> instances of TSP to determine the sequence in which
 *  customers should be visited by vehicles.
 *
 *  @param data The problem instance to solve
 *  @param sol  The solution found
 *  @return 1 on failure, 0 otherwise
 */

int BEL_SolveVRPProblem(BEL_VRPData *data, BEL_VRPSolution *sol)
{
 	int dimension = data->dimension;
	int items = data->ncustomers;
	int seeds = data->nvehicles;
	int capacity = data->capacity;
	int depot = data->depots[curr_depot];

	int demand[items];
  int cost[items][items];
  int seed_cost[items];
  int cluster[items];
  int customer2node[items];
  int node2customer[dimension];
	int i, j, k, l;
	k = 0;
	l = 0;
	for (i = 0; i < dimension; i++)
	{
		/**
		 *  Iterate on node index. If the current node is a customer, we add it to the
		 *  list of assignable nodes and memorize the correspondence between absolute
		 *  and relative index in customer2node. Pass to BEL_CCLPSolve a subset of
		 *	nodes and then translate it back to absolute node indexes.
		 */
		 
		node2customer[i] = -1; // Initialized to -1. When equal to -1, node k is a depot
		if (!data->isadepot[i])
		{
			customer2node[k] = i; // Current i-th node is the k-th customer.
			node2customer[i] = k; // Analogously, k-th customer is i-th node. We'll need this later.
			demand[k]= data->demand[i];

        /**
         *  In the Bramel & Simchi-Levi heuristic seed cost is
         *  defined as twice the distance from candidate seed
         *  to the depot
         *
         *  <code>seed_cost<sub>i</sub> = 2d<sub>i0</sub></code>
         */

			seed_cost[k] = 2 * (data->dat->edgelen)(i, depot, data->dat);

			for (j = 0; j < dimension; j++)
			{
        /**
         *  Node cost is defined as the difference between the cost of a path
         *  from the depot to designed seed through current node
         *  and the cost of a straight path from the depot to the
         *  seed
         *
         *  <code>cost<sub>ij</sub> = d<sub>i0</sub> + d<sub>ij</sub> - d<sub>j0</sub></code>
         */

        /**
				 *	As a further development, we could let the user decide what depot
				 *	to choose, in multiple-depot instances of VRP.
				 */

				if (!data->isadepot[j])
				{

					cost[k][l] = (data->dat->edgelen)(i, depot, data->dat) +
								 (data->dat->edgelen)(i, j, data->dat) -
								 (data->dat->edgelen)(depot, j, data->dat);
					l++;
				}
			}
			k++;
			l = 0;
		}
	}
	
	// Call the CCLP solver
  BEL_CCLPSolve(items, cost, demand, seeds, seed_cost, capacity, cluster, !silent);
  
#ifdef DEBUG
	print_array(items, demand, "demand");
	print_array(items, seed_cost, "seed_cost");
	print_matrix(items, items, cost, "cost");
  print_array(items, cluster, "cluster");
  print_array(items, customer2node, "customer2node");
  print_array(dimension, node2customer, "node2customer");
#endif

	/**
	 *	Solve the K instances of TSP using Concorde routines
	 *
	 *	We need a routine that accepts as input a list of nodes and gives as
	 *	output the sequence. May need to build a custom data structure.
	 */

  CCdatagroup routes[data->nvehicles];
  int seed[seeds];
  int total_cost = 0, n = 0;
  for (i = 0; i < items; i++)
  {
		// If cluster[i] == i then node customer2node[i] is a seed
    if (cluster[i] == i)
    {
      seed[n++] = customer2node[i];
    }
  }
#ifdef DEBUG
  print_array(n, seed, "seed");
#endif

  sol->nvehicles = data->nvehicles;
  sol->routelen = (int *)calloc(sol->nvehicles, sizeof(int));
  sol->routes = (int **)calloc(sol->nvehicles, sizeof(int *));

  for (i = 0; i < data->nvehicles; i++)
  {
		// Group customers into clusters
    int current_set[items];
    int n = 1;
    // First element in the cluster is the current depot
    current_set[0] = depot;
    for (j = 0; j < items; j++)
    {
			// If j-th customer is assigned to X-th customer and the i-th seed is X-th customer
      if (cluster[j] == node2customer[seed[i]])
      {
				// Then node customer2node[j] must be included in the set
        current_set[n] = customer2node[j];
        n++;
      }
    }
#ifdef DEBUG
    print_array(n, current_set, "current_set");
#endif
    CCutil_init_datagroup(&(routes[i]));
    CCutil_dat_setnorm (&(routes[i]), data->dat->norm);
    if ((data->dat->norm & CC_NORM_SIZE_BITS) == CC_D2_NORM_SIZE) {
      routes[i].x = CC_SAFE_MALLOC (n, double);
      if (!routes[i].x) {
          CCutil_freedatagroup(&(routes[i]));
          return 1;
      }
      routes[i].y = CC_SAFE_MALLOC (n, double);
      if (!routes[i].y) {
          CCutil_freedatagroup(&(routes[i]));
          return 1;
      }
      for (k = 0; k < n; k++) {
          routes[i].x[k] = data->dat->x[current_set[k]];
          routes[i].y[k] = data->dat->y[current_set[k]];
#ifdef DEBUG
          printf("routes[%d].x[%d] = data->dat->x[current_set[%d]] = %f;\n", i, k, k, data->dat->x[current_set[k]]);
          printf("routes[%d].y[%d] = data->dat->y[current_set[%d]] = %f;\n", i, k, k, data->dat->y[current_set[k]]);
#endif
      }
    }
    else if ((norm & CC_NORM_SIZE_BITS) == CC_D3_NORM_SIZE) {
      routes[i].x = CC_SAFE_MALLOC (n, double);
      if (!routes[i].x) {
          CCutil_freedatagroup(&(routes[i]));
          return 1;
      }
      routes[i].y = CC_SAFE_MALLOC (n, double);
      if (!routes[i].y) {
          CCutil_freedatagroup(&(routes[i]));
          return 1;
      }
      routes[i].z = CC_SAFE_MALLOC (n, double);
      if (!routes[i].z) {
          CCutil_freedatagroup(&(routes[i]));
          return 1;
      }
      for (k = 0; k < n; k++) {
          routes[i].x[k] = data->dat->x[current_set[k]];
          routes[i].y[k] = data->dat->y[current_set[k]];
          routes[i].z[k] = data->dat->z[current_set[k]];
      }
    }
    else {
      fprintf (stderr, "ERROR: Node coordinates with norm %d?\n",
                   data->dat->norm);
      return 1;
    }
    char routename[255];
    sprintf(routename, "%s-route-%d", data->name, i);
    int *tour;
    sol->routelen[i] = n - 1;
    sol->routes[i] = (int *)calloc(n - 1, sizeof(int));

#ifdef DEBUG
    printf("Solving TSP on route %d: ", i);
    for (k = 0; k < n; k++)
    {
      printf("%d ", current_set[k]);
    }
    printf("\n");
#endif

    tour = BEL_TSPSolve(n, &(routes[i]), routename);

#ifdef DEBUG
		print_array(n, tour, "tour");
    printf("Route %d is: ", i);
    for (k = 0; k < n; k++)
    {
      printf("%d ", current_set[tour[k]]);
    }
    printf("%d\n", current_set[tour[0]]);
#endif
    for (k = 0; k < n; k++)
    {
      if (k > 0)
      {
        sol->routes[i][k - 1] = current_set[tour[k]];
        total_cost += (data->dat->edgelen)(current_set[tour[k - 1]], current_set[tour[k]], data->dat);
      }
    }
    total_cost += (data->dat->edgelen)(current_set[tour[n - 1]], current_set[tour[0]], data->dat);

    // Poi magari disegnamo un grafico in SVG! Sì! Sì!
  }
  sol->cost = total_cost;
  
  return 0;
}

/**
 *  See Concorde for details.
 *
 *  @see Concorde source
 */

static int handle_just_cuts (CCtsp_lp *lp, int the_cuts, CCrandstate *rstate,
       int silent)
{
    int rval = 0;
    CCtsp_cutselect sel;

    if (the_cuts == CC_JUST_FAST_CUTS) {
        CCtsp_init_fast_cutselect (&sel);
        rval = CCtsp_cutselect_set_tols (&sel, lp, -1, silent);
        CCcheck_rval (rval, "CCtsp_cutselect_set_tols failed");
        rval = CCtsp_cutting_loop (lp, &sel, 1, silent, rstate);
        CCcheck_rval (rval, "CCtsp_cutting_loop failed");
    } else if (the_cuts == CC_JUST_SUBTOUR) {
        rval = CCtsp_subtour_loop (lp, silent, rstate);
        CCcheck_rval (rval, "CCtsp_subtour_loop failed");
    } else if (just_cuts == CC_JUST_BLOSSOM) {
        rval = CCtsp_blossom_loop (lp, silent, rstate);
        CCcheck_rval (rval, "CCtsp_blossom_loop failed");
    } else if (just_cuts == CC_JUST_SUBTOUR_AND_BLOSSOM) {
        rval = CCtsp_subtour_and_blossom_loop (lp, silent, rstate);
        CCcheck_rval (rval, "CCtsp_subtour_and_blossom_loop failed");
    }

    printf ("Bound: %f\n", lp->lowerbound); fflush (stdout);
    CCutil_stop_timer (&lp->stats.total, 1);
    printf ("Final Root LP has %d rows, %d columns, %d nonzeros\n",
            CClp_nrows (lp->lp), CClp_ncols (lp->lp), CClp_nnonzeros (lp->lp));

CLEANUP:

    return rval;
}

/**
 *  Run Held-Karp TSP solver for small instances. See Concorde for details.
 *
 *  @see Concorde source
 */

static int run_hk (int ncount, CCdatagroup *dat, int *hk_tour)
{
    double hk_val;
    int hk_found, hk_yesno;
    int *hk_tlist = (int *) NULL;
    int rval = 0;

    hk_tlist = CC_SAFE_MALLOC (2*ncount, int);
    CCcheck_NULL (hk_tlist, "out of memory for hk_tlist");

    rval = CCheldkarp_small (ncount, dat, (double *) NULL, &hk_val,
                             &hk_found, 0, hk_tlist, 1000000, 2);
    CCcheck_rval (rval, "CCheldkarp_small failed");
    printf ("Optimal Solution: %.2f\n", hk_val); fflush (stdout);

    rval = CCutil_edge_to_cycle (ncount, hk_tlist, &hk_yesno, hk_tour);
    CCcheck_rval (rval, "CCutil_edge_to_cycle failed");

    if (hk_yesno == 0) {
        fprintf (stderr, "Held-Karp returned list that is not a tour\n");
        rval = 1;  goto CLEANUP;
    }

CLEANUP:

     CC_IFFREE (hk_tlist, int);
     return rval;
}

/**
 *  See Concorde for details.
 *
 *  @see Concorde source
 */

static void adjust_upbound (double *bound, int ncount, CCdatagroup *dat)
{
    double bnd;
    int i;

    bnd = CCutil_dat_edgelen (ncount - 1, 0, dat);
    for (i = 1; i < ncount; i++) {
        bnd += CCutil_dat_edgelen (i-1, i, dat);
    }
    if (bnd < *bound) {
        printf ("Set initial upperbound to %.0f (from tour)\n", bnd);
        fflush (stdout);
        *bound = bnd;
    }
}

/**
 *  See Concorde for details.
 *
 *  @see Concorde source
 */

static int build_edges (int *p_ecount, int **p_elist, int **p_elen,
        int ncount, int *ptour, CCdatagroup *dat, char *in_edgefname,
        char *in_edgegenfname, int in_just_cuts, int silent,
        CCrandstate *rstate)
{
    int rval = 0;
    int *elist = (int *) NULL;
    int ecount;
    int i;

    if (in_edgefname) {
        int *invperm = (int *) NULL;

        printf ("Read initial edge set\n"); fflush (stdout);

        rval = CCutil_getedgelist (ncount, in_edgefname, p_ecount, p_elist,
                                   p_elen, 0);
        CCcheck_rval (rval, "CCutil_getedgelist failed");
        ecount = *p_ecount;
        elist = *p_elist;
        printf ("Initial edgeset: %d edges (%d nodes)\n", ecount, ncount);
        printf ("Rearrange the edges to match the tour order\n");
        fflush (stdout);

        invperm = CC_SAFE_MALLOC (ncount, int);
        CCcheck_NULL (invperm, "out of memory for invperm");
        for (i = 0; i < ncount; i++) invperm[ptour[i]] = i;
        for (i = 0; i < 2*ecount; i++) elist[i] = invperm[elist[i]];
        CC_FREE (invperm, int);
    } else if (dat) {
        CCedgegengroup plan;

        if (in_edgegenfname) {
            rval = CCedgegen_read (in_edgegenfname, &plan);
            CCcheck_rval (rval, "CCedgegen_read failed");
        } else {
            CCedgegen_init_edgegengroup (&plan);
            if (in_just_cuts == CC_JUST_SUBTOUR ||
                in_just_cuts == CC_JUST_BLOSSOM ||
                in_just_cuts == CC_JUST_SUBTOUR_AND_BLOSSOM) {
                plan.tour.greedy = 1;
                plan.f2match_nearest.number = 4;
            } else {
                plan.linkern.count = 10;
                plan.linkern.quadnearest = 2;
                plan.linkern.greedy_start = 0;
                plan.linkern.nkicks = (ncount / 100) + 1;
            }
        }

        rval = getedges (dat, &plan, ncount, p_ecount, p_elist, p_elen,
                         silent, rstate);
        CCcheck_rval (rval, "getedges failed");
    }

CLEANUP:

    return rval;
}

/**
 *  See Concorde for details.
 *
 *  @see Concorde source
 */

static int build_fulledges (int *p_excount, int **p_exlist, int **p_exlen,
        int ncount, int *ptour, char *in_fullfname)
{
    int i;
    int rval = 0;
    int *exlist;
    int excount;

    if (in_fullfname) {
        int *invperm = (int *) NULL;

        rval = CCutil_getedgelist (ncount, in_fullfname, p_excount, p_exlist,
                                   p_exlen, 0);
        CCcheck_rval (rval, "CCutil_getedgelist failed");

        invperm = CC_SAFE_MALLOC (ncount, int);
        CCcheck_NULL (invperm, "out of memory for invperm");
        for (i = 0; i < ncount; i++) invperm[ptour[i]] = i;
        excount = *p_excount;
        exlist = *p_exlist;
        for (i = 0; i < 2*excount; i++) exlist[i] = invperm[exlist[i]];
        CC_FREE (invperm, int);
    } else {
        *p_excount = 0;
    }

CLEANUP:

    return rval;
}

/**
 *  See Concorde for details.
 *
 *  @see Concorde source
 */

static int find_tour (int ncount, CCdatagroup *dat, int *perm, double *ub,
        int trials, int silent, CCrandstate *rstate)
{
    int rval = 0;
    CCedgegengroup plan;
    int ecount;
    int *elist = (int *) NULL;
    int tcount;
    int *tlist = (int *) NULL;
    int *bestcyc = (int *) NULL;
    int *cyc     = (int *) NULL;
    int *tmp;
    double val, bestval, szeit;
    int kicks, i, istour;

    szeit = CCutil_zeit ();
    bestval = CCtsp_LP_MAXDOUBLE;

    if (trials == -1) {
        kicks = (ncount > 400 ? 100 : ncount/4);
    } else {
        kicks = (ncount > 1000 ? 500 : ncount/2);
    }

    if (!silent) {
        printf ("Finding a good tour for compression: %d\n", trials);
        fflush (stdout);
    }

    cyc = CC_SAFE_MALLOC (ncount, int);
    CCcheck_NULL (cyc, "out of memory for cyc");
    bestcyc = CC_SAFE_MALLOC (ncount, int);
    CCcheck_NULL (bestcyc, "out of memory for bestcyc");

    CCedgegen_init_edgegengroup (&plan);
    plan.quadnearest = 2;
    rval = CCedgegen_edges (&plan, ncount, dat, (double *) NULL, &ecount,
                            &elist, silent, rstate);
    CCcheck_rval (rval, "CCedgegen_edges failed");
    plan.quadnearest = 0;

    plan.tour.greedy = 1;
    rval = CCedgegen_edges (&plan, ncount, dat, (double *) NULL, &tcount,
                            &tlist, silent, rstate);
    CCcheck_rval (rval, "CCedgegen_edges failed");

    if (tcount != ncount) {
        fprintf (stderr, "wrong edgeset from CCedgegen_edges\n");
        rval = 1; goto CLEANUP;
    }

    rval = CCutil_edge_to_cycle (ncount, tlist, &istour, cyc);
    CCcheck_rval (rval, "CCutil_edge_to_cycle failed");
    if (istour == 0) {
        fprintf (stderr, "Starting tour has an error\n");
        rval = 1; goto CLEANUP;
    }
    CC_FREE (tlist, int);

    rval = CClinkern_tour (ncount, dat, ecount, elist, ncount, kicks,
                    cyc, bestcyc, &bestval, silent, 0.0, 0.0,
                    (char *) NULL,
                    CC_LK_GEOMETRIC_KICK, rstate);
    CCcheck_rval (rval, "CClinkern_tour failed");

    for (i = 0; i < trials; i++) {
        rval = CClinkern_tour (ncount, dat, ecount, elist, ncount, kicks,
                        (int *) NULL, cyc, &val, silent, 0.0, 0.0,
                        (char *) NULL, CC_LK_GEOMETRIC_KICK, rstate);
        CCcheck_rval (rval, "CClinkern_tour failed");
        if (val < bestval) {
            CC_SWAP (cyc, bestcyc, tmp);
            bestval = val;
        }
    }

    if (trials > 0) {
        rval = CClinkern_tour (ncount, dat, ecount, elist, ncount, 2 * kicks,
                        bestcyc, perm, ub, silent, 0.0, 0.0,
                        (char *) NULL, CC_LK_GEOMETRIC_KICK, rstate);
        CCcheck_rval (rval, "CClinkern_tour failed");
    } else {
        for (i = 0; i < ncount; i++) {
            perm[i] = bestcyc[i];
        }
    }

    if (!silent) {
        printf ("Time to find compression tour: %.2f (seconds)\n",
                CCutil_zeit() - szeit);
        fflush (stdout);
    }

CLEANUP:

    CC_IFFREE (cyc, int);
    CC_IFFREE (bestcyc, int);
    CC_IFFREE (elist, int);
    CC_IFFREE (tlist, int);
    return rval;
}

/**
 *  See Concorde for details.
 *
 *  @see Concorde source
 */

static int getedges (CCdatagroup *dat, CCedgegengroup *plan, int ncount,
        int *ecount, int **elist, int **elen, int silent,
        CCrandstate *rstate)
{
    int i;
    int rval = 0;

    *elist = (int *) NULL;
    *elen = (int *) NULL;

    if (dat == (CCdatagroup *) NULL || plan == (CCedgegengroup *) NULL) {
        fprintf (stderr, "getedges needs CCdatagroup and CCedgegengroup\n");
        rval = 1;  goto CLEANUP;
    }

    rval = CCedgegen_edges (plan, ncount, dat, (double *) NULL, ecount, elist,
                            silent, rstate);
    CCcheck_rval (rval, "CCedgegen_edges failed");

    *elen = CC_SAFE_MALLOC(*ecount, int);
    CCcheck_NULL (*elen, "out of memory for elen");

    for (i = 0; i < *ecount; i++) {
        (*elen)[i] = CCutil_dat_edgelen ((*elist)[2*i], (*elist)[(2*i)+1], dat);
    }

CLEANUP:

    if (rval) {
        CC_IFFREE (*elist, int);
        CC_IFFREE (*elen, int);
    }
    return rval;
}

/**
 *  See Concorde for details.
 *
 *  @see Concorde source
 */

static int dump_rc (CCtsp_lp *lp, int count, char *pname, int usesparse)
{
    int rval = 0;
    char rcnname[1024];

    sprintf (rcnname, "%s.rcn", pname);
    rval = CCtsp_dump_rc_nearest (lp, count, rcnname, usesparse);
    CCcheck_rval (rval, "CCtsp_dump_rc failed");

CLEANUP:

    return rval;
}

/** Parse the commandline arguments.
 *
 *  Parse the commandline arguments and assign relevant values to global variables
 *  that influence the behavior of the program.
 *
 *  @param ac Arguments list length
 *  @param av Arguments list
 *  @return 1 on failure, 0 otherwise
 */

static int parseargs(int ac, char **av)
{
    int c, inorm;
    int boptind = 1;
    char *boptarg = (char *) NULL;
    char *execname;

 	char *tok;
 	tok = strtok(av[0], "/");
 	if (tok == NULL)
 	{
 		execname = av[0];
 	}
 	else
 	{
 		do
 		{
 			execname = tok;
 			tok = strtok(NULL, "/");
 		} while (tok != NULL);
 	}
 	
    /* options that require an argument must be followed by a colon (:) */
    /* Claudio 10/3/2006 */
    while ((c = CCutil_bix_getopt (ac, av, "k:N:o:s:vt:T:D:", &boptind, &boptarg)) != EOF)
        switch (c) {
        case 'k':
            nnodes_want = atoi (boptarg);
            break;
        case 't':
            optfname = boptarg;
            break;
        case 'T':
            tsplibfname = boptarg;
            break;
        case 'o':
            outfname = boptarg;
            break;
        case 's':
            seed = atoi (boptarg);
            break;
        case 'D':
            curr_depot = atoi (boptarg);
            break;
        case 'v':
            run_silently = 0;
            break;
        case 'N':
            inorm = atoi (boptarg);
            switch (inorm) {
      				case 0: norm = CC_MAXNORM; break;
      				case 1: norm = CC_MANNORM; break;
      				case 2: norm = CC_EUCLIDEAN; break;
      				case 3: norm = CC_EUCLIDEAN_3D; break;
      				case 4: norm = CC_USER; break;
      				case 5: norm = CC_ATT; break;
      				case 6: norm = CC_GEOGRAPHIC; break;
      				case 7: norm = CC_MATRIXNORM; break;
      				case 8: norm = CC_DSJRANDNORM; break;
      				case 9: norm = CC_CRYSTAL; break;
      				case 10: norm = CC_SPARSE; break;
      				case 11: norm = CC_RHMAP1; break;
      				case 12: norm = CC_RHMAP2; break;
      				case 13: norm = CC_RHMAP3; break;
      				case 14: norm = CC_RHMAP4; break;
      				case 15: norm = CC_RHMAP5; break;
      				case 16: norm = CC_EUCTOROIDAL; break;
      				case 17: norm = CC_GEOM; break;
      				case 18: norm = CC_EUCLIDEAN_CEIL; break;
      				default:
      					usage (execname);
      				return 1;
            }
            tsplib_in = 0;
            break;
        case CC_BIX_GETOPT_UNKNOWN:
        case '?':
        default:
            usage (execname);
            return 1;
        }
    if (boptind < ac) {
        datfname = av[boptind++];
    }

    if (boptind != ac) {
        usage (execname);
        return 1;
    }

    if (datfname == (char *) NULL && nnodes_want == 0) {
        usage (execname);
        return 1;
    }

    return 0;
}

/** Outputs the usage of this program.
 *
 *	Prints a list of available options and parameters.
 *
 *  @param execname The executable name
 */

static void usage (char *execname)
{
    fprintf (stderr, "Usage: %s [options] dat_file\n", execname);
    fprintf (stderr, "   -k #  number of nodes for random problem\n");
    fprintf (stderr, "   -D #  use custom depot (if more than one)\n");
    fprintf (stderr, "   -t f  output tour file name\n");
    fprintf (stderr, "   -T f  output TSPLIB file name\n");
    fprintf (stderr, "   -o f  output file name (for optimal tour)\n");
    fprintf (stderr, "   -s #  random seed\n");
    fprintf (stderr, "   -v    verbose (turn on lots of messages)\n");
    fprintf (stderr, "   -N #  norm (must specify if dat file is not a TSPLIB file)\n");
    fprintf (stderr, "         0=MAX, 1=L1, 2=L2, 3=3D, 4=USER, 5=ATT, 6=GEO, 7=MATRIX,\n");
    fprintf (stderr, "         8=DSJRAND, 9=CRYSTAL, 10=SPARSE, 11-15=RH-norm 1-5, 16=TOROIDAL\n");
    fprintf (stderr, "         17=GEOM, 18=JOHNSON\n");
}
