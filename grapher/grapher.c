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

 *  Created: 03/09/06 16.06.16 by claudio
 *  $Id: C\040Console.c,v 1.1.2.1 2003/08/13 00:38:46 neum Exp $
 */

#include "grapher.h"

char *datfname = (char *) NULL; //!< Input TSPLIB data file name
char *tourfname = (char *) NULL; //!< Input tour file name
char *bgimage = (char *) NULL; //!< Background image file name
char *outfname = "out.svg"; //!< Output SVG file name
char *execname; //!< Executable name

int verbose = 0; //!< Be verbose
int labelize = 0; //!< Display node labels
int dontscale = 0; //!< Do not scale graph
int bgimgwidth = WIDTH; //!< Background image width
int bgimgheight = HEIGHT; //!< Background image height
int depot = 0; //!< Depot in use
int usedots = 0; //!< Use dots to represent nodes instead of bullets

/** Displays usage info on the program.
 *
 *  Displays a list of available options and arguments and returns.
 *
 *  @param programName  The executable name
 */

void Usage(char *programName)
{
  /* Modify here to add your usage message when the program is
   * called without arguments */
  fprintf(stderr, "\n");
  fprintf(stderr, "%s [-v] [-l] -d datfile -t tourfile [-o outputfile]\n", programName);
  fprintf(stderr, "%s -s -b myimage.png -w 450 -h 350 -d datfile -t tourfile\n", programName);
  fprintf(stderr, "%s -?|-help\n", programName);
  fprintf(stderr, "\n");
  fprintf(stderr, "Usage:\n");
  fprintf(stderr, "-v              Be verbose\n");
  fprintf(stderr, "-D depot        Force depot to be the depot-th node\n");
  fprintf(stderr, "-p              Use dots to represent nodes instead of bullets\n");
  fprintf(stderr, "-l              Display labels with node number and demand\n");
  fprintf(stderr, "-d datfile      Reads problem data from datfile (required)\n");
  fprintf(stderr, "-t tourfile     Reads tour data from tourfile (required)\n");
  fprintf(stderr, "-o outputfile   Writes SVG data to outputfile (default out.svg)\n");
  fprintf(stderr, "-s              Do not scale graph to fit. Recommended if you want\n");
  fprintf(stderr, "                the graph to overlay on the background image.\n");
  fprintf(stderr, "-b bgimage      Uses bgimage as background image for the graph. Image\n");
  fprintf(stderr, "                must be in the same folder of the output file. Allowed\n");
	fprintf(stderr, "                types: PNG, JPEG or SVG images\n");
  fprintf(stderr, "-w              Background image width (Default %d)\n", WIDTH);
  fprintf(stderr, "-h              Background image height (Default %d)\n", HEIGHT);
  fprintf(stderr, "-?, -help       Prints this help message\n");
  fprintf(stderr, "\n");
  fflush(stderr);
}

/** Handles the commandline options.
 *
 *  Parses the commandline options and assigns values to relevant global variables,
 *  influencing the behavior of the program.
 *
 *  @param argc The lenght of commandline arguments list
 *  @param argv The list of commandline arguments
 */
 
void HandleOptions(int argc, char *argv[])
{
  int i = 0;
  for (i = 1; i < argc; i++) {
    if (argv[i][0] == '/' || argv[i][0] == '-') {
      switch (argv[i][1]) {
        // Display usage
        case '?':
          Usage(execname);
          break;
        case 'h':
        case 'H':
          if (!stricmp(argv[i] + 1, "help")) {
            Usage(execname);
            break;
          }
    // Background image height
          bgimgheight = atoi(argv[i + 1]);
          if (verbose)
            printf("Background image height: %d\n", bgimgheight);
          break;
    // Be verbose
        case 'v':
          verbose = 1;
          printf("Verbose mode\n");
          break;
    // Do not use bullets
        case 'p':
          usedots = 1;
          printf("Using dots for nodes\n");
          break;
    // Don't scale
        case 's':
          dontscale = 1;
          if (verbose)
            printf("No scaling\n");
          break;
    // Background image width
        case 'w':
          bgimgwidth = atoi(argv[i + 1]);
          if (verbose)
            printf("Background image width: %d\n", bgimgwidth);
          break;
    // Use custom depot
        case 'D':
          depot = atoi(argv[i + 1]);
          if (verbose)
            printf("Depot is node #%d\n", depot);
          break;
    // Display labels
        case 'l':
          labelize = 1;
          if (verbose)
            printf("Will display labels\n");
          break;
        // TSPLIB input data file
        case 'd':
          datfname = (char *)calloc(strlen(argv[i + 1]) + 1, sizeof(char));
          strncpy(datfname, argv[i + 1], strlen(argv[i + 1]));
          if (verbose)
            printf("Data file is %s\n", datfname);
          break;
    // Tour input data file
        case 't':
          tourfname = (char *)calloc(strlen(argv[i + 1]) + 1, sizeof(char));
          strncpy(tourfname, argv[i + 1], strlen(argv[i + 1]));
          if (verbose)
            printf("Tour file is %s\n", tourfname);
          break;
    // Background image
        case 'b':
          bgimage = (char *)calloc(strlen(argv[i + 1]) + 1, sizeof(char));
          strncpy(bgimage, argv[i + 1], strlen(argv[i + 1]));
          if (verbose)
            printf("Background image is %s\n", bgimage);
          break;
    // SVG output file
        case 'o':
          outfname = (char *)calloc(strlen(argv[i + 1]) + 1, sizeof(char));
          strncpy(outfname, argv[i + 1], strlen(argv[i + 1]));
          if (verbose)
            printf("Output file is %s\n", outfname);
          break;
        default:
          fprintf(stderr, "unknown option %s\n", argv[i]);
      }
    }
  }
}

/** Main function.
 *
 *  The main function tries to load data from the given input TSPLIB file and tour file,
 *  combines them and plots a SVG output file representing the nodes and the vehicle routes.
 *
 *  @param argc The lenght of commandline arguments list
 *  @param argv The list of commandline arguments
 *  @return 1 on failure, 0 otherwise
 */

int main(int argc, char *argv[])
{
  BEL_VRPData data;
  BEL_VRPSolution sol;
  char *tok;
  tok = strtok(argv[0], "/");
  if (tok == NULL)
  {
    execname = argv[0];
  }
  else
  {
		do
		{
			execname = tok;
     	tok = strtok(NULL, "/");
		} while (tok != NULL);
  }

  if (argc == 1) {
    /* If no arguments we call the Usage routine and exit */
    Usage(execname);
    return 1;
  }
  /* handle the program options */
  HandleOptions(argc, argv);

  /* Init data */
  BEL_InitVRPData(&data);
  BEL_InitVRPSolution(&sol);
  
  /* Open the datfile for reading */
  if (BEL_VRPReadTSPLIB(datfname, &data, verbose))
  {
    printf("Error. Can't read data from %s. Aborting.\n", datfname);
    exit(1);
  }
  /* Open the tourfile for reading */
  if (BEL_VRPReadSolution(tourfname, &sol, data.dimension, verbose))
  {
    printf("Error. Can't read tour from %s. Aborting.\n", tourfname);
    exit(1);
  }
  /* Write the SVG */
  BEL_PrintSVG(&data, &sol);
  return 0;
}

/** Represent the VRP instance solution as a SVG file.
 *
 *  Plot the nodes and the vehicle routes of the given VRP instance to a SVG file.
 *
 *  SVG (Scalable Vector Graphics) is a flexible XML format to represent graphical objects.
 *
 *  See <a href="http://www.w3.org/Graphics/SVG/">http://www.w3.org/Graphics/SVG/</a> for details.
 *
 *  @param data VRP instance data
 *  @param solution VRP instance solution
 */

void BEL_PrintSVG(BEL_VRPData *data, BEL_VRPSolution *solution)
{
  FILE *svgfile;
  srand(utime());
  
  /* Setup */
  
  int min_x = 0xffff,
      min_y = 0xffff,
      max_x = 0,
      max_y = 0,
      v_x = 0,
      v_y = 0,
			pad = 0;
  float stroke_width = (float) STROKE_WIDTH,
      	node_radius = (float) NODE_RADIUS,
      	scale = 1;
  int i, j, routes, dimension, routelen;

  dimension = data->dimension;
  for (i = 0; i < dimension; i++)
  {
    min_x = (data->dat->x[i] < min_x) ? (int)data->dat->x[i] : min_x;
    min_y = (data->dat->y[i] < min_y) ? (int)data->dat->y[i] : min_y;
    max_x = (data->dat->x[i] > max_x) ? (int)data->dat->x[i] : max_x;
    max_y = (data->dat->y[i] > max_y) ? (int)data->dat->y[i] : max_y;
  }

  if (!dontscale)
  {
	  scale = MIN((WIDTH - 2 * PAD) / (max_x - min_x),
							(HEIGHT - 2 * PAD) / (max_y - min_y));
	  node_radius /= scale;
  	stroke_width /= scale;
  	v_x = min_x;
  	v_y = min_y;
  	pad = PAD;
	}
  if (verbose)
  {
    printf("X coordinates: min %d, max %d\n", min_x, max_x);
    printf("Y coordinates: min %d, max %d\n", min_y, max_y);
    printf("Translation: (%d, %d)\n", - v_x, - v_y);
    printf("Scale: %f\n", scale);
		printf("Node radius: %f\n", node_radius);
		printf("Stroke width: %f\n", stroke_width);
		printf("Padding: %d\n", pad);
  }
  
  /* Draw */
  
  if (verbose)
    printf("Writing output to %s\n", outfname);
  if (!(svgfile = fopen(outfname, "w")))
  {
    printf("Error. Can't open file for writing. Aborting.\n");
    exit(1);
  }
  fprintf(svgfile, "<?xml version=\"1.0\" standalone=\"no\"?>\n");
  fprintf(svgfile, "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n");
  fprintf(svgfile, "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n");
  fprintf(svgfile, "\n");
  fprintf(svgfile, "<svg width=\"100%%\" height=\"100%%\" version=\"1.1\"\n");
  fprintf(svgfile, "xmlns=\"http://www.w3.org/2000/svg\"\n");
  fprintf(svgfile, "xmlns:xlink=\"http://www.w3.org/1999/xlink\">\n");
  fprintf(svgfile, "\n");
  fprintf(svgfile, "\t<g transform=\"translate(%d,%d)\">\n",
          pad,
          pad);
  fprintf(svgfile, "\t\t<g transform=\"scale(%f)\">\n",
          scale);
  fprintf(svgfile, "\t\t\t<g transform=\"translate(%d,%d)\">\n",
          - v_x,
          - v_y);

	/* Output instance name */
  fprintf(svgfile, "\t\t\t<title>%s</title>\n",
          data->name);

 	/* Output instance description */
  fprintf(svgfile, "\t\t\t<desc>%s</desc>\n",
          data->comment);

 	/* Output background image (optional) */
 	if (bgimage != (char *)NULL)
 	{
  	fprintf(svgfile, "\t\t\t<image x=\"%d\" y=\"%d\" width=\"%dpx\" height=\"%dpx\"\n",
	  	v_x,
			v_y,
			bgimgwidth,
			bgimgheight);
  	fprintf(svgfile, "\t\t\t\txlink:href=\"%s\" />\n",
  		bgimage);
	}
	
  /* Output routes */
  routes = solution->nvehicles;
  if (verbose)
    printf("Found %d routes\n", routes);
  for (i = 0; i < routes; i++)
  {
    routelen = solution->routelen[i];
    if (verbose)
      printf("Route #%d has length %d\n", i, routelen);
  	int red = (rand() % 0xff);
  	int green = (rand() % 0xff);
  	int blue = (rand() % 0xff);
  	
  	fprintf(svgfile, "<!-- Route #%d:", i);
    for (j = 0; j < routelen; j++)
    {
	    fprintf(svgfile, " %d", solution->routes[i][j]);
	   }
    fprintf(svgfile, " -->\n");
    fprintf(svgfile, "<path d=\"M%d %d\n",
            (int)data->dat->x[depot],
            (int)data->dat->y[depot]);
    for (j = 0; j < routelen; j++)
    {
	    fprintf(svgfile, "L%d %d\n",
    	        (int)data->dat->x[solution->routes[i][j]],
        	    (int)data->dat->y[solution->routes[i][j]]);
	}
	fprintf(svgfile, "Z\" style=\"fill:none;stroke:#%0.2x%0.2x%0.2x;stroke-width:%f\"/>\n",
			red,
      green,
      blue,
			stroke_width);
	
  }
  
  /* Output nodes */
  float label_x, label_y;
  float label_fontsize = (1.0 * node_radius);
  for (i = 0; i < dimension; i++)
  {
    fprintf(svgfile, "<!-- Node %d -->\n", i);
    if (usedots)
		{
	    fprintf(svgfile, "<circle cx=\"%d\" cy=\"%d\" r=\"%f\" style=\"fill:%s\" />\n",
	            (int)data->dat->x[i],
	            (int)data->dat->y[i],
	            stroke_width,
	            (i == 0 ? DEPOT_COLOR : NODE_STROKE));
		}
		else
		{
	    fprintf(svgfile, "<circle cx=\"%d\" cy=\"%d\" r=\"%f\" style=\"fill:%s;stroke:%s;stroke-width:%f\" />\n",
	            (int)data->dat->x[i],
	            (int)data->dat->y[i],
	            node_radius,
	            (labelize ? LABEL_BG : (i == 0 ? DEPOT_COLOR : CUSTOMER_COLOR)),
	            NODE_STROKE,
							stroke_width);

			if (labelize)
			{
				// Calculate horizontal offset for label text based on number of cyphers in the label
				label_x = - ((0.6 * label_fontsize) * (floor(log10(i + 1)) + 1) / 2);
				label_y = - (0.2 * label_fontsize);
				// Print node index
		    fprintf(svgfile, "<text x=\"%f\" y=\"%f\" font-family=\"%s\" font-size=\"%f\" fill=\"%s\" style=\"font-weight:bold\">%d</text>\n",
		            data->dat->x[i] + label_x,
		            data->dat->y[i] + label_y,
		            LABEL_FONT,
		            label_fontsize,
		            LABEL_TEXT,
								i + 1);
				// Print node demand
		    fprintf(svgfile, "<line style=\"stroke-width:%f;stroke:%s\" x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" />\n",
		            stroke_width / 8,
		            LABEL_TEXT,
		            data->dat->x[i] - node_radius,
		            data->dat->y[i],
		            data->dat->x[i] + node_radius,
		            data->dat->y[i]);
				// Calculate horizontal offset for label text based on number of cyphers in the label
				label_x = - ((0.6 * label_fontsize) * (floor(log10(data->demand[i])) + 1) / 2);
				label_y = (0.8 * label_fontsize);
				// Print node demand
		    fprintf(svgfile, "<text x=\"%f\" y=\"%f\" font-family=\"%s\" font-size=\"%f\" fill=\"%s\" style=\"font-weight:bold\">%d</text>\n",
		            data->dat->x[i] + label_x,
		            data->dat->y[i] + label_y,
		            LABEL_FONT,
		            label_fontsize,
		            LABEL_TEXT,
								data->demand[i]);
			}
		}
  }

  fprintf(svgfile, "\t\t\t</g>\n");
  fprintf(svgfile, "\t\t</g>\n");
  fprintf(svgfile, "\t</g>\n");

  fprintf(svgfile, "</svg>\n");
  fflush(svgfile);
  fclose(svgfile);
}

