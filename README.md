# beluga-vrp-solver

Beluga is a heuristic solver for Capacitated Vehicle Routing Problems. The solver is available as a CLI tool, `beluga`. It's possible to launch the solver against a specific VRP instance, or to generate and solve a random instance with a given number of nodes.

The `beluga` tool accepts as input a file in the _TSPLIB_ format (http://branchandcut.org/VRP/data/) and produces the solution found as output in the standard _tourfile_ format used in literature to describe known optimum solutions, besides several temporary files for debugging purposes.

Beluga was written as part of my master thesis in Operations Research on heuristics for Vehicle Routing Problem for my CS degree.

# License

Copyright (c) 2005-2006 Claudio Procida

Beluga is released under the [GPL-2.0 license](https://opensource.org/licenses/GPL-2.0).