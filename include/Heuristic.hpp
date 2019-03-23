#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

#include "../include/Map.hpp"

#include <limits.h>
#include <iostream>

#include <ompl/base/ScopedState.h>
#include <boost/program_options.hpp>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

class Heuristic
{
	public:
		Heuristic(Map map, float dijkstra_grid_resolution, State target, Vehicle vehicle);

		Map map;
		float dijkstra_grid_resolution;
		State target;
		Vehicle vehicle;

		int dijkstra_grid_x;
		int dijkstra_grid_y;

		double get_heuristic(State pos);
		float** d;

		double DubinCost(State,State,double);
		vector<State> DubinShot(State begin, State end, double min_radius);
};
#endif