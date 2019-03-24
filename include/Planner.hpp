#ifndef PLANNER_HPP
#define PLANNER_HPP

#include "../include/GUI.hpp"
#include "../include/Map.hpp"
#include "../include/Heuristic.hpp"

#include <limits.h>
#include <pthread.h>
#include <vector>

using namespace std;

class Planner
{
	public:	
		State*** visited_state;
		bool*** visited;

		int map_x;
		int map_y;

		float map_grid_resolution;
		float planner_grid_resolution;

		int planner_grid_x;
		int planner_grid_y;
		int planner_grid_theta;
		
		vector<State> path;

		Planner(int map_x, int map_y, float map_grid_resolution, float planner_grid_resolution);
		vector<State> plan(State start, State end, Vehicle car, int** obstacles, GUI display);

		//for timing
		float map_init_time;
		float dijkstra_time;
};

#endif
