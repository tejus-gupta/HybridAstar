#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

// #include "../src/Vehicle.cpp"
#include "../src/Map.cpp"
#define DX 250
#define DY 250
#define D_S 5.0
class Heuristic
{
	public:
		typedef struct
		{
			int x,y;
			float dis;
		}smallestcost_2d;

		smallestcost_2d** h_vals;
		double*** dub_cost;
		State target;
		Heuristic (){}
		void Dijkstra(Map map,State target);
		void Dubins(double min_radius);
};
#endif