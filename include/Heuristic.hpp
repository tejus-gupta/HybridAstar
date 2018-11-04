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

		typedef struct
		{
			int x,y,z;
			double cost,theta;
		}smallestcost_3d;

		smallestcost_2d** h_vals;
		smallestcost_3d*** dub_cost;
		State target;
		double min_radius;
		Heuristic (){}
		void Dijkstra(Map map,State target);
		void Dubins_write(char *);
		void Dubins_read(char *);
};
#endif