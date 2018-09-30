#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

// #include "../src/Vehicle.cpp"
#include "../src/Map.cpp"
#define DX 250
#define DY 250
class Heuristic
{
public:
	typedef struct
	{
		int x,y;
		float dis;
	}smallestcost_2d;

	Heuristic (){}
	smallestcost_2d** h_vals;
	State target;
	void Dijkstra(Map map,State target);
};
#endif
