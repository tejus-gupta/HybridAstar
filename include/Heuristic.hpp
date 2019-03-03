#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

#include "../src/Map.cpp"


class Heuristic
{
	public:
		Heuristic ()
		{

		}

		typedef struct
		{
			int x,y;
			double dis;
		}smallestcost_2d;
		smallestcost_2d** h_vals;
		void Dijkstra(Map map,State target);

		double DubinCost(State,State,double);
		vector<State> DubinShot(State begin, State end, double min_radius);

};
#endif