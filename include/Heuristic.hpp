#include "../include/Vehicle.hpp"
#define DX 100
#define DY 100
class Heuristic
{
public:
	Vehicle::State target;
	typedef struct
	{
		int x,y;
		float dis;
	}smallestcost_2d;
	smallestcost_2d** h_vals;
	void Dijkstra();
}