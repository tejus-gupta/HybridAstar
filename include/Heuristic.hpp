// #include "../src/Vehicle.cpp"
#include "../src/Map.cpp"
#define DX 100
#define DY 100
class Heuristic
{
public:
	typedef struct
	{
		int x,y;
		float dis;
	}smallestcost_2d;
	Heuristic ();
	Map map;
	smallestcost_2d** h_vals;
	State target;
	void Dijkstra();
};