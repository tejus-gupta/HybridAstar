// #include "../src/Vehicle.cpp"
#include "../src/Map.cpp"
#define DX 1000
#define DY 1000
class Heuristic
{
public:
	typedef struct
	{
		int x,y;
		float dis;
	}smallestcost_2d;
	Heuristic ()
	{
		
	}
	smallestcost_2d** h_vals;
	//State target;
	void Dijkstra(Map map,State);
};