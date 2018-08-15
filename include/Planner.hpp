#include "../src/Heuristic.cpp"
// #include "../src/Map.cpp"
using namespace std;
class Planner
{
public:	
	Planner();
	Heuristic h_obj;
	void plan(State,State,bool**,Vehicle);
};