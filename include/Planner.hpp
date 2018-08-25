#include "../src/GUI.cpp"
// #include "../src/Map.cpp"
using namespace std;
class Planner
{
public:	
	Planner()
	{

	}
	Heuristic h_obj;
	vector<State> plan(State,State,bool**,Vehicle);
};