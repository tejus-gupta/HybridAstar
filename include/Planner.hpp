#include "../src/GUI.cpp"
#include "../src/Map.cpp"
#include "../src/Heuristic.cpp"

using namespace std;

class Planner
{
public:	
	Planner(){}
	Heuristic h_obj;
	bool operator()(State a,State b);
	vector<State> plan(State,State,bool**,Vehicle);
};