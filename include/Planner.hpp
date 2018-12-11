#include "../src/GUI.cpp"
// #include "../src/Map.cpp"
#include "../src/Heuristic.cpp"

using namespace std;

class Planner
{
	public:	
		Heuristic h_obj;
		Planner(){}
		bool operator()(State a,State b);
		vector<State> path;
		vector<State> plan(State,State,vector<vector<bool> > ,Vehicle, vector<vector<Point>> obs, float scale);
};