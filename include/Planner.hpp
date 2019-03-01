#include "../src/GUI.cpp"
// #include "../src/Map.cpp"
#include "../src/Heuristic.cpp"

using namespace std;


typedef struct _dubins
{
	State initial;
	State final;
	double radius;
	double dubins_cost;
}Dubins;

class Planner
{
	public:	
		Heuristic h_obj;
		Planner(){}
		bool operator()(State a,State b);
		vector<State> path;
		vector<State> plan(State,State,vector<vector<bool> > ,Vehicle, vector<vector<Point>> obs, float scale);
};