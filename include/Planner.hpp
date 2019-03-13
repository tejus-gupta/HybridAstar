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
		vector< vector< vector< State > > > visited_state;
		vector< vector< vector< bool > > > visited;
		Planner(){

			
		}
		bool operator()(State a,State b);
		vector<State> path;
		vector<State> plan(State, State, Vehicle, vector<vector<Point>> obs, GUI display, int rows, int cols);
};
