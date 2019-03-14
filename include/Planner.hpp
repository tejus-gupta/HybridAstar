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
	int rows,cols;
	public:	
		Heuristic h_obj;
		vector< vector< vector< State > > > visited_state;
		vector< vector< vector< bool > > > visited;
		Planner(int rows,int cols)
		{
			// Initialising rows and cols
			this->rows = rows;
			this->cols = cols;

			// Array of states allocation
			clock_t time_begin= clock();
			visited_state.resize(rows,vector< vector< State > >(cols,vector< State >(72)));	
			clock_t time_end= clock();
			cout<<"Time: Array of States Allocation = "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;

			// Array of visited allocation
			time_begin= clock();
			visited.resize(rows,vector< vector< bool > >(cols,vector< bool >(72,false)));
			time_end= clock();
			cout<<"Time: Visited Array of States Allocation = "<<double(time_end-time_begin)/CLOCKS_PER_SEC<<endl;
		}
		// bool operator()(State a,State b);
		vector<State> path;
		vector<State> plan(State, State, Vehicle, vector<vector<Point>> obs, GUI display);
};

class PriQ
{
	public:
		PriQ(){}
		bool operator()(State a,State b);
};
