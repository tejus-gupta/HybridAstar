#include <bits/stdc++.h>
using namespace std;
class Planner
{
private:
	struct state
	{
		float x;
		float y;
		float theta;
		struct state* parent;
		// Motion primitives
	}

public:	
	void plan(struct state, struct state, int**);

}

void Planner::plan(struct state start, struct state end, int** obs_map)
{
	int visited[MAPX][MAPY][MAP_THETA]={0};//To mark the visited states. MAXX, MAXY and MAX_THETA are to be imported from the map class
	Map map(obs_map);//object of Map class
	Vehicle car;//object of vehicle class
	priority_queue<state, Heuristic::compare> pq;

	pq.push(start);
	while(true)
	{
		struct state current=pb.top();
		pq.pop();

		visited[current.x][current.y][current.theta]=1;
		if(Map::isreached(current))//checks if it has reached the goal
			break;
		vector <state> next=car.next_states(state);
		for(vector <state>::iterator it= next.begin(); it!=next.end();it++)
		{
			if(visited[*it.x][*it.y][*it.theta])
				continue;
			if(map.is_collision_free(it))
				pq.push(*it);

		}
	}
}
int main()
{

}