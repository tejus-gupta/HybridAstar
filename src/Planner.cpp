#include "../include/Planner.hpp"

void Planner::plan(State start, State end, bool** obs_map, Vehicle car)
{
	Map map(obs_map,end);//object of Map class
	bool visited[map.MAPX][map.MAPY][map.MAP_THETA]={0};//To mark the visited states. MAXX, MAXY and MAX_THETA are to be imported from the map class
	priority_queue<State, Heuristic::compare> pq;

	pq.push(start);
	while(true)
	{
		State current=pb.top();
		pq.pop();

		visited[current.x][current.y][current.theta]=1;
		if(Map::isReached(current))//checks if it has reached the goal
			break;
		vector <State> next=car.nextStates(current);
		for(vector <State>::iterator it= next.begin(); it!=next.end();it++)
		{
			if(visited[*it.x][*it.y][*it.theta])
				continue;
			if(map.checkCollision(*it))
				pq.push(*it);

		}
	}
}
int main()
{

}