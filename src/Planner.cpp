#include "../include/Planner.hpp"
Planner p;
bool operator < (State a,State b)
{
	return (p.h_obj.h_vals[a.gx][a.gy].dis<p.h_obj.h_vals[b.gx][b.gy].dis);
}
void Planner::plan(State start, State end, bool** obs_map, Vehicle car)
{
	Map map(obs_map,end);//object of Map class
	bool visited[map.MAPX][map.MAPY][map.MAP_THETA]={0};//To mark the visited states. MAXX, MAXY and MAX_THETA are to be imported from the map class
	priority_queue < State> pq;
	pq.push(start);
	while(true)
	{
		State current=pq.top();
		pq.pop();
		visited[current.gx][current.gy][(int)current.theta]=1;
		if(map.isReached(current))//checks if it has reached the goal
			break;
		vector <State> next=car.nextStates(current);
		for(vector <State>::iterator it= next.begin(); it!=next.end();it++)
		{
			State s;
			s=*it;
			if(visited[s.gx][s.gy][(int)s.theta])
				continue;
			if(map.checkCollision(s))
				pq.push(s);

		}
	}
}
