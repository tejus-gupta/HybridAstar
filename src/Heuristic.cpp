#include "../include/Heuristic.hpp"
#include <limits.h>

class compareHeuristic{
 public:
	bool operator ()(Heuristic::smallestcost_2d a,Heuristic::smallestcost_2d b)
	{
		return (a.dis>b.dis);
	}
};


bool isvalid (Heuristic::smallestcost_2d neighbor)
{
	if (neighbor.x<0||neighbor.y<0||neighbor.x>=DX||neighbor.y>=DY)
		return false;
	return true;
}


float distance (Heuristic::smallestcost_2d source,Heuristic::smallestcost_2d neighbor)
{
	return (sqrt((source.x-neighbor.x)*(source.x-neighbor.x)+(source.y-neighbor.y)*(source.y-neighbor.y)));
}


void Heuristic::Dijkstra(Map map,State target)
{
	priority_queue <smallestcost_2d,vector<smallestcost_2d>,compareHeuristic> pq;

	int** grid_map=new int*[DX];
	for(int i=0;i<DX;i++)
	{
		grid_map[i]=new int[DY];
	}
	
	for(int i=0;i<map.MAPX;i++)
	{
		for(int j=0;j<map.MAPY;j++)
		{
			if(map.obs_map[i][j])
			grid_map[i*DX/map.MAPX][j*DY/map.MAPY]=1;
		}
	}
	h_vals=new smallestcost_2d*[DX];
	for(int i=0;i<DX;i++)
	{
		h_vals[i]=new smallestcost_2d[DY];
        for (int j=0;j<DY;j++)
			h_vals[i][j].dis=FLT_MAX;
	}

	bool **is_visited=new bool*[DX];
	for (int i=0;i<DX;i++)
	{
		is_visited[i]=new bool[DY];
		for (int j=0;j<DY;j++)
		{
			is_visited[i][j]=false;
		}
	}

	is_visited[target.gx*DX/map.MAPX][target.gy*DY/map.MAPY]=true;

	h_vals[target.gx*DX/map.MAPX][target.gy*DY/map.MAPY].dis=0;
	h_vals[target.gx*DX/map.MAPX][target.gy*DY/map.MAPY].x=target.gx*DX/map.MAPX;
	h_vals[target.gx*DX/map.MAPX][target.gy*DY/map.MAPY].y=target.gy*DY/map.MAPY;
	pq.push(h_vals[target.gx*DX/map.MAPX][target.gy*DY/map.MAPY]);


	while (pq.size()>0)
	{
		smallestcost_2d temp;
		temp=pq.top();
		pq.pop();
		is_visited[temp.x][temp.y]=true;

		for (int i=temp.x-1;i<=temp.x+1;i++)
		{
			for (int j=temp.y-1;j<=temp.y+1;j++)
			{
				smallestcost_2d neighbor;
				neighbor.x=i;
				neighbor.y=j;

				if(!isvalid(neighbor)) continue;				
				if ( grid_map[i][j]!=1 &&is_visited[i][j]==false )
				{
					if (h_vals[i][j].dis>h_vals[temp.x][temp.y].dis+distance(temp,neighbor))
					{
						h_vals[i][j].dis=h_vals[temp.x][temp.y].dis+distance(temp,neighbor);
						h_vals[i][j].x=i;
						h_vals[i][j].y=j;
						pq.push(h_vals[i][j]);
					}		 						
				}
			}
		}
	}


}
