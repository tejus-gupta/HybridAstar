#include "../include/Heuristic.hpp"
#include <limits.h>
struct compare{
    bool operator() (const smallestcost_2d p1,const smallestcost_2d p2 ){
         return p1.dis<p2.dis;
    }
};
bool isvalid (smallestcost_2d neighbor)
{
	if (a.x<0||a.y<0||a.x>=DX||a.y>=DY)
		return false;
	return true;
}
float distance (smallestcost_2d source,smallestcost_2d neighbor)
{
	return (sqrt((source.x-neighbor.x)*(source.x-neighbor.x)+(source.y-neighbor.y)*(source.y-neighbor.y)));
}
void Heuristic::Dijkstra()
{
	priority_queue <smallestcost_2d,compare> pq;
	h_vals=new smallestcost_2d*[DX];
	for(int i=0;i<DX;i++)
	{
		h_vals[i]=new smallestcost_2d[DY];
	}
	for(int i=0;i<DX;i++)
	{
		for (int j=0;j<DY;j++)
		{
			h_vals[i][j].dis=FLT_MAX;
		}
	}
	h_vals[target.gx][target.gy].dis=0;
	h_vals[target.gx][target.gy].x=target.gx;
	h_vals[target.gx][target.gy].y=target.gy;
	pq.push(h_vals[target.gx][target.gy]);
	while (pq.size()>0)
	{
		smallestcost_2d temp;
		temp=pq.top();
		pq.pop();
		for (int i=temp.x-1;i<=temp.x+1;i++)
		{
			for (int j=temp.y-1;j<=temp.y+1;j++)
			{
				smallestcost_2d neighbor;
				neighbor.x=i;
				neighbor.y=j;
				if (i!=temp.x&&j!=temp.y&&isvalid(neighbor)&&obs_map[i][j]!=1)
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
