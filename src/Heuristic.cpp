#include "../include/Heuristic.hpp"
#include <limits.h>

class compareHeur
{
public:
	bool operator () (Heuristic::smallestcost_2d a,Heuristic::smallestcost_2d b)
	{
		return (a.dis>b.dis);
	}
};
bool isvalid (Heuristic::smallestcost_2d neighbor)
{
	// cout<<"neighbor = "<<neighbor.x<<"::"<<neighbor.y<<endl;
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
		cout<<map.obs_map[800][800]<<"Inside Dijkstra"<<endl;
	bool** is_visited;
	is_visited=new bool*[DX];
	for (int i=0;i<DX;i++)
	{
		is_visited[i]=new bool[DY];
		for (int j=0;j<DY;j++)
		{
			is_visited[i][j]=false;
		}
	}
	priority_queue <smallestcost_2d,vector<smallestcost_2d>,compareHeur> pq;
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
	//cout<<"#########@@@@@@@@@@@&&&&&&&&&&&&343"<<endl;
	h_vals[target.gx][target.gy].dis=0;
	h_vals[target.gx][target.gy].x=target.gx;
	h_vals[target.gx][target.gy].y=target.gy;
	is_visited[target.gx][target.gy]=1;
	pq.push(h_vals[target.gx][target.gy]);
	int count=0;
	while (pq.size()>0)
	{
		smallestcost_2d temp;
		temp=pq.top();
		pq.pop();			
		count++;
		// cout<<count<<endl;	
		 //cout<<"#########&&&&&&&&&&&&343"<<endl;
		is_visited[temp.x][temp.y]=true;
		for (int i=temp.x-1;i<=temp.x+1;i++)
		{
			for (int j=temp.y-1;j<=temp.y+1;j++)
			{
				smallestcost_2d neighbor;
				neighbor.x=i;
				neighbor.y=j;

				if(!isvalid(neighbor)) continue;
											//cout<<"######&&&&&&&&&&&43 = "<<map.obs_map[i][j]<<endl;

				if (i!=temp.x&&j!=temp.y&&map.obs_map[i][j]!=1&&is_visited[i][j]==false)
				{
								 //cout<<"#########&&&&&&&&&&&&343"<<endl;

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
	cout<<h_vals[800][800].dis<<endl;
    cout<<h_vals[400][400].dis<<endl;
   	cout<<h_vals[10][10].dis<<endl;  
}
