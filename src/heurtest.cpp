#include "../include/Heuristic.hpp"
#include <limits.h>
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
typedef struct
{
	int x,y;
	double dis;
}smallestcost_2d;
smallestcost_2d** h_vals;

bool operator < (smallestcost_2d a,smallestcost_2d b)
{
	return (a.dis<b.dis);
}
bool isvalid (smallestcost_2d neighbor)
{
	// cout<<"neighbor = "<<neighbor.x<<"::"<<neighbor.y<<endl;
	if (neighbor.x<0||neighbor.y<0||neighbor.x>=DX||neighbor.y>=DY)
		return false;
	return true;
}
float distance (smallestcost_2d source,smallestcost_2d neighbor)
{
	return (sqrt((source.x-neighbor.x)*(source.x-neighbor.x)+(source.y-neighbor.y)*(source.y-neighbor.y)));
}
void Dijkstra(bool** obs_map,int a,int b)
{
	bool** is_visited;
	is_visited=new bool*[DX];
	for (int i=0;i<DX;i++)
	{
		is_visited[i]=new bool[DY];
		for (int j=0;j<DY;j++)
		{
			is_visited[i][j]=0;
		}
	}
	priority_queue <smallestcost_2d> pq;
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
	cout<<"#########@@@@@@@@@@@&&&&&&&&&&&&343"<<endl;
	h_vals[a][b].dis=0;
	h_vals[a][b].x=a;
	h_vals[a][b].y=b;
	is_visited[a][b]=1;
	pq.push(h_vals[a][b]);
	int count=0;
	while (pq.size()>0)
	{
		smallestcost_2d temp;
		temp=pq.top();
		pq.pop();			
		count++;
		cout<<count<<endl;	
		 cout<<"#########&&&&&&&&&&&&343"<<endl;
		is_visited[temp.x][temp.y]=true;
		for (int i=temp.x-1;i<=temp.x+1;i++)
		{
			for (int j=temp.y-1;j<=temp.y+1;j++)
			{
				smallestcost_2d neighbor;
				neighbor.x=i;
				neighbor.y=j;

				if(!isvalid(neighbor)) continue;
											cout<<"######&&&&&&&&&&&43 = "<<obs_map[i][j]<<endl;

				if (i!=temp.x&&j!=temp.y&&obs_map[i][j]!=1&&is_visited[i][j]==false)
				{
								 cout<<"#########&&&&&&&&&&&&343"<<endl;

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
int main()
{
	// Mat img;
	// img=imread("map.jpg",0);
	Mat obs_img = imread("../maps/map.jpg", 0);
    int h = obs_img.rows, w = obs_img.cols;    
  
    bool** obs_map = new bool*[h]; 
    for(int i=0; i<h; i++)  
    {
        obs_map[i] = new bool[w];   
        for(int j=0; j< w; j++)
            obs_map[i][j] = !(obs_img.at<uchar>(j, h-i) >= 120);
    }
    Dijkstra(obs_map,900,900);
    cout<<h_vals[800][800].dis<<endl;
    cout<<h_vals[400][400].dis<<endl;
    cout<<h_vals[10][10].dis<<endl;   
}