#include "../include/Heuristic.hpp"
#include <limits.h>

double Pi=M_PI;

#define D_X 200
#define D_Y 200

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

				if(!isvalid(neighbor))
                    continue;				
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

// Dubin's Path

double fmod(double a, double b)
{
    return a - b*floor(a / b);
}

double distance(State x, State y)
{
    return sqrt(round(x.x-y.x)*round(x.x-y.x) + round(x.y-y.y)*round(x.y-y.y));
}

class Dubins_Path{

    State start,end,*path;
    double c,cost;

    void cost1()
    {
        cost=0;
        State *prev=path;
        State *head=path->next;
        while(head!=NULL)
        {
            cost+=distance(*prev,*head);
            prev=head;
            head=head->next;
        }
    } 

    vector<double> LSL(double alpha, double beta, double d, int& flag)
    {
        flag=0;
        vector<double> val(3);
        double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
        double c_ab = cos(alpha-beta);
        double var = d + sa - sb;
        double check = 2 + d*d - 2*c_ab + 2*d*(sa-sb);
        if(check < 0) return val;
        flag=1;
        double temp = atan2(cb-ca,var);
        val[0]=fmod(-alpha+temp,2*Pi);
        val[1]=sqrt(check);
        val[2]=fmod(beta-temp,2*Pi);
        return val;
    }

    vector<double> RSR(double alpha, double beta, double d, int& flag)
    {
        flag=0;
        vector<double> val(3);
        double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
        double c_ab = cos(alpha-beta);
        double var = d - sa + sb;
        double check = 2 + d*d - 2*c_ab + 2*d*(sb-sa);
        if(check < 0) return val;
        flag=1;
        double temp = atan2(ca-cb,var);
        val[0]=fmod(alpha-temp,2*Pi);
        val[1]=sqrt(check);
        val[2]=fmod(-beta+temp,2*Pi);
        return val;
    }

    vector<double> LSR(double alpha, double beta, double d, int& flag)
    {
        flag=0;
        vector<double> val(3);
        double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
        double c_ab = cos(alpha-beta);
        double var = d + sa + sb;
        double check = -2 + d*d + 2*c_ab + 2*d*(sa+sb);
        if(check < 0) return val;
        flag=1;
        double p=sqrt(check);
        double temp = atan2(-cb-ca,var) - atan2(-2.0,p);
        val[0]=fmod(-alpha+temp,2*Pi);
        val[1]=p;
        val[2]=fmod(-beta+temp,2*Pi);
        return val;
    }

    vector<double> RSL(double alpha, double beta, double d, int& flag)
    {
        flag=0;
        vector<double> val(3);
        double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
        double c_ab = cos(alpha-beta);
        double var = d - sa - sb;
        double check = -2 + d*d + 2*c_ab - 2*d*(sa+sb);
        if(check < 0) return val;
        flag=1;
        double p=sqrt(check);
        double temp = atan2(cb+ca,var) - atan2(2.0,p);
        val[0]=fmod(alpha-temp,2*Pi);
        val[1]=p;
        val[2]=fmod(beta-temp,2*Pi);
        return val;
    }

    vector<double> RLR(double alpha, double beta, double d, int& flag)
    {
        flag=0;
        vector<double> val(3);
        double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
        double c_ab = cos(alpha-beta);
        double var = d - sa + sb;
        double check = 6 - d*d + 2*c_ab + 2*d*(sa-sb);
        check/=8;
        if(fabs(check) > 1) return val;
        flag=1;
        double p=fmod(2*Pi-acos(check),2*Pi);
        double temp = atan2(-cb+ca,var) - p/2;
        val[0]=fmod(alpha-temp,2*Pi);
        val[1]=p;
        val[2]=fmod(alpha-beta-val[0]+p,2*Pi);
        return val;
    }

    vector<double> LRL(double alpha, double beta, double d, int& flag)
    {
        flag=0;
        vector<double> val(3);
        double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
        double c_ab = cos(alpha-beta);
        double var = d + sa - sb;
        double check = 6 - d*d + 2*c_ab + 2*d*(-sa+sb);
        check/=8;
        if(fabs(check) > 1) return val;
        flag=1;
        double p=fmod(2*Pi-acos(check),2*Pi);
        double temp = atan2(-cb+ca,var) - p/2;
        val[0]=fmod(-alpha-temp,2*Pi);
        val[1]=p;
        val[2]=fmod(beta-alpha-val[0]+p,2*Pi);
        return val;
    }

    vector<State> generate_path(vector<double> length, string type, double c)
    {
        vector<State> path;
        State tea(0.0,0.0,0.0);
        path.push_back(tea);
        for(int i=0;i<3;i++)
        {
            double d,pd=0;
            if(type[i] == 'S') d=c;
            else d=D_S*Pi/180;
            while (pd < fabs(length[i]-d))
            {
                State temp=path.back();
                State new1;
                new1.x = temp.x + d/c*cos(temp.theta);
                new1.y = temp.y + d/c*sin(temp.theta);
                new1.theta = type[i]=='L'? temp.theta + d : type[i]=='S'? temp.theta :temp.theta-d;
                path.push_back(new1);
                pd += d;
            }
            d=length[i] - pd;
            State temp=path.back();
            State new1;
            new1.x = temp.x + d/c*cos(temp.theta);
            new1.y = temp.y + d/c*sin(temp.theta);
            new1.theta = type[i]=='L'? temp.theta + d : type[i]=='S'? temp.theta :temp.theta-d;
            path.push_back(new1);
            pd += d;
        }
        return path;
    }

    double find_cost(double alpha, double beta, double d, string& s)
    {
        // LSL, RSR, LSR, RSL, RLR, LRL
        double cost=1e9;
        vector<double> fin;
        int flag;
        double temp_c;
        vector<double> temp;

        //LSL
        temp = LSL(alpha,beta,d,flag);
        temp_c = c*fabs(temp[0]) + fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c;
            
        //RSR
        temp = RSR(alpha,beta,d,flag);
        temp_c = c*fabs(temp[0]) + fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c;
        
        //LSR
        temp = LSR(alpha,beta,d,flag);
        temp_c = c*fabs(temp[0]) + fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c;
            
        //RSL
        temp = RSL(alpha,beta,d,flag);
        temp_c = c*fabs(temp[0]) + fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c;
            
        //LRL
        temp = LRL(alpha,beta,d,flag);
        temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c;
            
        //RLR
        temp = RLR(alpha,beta,d,flag);
        temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c;
            
        return cost;
    }

    void dubins_path_origin(double ex, double ey, double eyaw, double c)
    {
        double dx=ex,dy=ey;
        double D=sqrt(dx*dx + dy*dy);
        double d = D*c;

        double theta = fmod(atan2(dy,dx),2*Pi);
        double alpha = fmod(-theta,2*Pi);
        double beta = fmod(eyaw-theta,2*Pi);

        string type;
        cost=find_cost(alpha,beta,d,type);
    }

    public:
        void dubins_path()
        {
            double rel_ex = end.x-start.x, rel_ey = end.y-start.y;
            double lex = cos(start.theta)*rel_ex + sin(start.theta)*rel_ey;
            double ley = cos(start.theta)*rel_ey - sin(start.theta)*rel_ex;
            double leyaw = end.theta - start.theta;

            dubins_path_origin(lex,ley,leyaw,c);
        }

        double ret_cost()
        {
            return cost;
        }

        State * ret_path()
        {
            return path;
        } 

        Dubins_Path(State x,State y,double z)
        {
            start=x;
            end=y;
            c=z;
            dubins_path();
        }
};

void Heuristic::Dubins_write(char *file)
{
    int DT = 360/D_S;
    ofstream dubin;
    dubin.open(file);

    if(min_radius < 1e-6)
        min_radius=1.0;
    State target(DX/2,DY/2,0);
    for(int i=0;i<DX;i++)
    {
        for(int j=0;j<DY;j++)
        {
            for(int k=0;k<DT;k++)
            {
                double theta = k*D_S*Pi/180;
                State start(i,j,theta);
                Dubins_Path temp(start,target,min_radius);
                dubin<<temp.ret_cost()<<" ";
            }
        }
    }
}

void Heuristic::Dubins_read(char *file)
{
    dub_cost = new smallestcost_3d**[DX];
    int DT = 360/D_S;
    ifstream dubin;
    dubin.open(file);

    for(int i=0;i<DX;i++)
    {
        dub_cost[i] = new smallestcost_3d*[DY];
        for(int j=0;j<DY;j++)
            dub_cost[i][j] = new smallestcost_3d[DT];
    }

    for(int i=0;i<DX;i++)
    {
        for(int j=0;j<DY;j++)
        {
            for(int k=0;k<DT;k++)
            {
                double theta = k*D_S*Pi/180;
                smallestcost_3d *val=&dub_cost[i][j][k];
                val->x=i,val->y=j,val->z=k,val->theta=theta;
                dubin>>val->cost;
            }
        }
    }
}
