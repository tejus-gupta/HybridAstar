#include "../include/Heuristic.hpp"
#include <limits.h>

double Pi=M_PI;
int xlimit,ylimit;

// #define D_X 200
// #define D_Y 200

class compareHeuristic{
 public:
	bool operator ()(Heuristic::smallestcost_2d a,Heuristic::smallestcost_2d b)
	{
		return (a.dis>b.dis);
	}
};

bool isvalid (Heuristic::smallestcost_2d neighbor)
{
	if (neighbor.x<0||neighbor.y<0||neighbor.x>=xlimit||neighbor.y>=ylimit)
		return false;
	return true;
}

float distance (Heuristic::smallestcost_2d source,Heuristic::smallestcost_2d neighbor)
{
	return (sqrt((source.x-neighbor.x)*(source.x-neighbor.x)+(source.y-neighbor.y)*(source.y-neighbor.y)));
}

void Heuristic::Dijkstra(Map map,State target)
{
    xlimit = map.VISX, ylimit = map.VISY;
	priority_queue <smallestcost_2d,vector<smallestcost_2d>,compareHeuristic> pq;

	h_vals=new smallestcost_2d*[map.VISX];
	for(int i=0;i<map.VISX;i++)
	{
		h_vals[i]=new smallestcost_2d[map.VISY];
        for (int j=0;j<map.VISY;j++)
			h_vals[i][j].dis=FLT_MAX;
	}

	bool **is_visited=new bool*[map.VISX];
	for (int i=0;i<map.VISX;i++)
	{
		is_visited[i]=new bool[map.VISY];
		for (int j=0;j<map.VISY;j++)
		{
			is_visited[i][j]=false;
		}
	}

	is_visited[target.gx][target.gy]=true;

	h_vals[target.gx][target.gy].dis=0;
	h_vals[target.gx][target.gy].x=target.gx;
	h_vals[target.gx][target.gy].y=target.gy;
	pq.push(h_vals[target.gx][target.gy]);

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
				if (map.obs_map[i][j]!=1 && is_visited[i][j]==false )
				{
					if (h_vals[i][j].dis > h_vals[temp.x][temp.y].dis+distance(temp,neighbor))
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
        temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c,s="LSL";
            
        //RSR
        temp = RSR(alpha,beta,d,flag);
        temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c,s="RSR";
        
        //LSR
        temp = LSR(alpha,beta,d,flag);
        temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c,s="LSR";
            
        //RSL
        temp = RSL(alpha,beta,d,flag);
        temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c,s="RSL";
            
        //LRL
        temp = LRL(alpha,beta,d,flag);
        temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c,s="LRL";
            
        //RLR
        temp = RLR(alpha,beta,d,flag);
        temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
        if(cost>temp_c && flag)
            cost=temp_c,s="RLR";
            
        return cost;
    }
    
    void dubins_path_origin(double ex, double ey, double eyaw, double c)
    {
        double dx=ex,dy=ey;
        double D=sqrt(dx*dx + dy*dy);
        double d = D/c;

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

        void change(State x,State y,double z)
        {
            start=x;
            end=y;
            c=z;
            dubins_path();
        }
};

void Heuristic::Dubins(double min_radius)
{
    State target(DX,DY,0);
    dub_cost = new double**[2*DX];
    int DT = 360/D_S;
    
    for(int i=0;i<2*DX;i++)
    {
        dub_cost[i] = new double*[2*DY];
        for(int j=0;j<2*DY;j++)
            dub_cost[i][j] = new double[DT];
    }

    Dubins_Path temp;
    for(int i=0;i<2*DX;i++)
    {
        for(int j=0;j<2*DY;j++)
        {
            for(int k=0;k<DT;k++)
            {
                double theta = k*D_S*Pi/180;
                State start(i,j,theta);
                
                temp.change(start,target,min_radius);
                double *val=&dub_cost[i][j][k];

                *val=temp.ret_cost();
            }
        }
    }
}

double Heuristic::Dubin_cost(State start, State end, double min_radius)
{
    Dubins_Path temp;
    temp.change(start,end,min_radius);
    return temp.ret_cost();
}