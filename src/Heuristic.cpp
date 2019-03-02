#include "../include/Heuristic.hpp"
#include <limits.h>
#include <iostream>

#include <ompl/base/ScopedState.h>
#include <boost/program_options.hpp>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
 
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class compareHeuristic{
 public:
	bool operator ()(Heuristic::smallestcost_2d a,Heuristic::smallestcost_2d b)
	{
		return (a.dis>b.dis);
	}
};

float distance (Heuristic::smallestcost_2d source,Heuristic::smallestcost_2d neighbor)
{
	return (sqrt((source.x-neighbor.x)*(source.x-neighbor.x)+(source.y-neighbor.y)*(source.y-neighbor.y)));
}

void Heuristic::Dijkstra(Map map,State target)
{
 	priority_queue <smallestcost_2d,vector<smallestcost_2d>,compareHeuristic> pq;

	cv::Mat grid(map.VISX, map.VISY, CV_8UC1, Scalar(255));
	cv::drawContours(grid, map.obs, -1, Scalar(0), -1);

    cv::transpose(grid,grid);
    cv::imshow("Obstacle Image",grid);
    cv::waitKey(0);

	h_vals=new smallestcost_2d*[map.VISX];
	for(int i=0;i<map.VISX;i++)
	{
		h_vals[i]=new smallestcost_2d[map.VISY];
        	for (int j=0;j<map.VISY;j++)
			h_vals[i][j].dis=FLT_MAX;
	}

	bool is_visited[map.VISX][map.VISY];
	for (int i=0;i<map.VISX;i++)
		for (int j=0;j<map.VISY;j++)
			is_visited[i][j]=false;

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

				if(!map.isValid({neighbor.x, neighbor.y}))
                    			continue;				
				if ( grid.at<uchar>(i,j)!=0 && is_visited[i][j]==false )
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

// double fmod(double a, double b)
// {
//     return a - b*floor(a / b);
// }

// double distance(State x, State y)
// {
//     return sqrt(round(x.x-y.x)*round(x.x-y.x) + round(x.y-y.y)*round(x.y-y.y));
// }

// class Dubins_Path{

//     State start,end,*path;
//     double c,cost;

//     void cost1()
//     {
//         cost=0;
//         State *prev=path;
//         State *head=path->next;
//         while(head!=NULL)
//         {
//             cost+=distance(*prev,*head);
//             prev=head;
//             head=head->next;
//         }
//     } 

//     vector<double> LSL(double alpha, double beta, double d, int& flag)
//     {
//         flag=0;
//         vector<double> val(3);
//         double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
//         double c_ab = cos(alpha-beta);
//         double var = d + sa - sb;
//         double check = 2 + d*d - 2*c_ab + 2*d*(sa-sb);
//         if(check < 0) return val;
//         flag=1;
//         double temp = atan2(cb-ca,var);
//         val[0]=fmod(-alpha+temp,2*M_PI);
//         val[1]=sqrt(check);
//         val[2]=fmod(beta-temp,2*M_PI);
//         return val;
//     }

//     vector<double> RSR(double alpha, double beta, double d, int& flag)
//     {
//         flag=0;
//         vector<double> val(3);
//         double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
//         double c_ab = cos(alpha-beta);
//         double var = d - sa + sb;
//         double check = 2 + d*d - 2*c_ab + 2*d*(sb-sa);
//         if(check < 0) return val;
//         flag=1;
//         double temp = atan2(ca-cb,var);
//         val[0]=fmod(alpha-temp,2*M_PI);
//         val[1]=sqrt(check);
//         val[2]=fmod(-beta+temp,2*M_PI);
//         return val;
//     }

//     vector<double> LSR(double alpha, double beta, double d, int& flag)
//     {
//         flag=0;
//         vector<double> val(3);
//         double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
//         double c_ab = cos(alpha-beta);
//         double var = d + sa + sb;
//         double check = -2 + d*d + 2*c_ab + 2*d*(sa+sb);
//         if(check < 0) return val;
//         flag=1;
//         double p=sqrt(check);
//         double temp = atan2(-cb-ca,var) - atan2(-2.0,p);
//         val[0]=fmod(-alpha+temp,2*M_PI);
//         val[1]=p;
//         val[2]=fmod(-beta+temp,2*M_PI);
//         return val;
//     }

//     vector<double> RSL(double alpha, double beta, double d, int& flag)
//     {
//         flag=0;
//         vector<double> val(3);
//         double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
//         double c_ab = cos(alpha-beta);
//         double var = d - sa - sb;
//         double check = -2 + d*d + 2*c_ab - 2*d*(sa+sb);
//         if(check < 0) return val;
//         flag=1;
//         double p=sqrt(check);
//         double temp = atan2(cb+ca,var) - atan2(2.0,p);
//         val[0]=fmod(alpha-temp,2*M_PI);
//         val[1]=p;
//         val[2]=fmod(beta-temp,2*M_PI);
//         return val;
//     }

//     vector<double> RLR(double alpha, double beta, double d, int& flag)
//     {
//         flag=0;
//         vector<double> val(3);
//         double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
//         double c_ab = cos(alpha-beta);
//         double var = d - sa + sb;
//         double check = 6 - d*d + 2*c_ab + 2*d*(sa-sb);
//         check/=8;
//         if(fabs(check) > 1) return val;
//         flag=1;
//         double p=fmod(2*M_PI-acos(check),2*M_PI);
//         double temp = atan2(-cb+ca,var) - p/2;
//         val[0]=fmod(alpha-temp,2*M_PI);
//         val[1]=p;
//         val[2]=fmod(alpha-beta-val[0]+p,2*M_PI);
//         return val;
//     }

//     vector<double> LRL(double alpha, double beta, double d, int& flag)
//     {
//         flag=0;
//         vector<double> val(3);
//         double sa=sin(alpha),sb=sin(beta),ca=cos(alpha),cb=cos(beta);
//         double c_ab = cos(alpha-beta);
//         double var = d + sa - sb;
//         double check = 6 - d*d + 2*c_ab + 2*d*(-sa+sb);
//         check/=8;
//         if(fabs(check) > 1) return val;
//         flag=1;
//         double p=fmod(2*M_PI-acos(check),2*M_PI);
//         double temp = atan2(-cb+ca,var) - p/2;
//         val[0]=fmod(-alpha-temp,2*M_PI);
//         val[1]=p;
//         val[2]=fmod(beta-alpha-val[0]+p,2*M_PI);
//         return val;
//     }

//     vector<State> generate_path(vector<double> length, string type, double c)
//     {
//         vector<State> path;
//         State tea(0.0,0.0,0.0);
//         path.push_back(tea);
//         for(int i=0;i<3;i++)
//         {
//             double d,pd=0;
//             if(type[i] == 'S') d=c;
//             else d=D_S*M_PI/180;
//             while (pd < fabs(length[i]-d))
//             {
//                 State temp=path.back();
//                 State new1;
//                 new1.x = temp.x + d/c*cos(temp.theta);
//                 new1.y = temp.y + d/c*sin(temp.theta);
//                 new1.theta = type[i]=='L'? temp.theta + d : type[i]=='S'? temp.theta :temp.theta-d;
//                 path.push_back(new1);
//                 pd += d;
//             }
//             d=length[i] - pd;
//             State temp=path.back();
//             State new1;
//             new1.x = temp.x + d/c*cos(temp.theta);
//             new1.y = temp.y + d/c*sin(temp.theta);
//             new1.theta = type[i]=='L'? temp.theta + d : type[i]=='S'? temp.theta :temp.theta-d;
//             path.push_back(new1);
//             pd += d;
//         }
//         return path;
//     }

//     double find_cost(double alpha, double beta, double d, string& s)
//     {
//         // LSL, RSR, LSR, RSL, RLR, LRL
//         double cost=1e9;
//         vector<double> fin;
//         int flag;
//         double temp_c;
//         vector<double> temp;

//         //LSL
//         temp = LSL(alpha,beta,d,flag);
//         temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
//         if(cost>temp_c && flag)
//             cost=temp_c,s="LSL";
            
//         //RSR
//         temp = RSR(alpha,beta,d,flag);
//         temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
//         if(cost>temp_c && flag)
//             cost=temp_c,s="RSR";
        
//         //LSR
//         temp = LSR(alpha,beta,d,flag);
//         temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
//         if(cost>temp_c && flag)
//             cost=temp_c,s="LSR";
            
//         //RSL
//         temp = RSL(alpha,beta,d,flag);
//         temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
//         if(cost>temp_c && flag)
//             cost=temp_c,s="RSL";
            
//         //LRL
//         temp = LRL(alpha,beta,d,flag);
//         temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
//         if(cost>temp_c && flag)
//             cost=temp_c,s="LRL";
            
//         //RLR
//         temp = RLR(alpha,beta,d,flag);
//         temp_c = c*fabs(temp[0]) + c*fabs(temp[1]) + c*fabs(temp[2]);
//         if(cost>temp_c && flag)
//             cost=temp_c,s="RLR";
            
//         return cost;
//     }
    
//     void dubins_path_origin(double ex, double ey, double eyaw, double c)
//     {
//         double dx=ex,dy=ey;
//         double D=sqrt(dx*dx + dy*dy);
//         double d = D/c;

//         double theta = fmod(atan2(dy,dx),2*M_PI);
//         double alpha = fmod(-theta,2*M_PI);
//         double beta = fmod(eyaw-theta,2*M_PI);

//         string type;
//         cost=find_cost(alpha,beta,d,type);
//     }

//     public:
//         void dubins_path()
//         {
//             double rel_ex = end.x-start.x, rel_ey = end.y-start.y;
//             double lex = cos(start.theta)*rel_ex + sin(start.theta)*rel_ey;
//             double ley = cos(start.theta)*rel_ey - sin(start.theta)*rel_ex;
//             double leyaw = end.theta - start.theta;

//             dubins_path_origin(lex,ley,leyaw,c);
//         }

//         double ret_cost()
//         {
//             return cost;
//         }

//         State * ret_path()
//         {
//             return path;
//         } 

//         void change(State x,State y,double z)
//         {
//             start=x;
//             end=y;
//             c=z;
//             dubins_path();
//         }
// };

// void Heuristic::Dubins(double radius)
// {
//     State target(DX,DY,0);
//     dub_cost = new double**[2*DX];
//     int DT = 360/D_S;
    
//     for(int i=0;i<2*DX;i++)
//     {
//         dub_cost[i] = new double*[2*DY];
//         for(int j=0;j<2*DY;j++)
//             dub_cost[i][j] = new double[DT];
//     }

//     Dubins_Path temp;
//     for(int i=0;i<2*DX;i++)
//     {
//         for(int j=0;j<2*DY;j++)
//         {
//             for(int k=0;k<DT;k++)
//             {
//                 double theta = k*D_S*M_PI/180;
//                 State start(i,j,theta);
                
//                 temp.change(start,target,radius);
//                 double *val=&dub_cost[i][j][k];

//                 *val=temp.ret_cost();
//             }
//         }
//     }
// }

double Heuristic::Dubin_cost(State begin, State end, double radius)
{
	bool DEBUG=false;
    vector<State> nextStates;

    // Declaration of an OMPL Coordinate System  
    ob::StateSpacePtr space(new ompl::base::SE2StateSpace());

    // Declaration of two states in this Coordinate System  
    ob::State *start = space->allocState();
    ob::State *goal  = space->allocState();

    // Declaration of variables that configures it for 2D motion
    auto *s = start->as<ob::SE2StateSpace::StateType>();
    auto *t = goal->as<ob::SE2StateSpace::StateType>();
    
    // Reference :
    // http://docs.ros.org/diamondback/aM_PI/ompl/html/classompl_1_1base_1_1SE2StateSpace_1_1StateType.html
    
    s->setX(begin.x);
    s->setY(begin.y);
    s->setYaw(begin.theta);

    t->setX(end.x);
    t->setY(end.y);
    t->setYaw(end.theta);
    
    if(DEBUG)
    {
        double x1=s->getX(), y1=s->getY() ,theta1=s->getYaw();
        double x2=t->getX(), y2=t->getY() ,theta2=t->getYaw();
        cout<<x1<<" "<<y1<<" "<<theta1<<" "<<x2<<" "<<y2<<" "<<theta2<<endl;
    }
    
    // http://ompl.kavrakilab.org/classompl_1_1base_1_1DubinsStateSpace_1_1DubinsPath.html
    ob::DubinsStateSpace DP(radius,true);
    auto Path=DP.dubins(start,goal);
    if(DEBUG)
    	cout<<"Path Length : "<<Path.length()<<endl;
    return Path.length()*radius;
}

vector<State> Heuristic::DubinShot(State begin, State end, double radius)
{
    bool DEBUG=false;
    vector<State> nextStates;

    // Declaration of an OMPL Coordinate System  
    ob::StateSpacePtr space(new ompl::base::SE2StateSpace());

    // Declaration of two states in this Coordinate System  
    ob::State *start = space->allocState();
    ob::State *goal  = space->allocState();

    // Declaration of variables that configures it for 2D motion
    auto *s = start->as<ob::SE2StateSpace::StateType>();
    auto *t = goal->as<ob::SE2StateSpace::StateType>();
    
    // Reference :
    // http://docs.ros.org/diamondback/aM_PI/ompl/html/classompl_1_1base_1_1SE2StateSpace_1_1StateType.html
    
    s->setX(begin.x);
    s->setY(begin.y);
    s->setYaw(begin.theta);

    t->setX(end.x);
    t->setY(end.y);
    t->setYaw(end.theta);
    
    if(DEBUG)
    {
        double x1=s->getX(), y1=s->getY() ,theta1=s->getYaw();
        double x2=t->getX(), y2=t->getY() ,theta2=t->getYaw();
        cout<<"Inside Dubins "<<x1<<" "<<y1<<" "<<theta1<<" "<<x2<<" "<<y2<<" "<<theta2<<endl;
    }
    
    // http://ompl.kavrakilab.org/classompl_1_1base_1_1DubinsStateSpace_1_1DubinsPath.html
    ob::DubinsStateSpace DP(radius,true);
    auto Path=DP.dubins(start,goal);

    if(DEBUG)
    {
        // This gives the total distance travelled along a curved path :DubinsCost
        cout<<"Dubins Distance : "<<Path.length()*radius<<endl;

        /*
        The type of dubins curve possible are:
        {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT},
        {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT},
        {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT},
        {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT},
        {DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT},
        {DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT}}

        Here in OMPL implementation the following enum are used :
        DUBINS_LEFT=0 , DUBINS_STRAIGHT=1, DUBINS_RIGHT=2
        */
        cout<<"Dubins Type : "<<Path.type_[0]<<" "<<Path.type_[1]<<" "<<Path.type_[2]<<endl;

        // It gives path length corresponding to each of the three component curves :
        cout<<"Length_0 :"<<Path.length_[0]<<endl;
        cout<<"Length_1 :"<<Path.length_[1]<<endl;
        cout<<"Length_2 :"<<Path.length_[2]<<endl;
    }

    int stride = 1,d=2;
    State E1,E2,next;

    if(DEBUG)
	    cout<<"Inside First Loop"<<endl;
    
    double alhpa1 = (Path.type_[0]==0)?1*Path.length_[0]:-1*Path.length_[0];
    if( Path.length_[0]>0.1 )
    {
        while(stride < Path.length_[0]*radius)
        {
            next.x = (begin.x - alhpa1/abs(alhpa1)*( radius*sin(begin.theta) - radius*sin( alhpa1*stride/(Path.length_[0]*radius) +begin.theta )));
            next.y = (begin.y + alhpa1/abs(alhpa1)*( radius*cos(begin.theta) - radius*cos( alhpa1*stride/(Path.length_[0]*radius) +begin.theta )));
            next.theta = begin.theta + (float)(alhpa1*stride/(Path.length_[0]*radius)) ;
            nextStates.push_back(next);        
            
            if(DEBUG)            
                cout<<next.x<<" "<< next.y <<" "<<next.theta<<endl;
            
            stride+=d;
        }

        E1.x = (begin.x - alhpa1/abs(alhpa1)*(radius*sin(begin.theta) - radius*sin( alhpa1+begin.theta )));
        E1.y = (begin.y + alhpa1/abs(alhpa1)*(radius*cos(begin.theta) - radius*cos( alhpa1+begin.theta )));
        E1.theta = begin.theta + alhpa1 ;
	    if(DEBUG)
			cout<<E1.x<<" "<< E1.y <<" "<<E1.theta<<endl;

        nextStates.push_back(E1);        
    }
    else
    {
        E1.x = begin.x;
        E1.y = begin.y; 
        E1.theta = begin.theta;
    }    
    
    if(DEBUG)
    	cout<<"Inside Second Loop"<<endl;
    stride = 1;
    
    if( Path.length_[1]>0.1 )
    {
        while( stride<Path.length_[1]*radius )
        {
            next.x = (E1.x + stride*cos(E1.theta) );
            next.y = (E1.y + stride*sin(E1.theta) );
            next.theta = E1.theta ;
            nextStates.push_back(next);

            if(DEBUG)            
                cout<<next.x<<" "<< next.y <<" "<<next.theta<<endl;
            
            stride+=d;
        }
        E2.x = (E1.x + Path.length_[1]*radius*cos(E1.theta) );
        E2.y = (E1.y + Path.length_[1]*radius*sin(E1.theta) );
        E2.theta = E1.theta ;
    
        if(DEBUG)
	        cout<<E2.x<<" "<< E2.y <<" "<<E2.theta<<endl;
    
        nextStates.push_back(E2);
    }
    else
    {
        E2.x = E1.x;
        E2.y = E1.y;
        E2.theta = E1.theta;
    }
    
    if(DEBUG)
    	cout<<"Inside Third Loop"<<endl;
    
    double alhpa2 = (Path.type_[2]==0)?1*Path.length_[2]:-1*Path.length_[2];
    stride = 1;
    if( Path.length_[2]>0.1 )
    {
        while( stride<Path.length_[2]*radius )
        {
           
            next.x = (E2.x - alhpa2/abs(alhpa2)*(radius*sin(E2.theta) - radius*sin( alhpa2*stride/(Path.length_[2]*radius)+E2.theta )));
            next.y = (E2.y + alhpa2/abs(alhpa2)*(radius*cos(E2.theta) - radius*cos( alhpa2*stride/(Path.length_[2]*radius)+E2.theta )));
            next.theta = E2.theta + (float)(alhpa2*stride/(Path.length_[2]*radius)) ;
            nextStates.push_back(next);

            if(DEBUG)                       
                cout<<next.x<<" "<< next.y <<" "<<next.theta<<endl;
            
            stride+=d;
        }
        next.x = (E2.x - alhpa2/abs(alhpa2)*(radius*sin(E2.theta) - radius*sin(alhpa2 + E2.theta )));
        next.y = (E2.y + alhpa2/abs(alhpa2)*(radius*cos(E2.theta) - radius*cos(alhpa2 + E2.theta )));
        next.theta = E2.theta + alhpa2 ;
	
        if(DEBUG)
			cout<<next.x<<" "<< next.y <<" "<<next.theta<<endl;
    
        nextStates.push_back(next);
    }
	if(DEBUG)
    	cout<<"End of Dubins Shot"<<endl;

    return nextStates;
}
// void Heuristic::Dubins(double radius)
// {
//     bool DEBUG=false;
    
//     // Declaration of an OMPL Coordinate System  
//     ob::StateSpacePtr space(new ompl::base::SE2StateSpace());

//     // Declaration of two states in this Coordinate System  
//     ob::State *start = space->allocState();
//     ob::State *goal  = space->allocState();

//     // Declaration of variables that configures it for 2D motion
//     auto *s = start->as<ob::SE2StateSpace::StateType>();
//     auto *t = goal->as<ob::SE2StateSpace::StateType>();
    
//     // Reference :
//     // http://docs.ros.org/diamondback/aM_PI/ompl/html/classompl_1_1base_1_1SE2StateSpace_1_1StateType.html

//     t->setX(DX);
//     t->setY(DY);
//     t->setYaw(0);
    
//     if(DEBUG)
//     {
//         double x1=s->getX(), y1=s->getY() ,theta1=s->getYaw();
//         double x2=t->getX(), y2=t->getY() ,theta2=t->getYaw();
//         cout<<x1<<" "<<y1<<" "<<theta1<<" "<<x2<<" "<<y2<<" "<<theta2<<endl;
//     }
    
//     // http://ompl.kavrakilab.org/classompl_1_1base_1_1DubinsStateSpace_1_1DubinsPath.html
//     ob::DubinsStateSpace DP(radius,true);

//     dub_cost = new double**[2*DX];    
//     for(int i =0 ;i < 2*DX ;i++)
//     {
//         dub_cost[i] = new double*[2*DY];
//         for(int j = 0;j < 2*DY; j++)
//             dub_cost[i][j] = new double[(int)(360/D_S)];
//     }

//     for(int i = 0; i < 2*DX; i++)
//     {
//         for(int j = 0; j < 2*DY; j++)
//         {
//             for(int k = 0; k < 360/D_S; k++)
//             {
//                 double theta = k*D_S*M_PI/180;

//                 s->setX(i);
//                 s->setY(j);
//                 s->setYaw(theta);
                
//                 auto Path=DP.dubins(start,goal);
//                 dub_cost[i][j][k]=Path.length();
//             }
//         }
//     }

// }
