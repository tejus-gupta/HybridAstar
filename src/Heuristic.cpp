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

	is_visited[(int)(target.x)][(int)(target.y)]=true;

	h_vals[(int)(target.x)][(int)(target.y)].dis=0;
	h_vals[(int)(target.x)][(int)(target.y)].x=(int)(target.x);
	h_vals[(int)(target.x)][(int)(target.y)].y=(int)(target.y);
	pq.push(h_vals[(int)(target.x)][(int)(target.y)]);


	while (pq.size()>0)
	{
		smallestcost_2d temp;
		temp=pq.top();
		pq.pop();
		is_visited[temp.x][temp.y]=true;

		for (int i=temp.x-1;i<=temp.x+1;i++)
			for (int j=temp.y-1;j<=temp.y+1;j++)
			{
				smallestcost_2d neighbor;
				neighbor.x=i;
				neighbor.y=j;

				if(!map.isValid({neighbor.x, neighbor.y}))
                    continue;				
				
                if ( map.obs_map.at<uchar>(map.RES*i,map.RES*j)==0 && is_visited[i][j]==false )
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

// Dubin's Path
double Heuristic::DubinCost(State begin, State end, double radius)
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
