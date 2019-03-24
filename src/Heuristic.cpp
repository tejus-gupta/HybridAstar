#include "../include/Heuristic.hpp"
#include <boost/heap/fibonacci_heap.hpp>
 
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

struct point2d
{
    int x;
    int y;
    float cost;
    bool operator<(point2d other) const
    {
        return cost > other.cost;
    }
};

float distance (point2d source, point2d neighbor)
{
	return (sqrt((source.x-neighbor.x)*(source.x-neighbor.x)+(source.y-neighbor.y)*(source.y-neighbor.y)));
}

Heuristic::Heuristic(Map map, float dijkstra_grid_resolution, State target, Vehicle vehicle)
{
    this->map = map;
    this->dijkstra_grid_resolution = dijkstra_grid_resolution;
    this->target = target;
    this->vehicle = vehicle;

    dijkstra_grid_x = toInt(map.map_x/dijkstra_grid_resolution);
    dijkstra_grid_y = toInt(map.map_y/dijkstra_grid_resolution);

    int** obs = new int*[dijkstra_grid_x];
	for(int i=0;i<dijkstra_grid_x;i++)
	{
		obs[i]=new int[dijkstra_grid_y];
		for(int j=0;j<dijkstra_grid_y;j++)
			obs[i][j] = 0;
	}

    for(int i=0;i<map.map_grid_x;i++)
        for(int j=0;j<map.map_grid_y;j++)
            if(map.obs[i][j] > 0)
                obs[roundDown(i*map.map_grid_resolution/dijkstra_grid_resolution)][roundDown(j*map.map_grid_resolution/dijkstra_grid_resolution)] = 1;
    
    d = new float*[dijkstra_grid_x];
	for(int i=0;i<dijkstra_grid_x;i++)
	{
		d[i]=new float[dijkstra_grid_y];
		for(int j=0;j<dijkstra_grid_y;j++)
			d[i][j] = FLT_MAX;
	}

    bool** visited = new bool*[dijkstra_grid_x];
	for(int i=0;i<dijkstra_grid_x;i++)
	{
		visited[i]=new bool[dijkstra_grid_y];
		for(int j=0;j<dijkstra_grid_y;j++)
			visited[i][j] = false;
	}

    priority_queue <point2d, vector<point2d>> pq;

    point2d start, current, next;
    start.x = target.x/dijkstra_grid_resolution;
    start.y = target.y/dijkstra_grid_resolution;
    start.cost = 0;

    pq.push(start);

    while(!pq.empty())
    {
        current = pq.top();
        pq.pop();

        if(visited[current.x][current.y])
            continue;

        visited[current.x][current.y] = true;
        d[current.x][current.y] = current.cost;

        for(int i=-1;i<=1;i++)
            for(int j=-1;j<=1;j++)
            {
                if(current.x+i<0 || current.x+i>=dijkstra_grid_x || current.y+j<0 || current.y+j>=dijkstra_grid_x)
                    continue;
                
                if(obs[current.x+i][current.y+j] || visited[current.x+i][current.y+j])
                    continue;

                next.x = current.x + i;
                next.y = current.y + j;
                next.cost = current.cost + distance(current, next);

                if(next.cost<d[next.x][next.y])
                {
                    d[next.x][next.y] = next.cost;
                    pq.push(next);
                }
            }
    }
    
    return;
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
    // http://docs.ros.org/diamondback/api/ompl/html/classompl_1_1base_1_1SE2StateSpace_1_1StateType.html
    
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


double Heuristic::get_heuristic(State pos)
{
    //cout<<roundDown(pos.x/dijkstra_grid_resolution)<<","<<roundDown(pos.y/dijkstra_grid_resolution)<<endl;
    //cout<<dijkstra_grid_x<<","<<dijkstra_grid_y<<endl;
    float h1 = dijkstra_grid_resolution * d[roundDown(pos.x/dijkstra_grid_resolution)][roundDown(pos.y/dijkstra_grid_resolution)];
    float h2 = DubinCost(pos, target, vehicle.min_radius);
    return max(h1, h2);
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
    // http://docs.ros.org/diamondback/api/ompl/html/classompl_1_1base_1_1SE2StateSpace_1_1StateType.html
    
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
