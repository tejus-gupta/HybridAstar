#include <iostream>
#include <ompl/base/ScopedState.h>
#include <boost/program_options.hpp>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
 
using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;

int main()
{
 //    auto space( std::make_shared<ob::SE3StateSpace >( ));

 //    ob::RealVectorBounds bounds(3);
 //    bounds.setLow(-1);
 //    bounds.setHigh(1);
 //    space->setBounds(bounds);

 //    ompl::geometric::SimpleSetup ss(space);

	// ob::ScopedState<> start(space);
 //    start.random();

 //    ob::ScopedState<> goal(space);
 //    goal.random();

 //    ss.setStartAndGoalStates(start, goal);
 //    ob::PlannerStatus solved = ss.solve(1.0);

 //    if (solved)
 //    {
 //        std::cout << "Found solution:" << std::endl;

 //        // print the path to screen
 //        ss.simplifySolution();
 //        ss.getSolutionPath().print(std::cout);
 //    }
	ob::StateSpacePtr space(new ompl::base::SE2StateSpace());
	// ob::ScopedState<> start(space), goal(space);
	ob::State *start = space->allocState();
	ob::State *goal  = space->allocState();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(18);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);
	auto *s = start->as<ob::SE2StateSpace::StateType>();
	auto *t = goal->as<ob::SE2StateSpace::StateType>();
    
    s->setX(1);
    s->setY(1);
    s->setYaw(3.14/2);

    double x1=s->getX(), y1=s->getY() ,theta1=s->getYaw();
    double x2=t->getX(), y2=t->getY() ,theta2=t->getYaw();
    cout<<x1<<" "<<y1<<" "<<theta1<<" "<<x2<<" "<<y2<<" "<<theta2<<endl;

    ob::DubinsStateSpace DP(1.0,true);
    double dp_dist=DP.distance(start,goal);
    cout<<"Dubins Distance : "<<dp_dist<<endl;

    ob::ReedsSheppStateSpace  RS(1.0);
    double rs_dist=RS.distance(start,goal);
    cout<<"ReedsShepp Distance : "<<rs_dist<<endl;

    return 0;
}

