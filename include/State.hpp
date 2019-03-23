#ifndef STATE_HPP
#define STATE_HPP

#include <math.h>

class State{

public:
	float x;
    float y;
	float theta;

	float g;
	float h;
	State* parent;

	State(float X,float Y,float THETA)
	{
		x=X;
		y=Y;
		theta=THETA;
		parent=NULL;
		g = 0;
	}

	State()
	{
		x=0;
		y=0;
		theta=0;
		parent=NULL;
		g = 0;
	}
};

inline int toInt(float n)
{
    return floor(n + 0.5);
}

inline int roundDown(float n)
{
    return floor(n);
}

#endif

