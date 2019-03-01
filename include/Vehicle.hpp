#ifndef VEH_HPP
#define VEH_HPP

#include <stdio.h>
#include <iostream>
#include "bits/stdc++.h"
#include "State.hpp"
//#include "ros/ros.h"

// #define M_PI 3.14159265359

using namespace std;

class Vehicle{

public:

	float BOT_L=2;
	float BOT_W=1;
	float BOT_MAX_ALPHA=30;
	double min_radius = 1 ;//abs(BOT_L/tan(BOT_MAX_ALPHA*M_PI/180));
	vector<State> nextStates(State*, float);
};
#endif