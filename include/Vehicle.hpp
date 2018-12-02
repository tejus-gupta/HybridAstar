#ifndef VEH_HPP
#define VEH_HPP

#include <stdio.h>
#include <iostream>
#include "bits/stdc++.h"
#include "State.hpp"
//#include "ros/ros.h"

#define PI 3.14159265359

using namespace std;

class Vehicle{

public:

	float BOT_L=2.5;
	float BOT_W=1.5;
	float BOT_MAX_ALPHA=30;
	double min_radius = abs(BOT_L/tan(BOT_MAX_ALPHA*PI/180));
	vector<State> nextStates(State*,float);
};
#endif