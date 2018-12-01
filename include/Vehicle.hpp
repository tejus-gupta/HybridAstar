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

	float BOT_L=10;
	float BOT_W=10;
	float BOT_MAX_ALPHA=90;
	vector<State> nextStates(State*);
};
#endif