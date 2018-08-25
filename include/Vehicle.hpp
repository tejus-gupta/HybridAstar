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

	 int BOT_L=50;
	 int BOT_W=20;
	 int BOT_MAX_ALPHA=30;

	vector<State> nextStates(State*);
};
#endif