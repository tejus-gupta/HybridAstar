#include "../include/GUI.hpp"

GUI::GUI(int rows, int cols){
	this->rows = rows;
	this->cols = cols;
	display=Mat(cv::Size(rows, cols), CV_8UC3, Scalar(220,220,220));
}

void GUI::draw_obstacles(bool** obs_map, float res){
	for(int i=0; i<cols; i++)
		for(int j=0; j<rows; j++)
			if(obs_map[(int)(i/res)][(int)(j/res)])
				display.at<Vec3b>(i,j) = {128, 128, 128};
	
	return;
}

void GUI::draw_car(State state, Vehicle car,float map_resolution){
	RotatedRect rotRect = RotatedRect(Point2f(state.y*map_resolution, state.x*map_resolution), Size2f(car.BOT_W*map_resolution, car.BOT_L*map_resolution), 180-state.theta*180/M_PI);
	Point2f vert[4];
	rotRect.points(vert);
	for(int i=0;i<4;i++)
		line(display, vert[i], vert[(i+1)%4], Scalar(200, 0, 0));
	
	return;
}

void GUI::draw_tree(State state,State next)
{
	Point root,child;
	root.x=state.y;
	root.y=state.x;
	child.x=next.y; 
	child.y=next.x;
	cout<<root.y<<" "<<root.x<<" "<<child.y<<" "<<child.x<<endl;
	line(display, root, child, Scalar(0,0,255), 1);

	return;
}

void GUI::show(){
	imshow("Display", display);
	waitKey(0);
	return;
}

void GUI::show(int t){
	imshow("Display", display);
	waitKey(t);
	return;
}


