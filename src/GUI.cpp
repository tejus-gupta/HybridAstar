#include "../include/GUI.hpp"

GUI::GUI(int rows, int cols){
	this->rows = rows;
	this->cols = cols;
	display=Mat(cv::Size(rows, cols), CV_8UC3, Scalar(220,220,220));
}

void GUI::draw_obstacles(bool** obs_map){
	for(int i=0; i<cols; i++)
		for(int j=0; j<rows; j++)
			if(obs_map[i][j])
				display.at<Vec3b>(rows-j, i) = {128, 128, 128};
	
	return;
}

void GUI::draw_car(State state, Vehicle car){
	RotatedRect rotRect = RotatedRect(Point2f(state.x*10, state.y*10), Size2f(car.BOT_L*10, car.BOT_W*10), state.theta*5);
	Point2f vert[4];
	rotRect.points(vert);
	for(int i=0;i<4;i++)
		line(display, vert[i], vert[(i+1)%4], Scalar(200, 0, 0));
	
	return;
}

void GUI::show(){
	imshow("Display", display);
	waitKey(0);
	return;
}


