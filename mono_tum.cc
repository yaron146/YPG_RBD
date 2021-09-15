/**

credits:

yanir, tom, amit - helping setting up the ctello libary on the virtual machine
daniel's and yahav - finding a line in code which can't be run which solved a bug
kareen - providing a function which converts csv file to vector
eliron and shahaf - helped a bit with the exit algorithm



* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#include <math.h>
#include <filesystem>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>	
#include<opencv2/core/core.hpp>
#include "ctello.h"
#include<System.h>
#include <Converter.h>	
#include <optional>
#include <thread>
#include <string.h>
#define PI 3.14159265
using std::filesystem::exists;
using namespace std;
using ctello::Tello;
const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};// the url of the tello
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);


enum class CSVState {//from csv to vector - karen function
    UnquotedField,
    QuotedField,
    QuotedQuote
};

cv::Mat im;// global variables
cv::Mat check;
bool ret=false;//camera started working boolean
bool finish=false;
bool orbslamr = false;//orbslamrunning boolean
int timeStamps=0;
double mindist = 0.1;
double t;
bool save=false;//a boolean whichc states to save or not to save the map
void scan(char **argv);//function for orbslam to process the picture
void picture();//function to take the picture
double angtodest(cv::Mat newcheck,vector<double> dest);//a function which returns the angle which the drone need to rotate in order to look at the exit
pair<double,double> getloc(cv::Mat newcheck);//getting the location of the drone
double distance(pair<double,double> location,vector<double> dest);//distance function
pair<double,double> normalize(pair<double,double> v);//normalizing the vector
double dot_product(pair<double,double> p1, pair<double,double> p2);//dot product of vectors
double det(pair<double, double> p1, pair<double, double> p2);//determenant of a matrix
double angleofnorm(pair<double, double> p1, pair<double, double> p2);//the angle of the normalized vectors
pair<double, double> rotateccw(pair<double, double> v,double angle);//rotating


vector<std::string> readCSVRow(const std::string& row) { //[kareen's]
    CSVState state = CSVState::UnquotedField;
    vector<std::string> fields{ "" };
    size_t i = 0; // index of the current field
    for (char c : row) {
        switch (state) {
        case CSVState::UnquotedField:
            switch (c) {
            case ',': // end of field
                fields.push_back(""); i++;
                break;
            case '"': state = CSVState::QuotedField;
                break;
            default:  fields[i].push_back(c);
                break;
            }
            break;
        case CSVState::QuotedField:
            switch (c) {
            case '"': state = CSVState::QuotedQuote;
                break;
            default:  fields[i].push_back(c);
                break;
            }
            break;
        case CSVState::QuotedQuote:
            switch (c) {
            case ',': // , after closing quote
                fields.push_back(""); i++;
                state = CSVState::UnquotedField;
                break;
            case '"': // "" -> "
                fields[i].push_back('"');
                state = CSVState::QuotedField;
                break;
            default:  // end of quote
                state = CSVState::UnquotedField;
                break;
            }
            break;
        }
    }
    return fields;
}

/// Read CSV file, Excel dialect. Accept "quoted fields ""with quotes""" [kareen's]
vector<vector<double>> readCSV(std::istream& in) {
    vector<vector<std::string>> table;
    std::string row;
    while (!in.eof()) {
        std::getline(in, row);
        if (in.bad() || in.fail()) {
            break;
        }
        auto fields = readCSVRow(row);
        table.push_back(fields);
    }
    // convert from an vector of strings to vector of doubles
    vector<vector<double>> points;
    for (auto& row : table)
    {
        double x = std::stod(row[0]);
        double y = std::stod(row[1]);
        double z = std::stod(row[2]);
        vector<double> values = { x, y, z };
        points.push_back(values);
        //double dist = sqrt(x * x + y * y + z * z);
        //std::cout << x << " " << y << " " << z << " " << dist << std::endl;
    }
    return points;
}
void saveMap(ORB_SLAM2::System &SLAM){ // [the course's moodle]
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/pointData.csv");
    for(auto p : mapPoints) {
        if (p != NULL)
        {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
        }
    }
    pointData.close();
}
//

int main(int argc, char **argv)
{ // [ours]
    if(argc != 3)//a check if all things needed got sent
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings" << endl;
        return 1;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
        //credit to the daniel's - there was cap.open function which didn't need to be there
    Tello tello;
    if (!tello.Bind())//binding to the drone
    {
        return 0;
    }
    tello.SendCommand("streamon");//starting the camera
    while (!(tello.ReceiveResponse()));
	sleep(2);
    std::thread picturethread(picture);//multithreading - thread which inserts the picture from the camera into the matrix
    while(!ret)//wiating for the camera to start
    {
	usleep(2000);
    }
    std::thread scanthread(scan,argv);//orbslam thread
    while(!orbslamr)//waiting for orbslam to finish setting up
    {
	usleep(2000);
    }
    tello.SendCommand("takeoff");//taking of the drone
    while (!(tello.ReceiveResponse()));
    	sleep(1);

	
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    while(im.empty())//waiting for the images to start loading
    {
        sleep(2);
    }
    for (;timeStamps<25;timeStamps++)//a loop which in it the drone runs a full 360 rotation in order to get map of the room in order to send it to the algorithim(written in python) 
    {
       tello.SendCommand("cw 15");    //rotating
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    	tello.SendCommand("back 20");//going back and forward in order to get a good view of the area
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    	tello.SendCommand("forward 20");
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    	while(check.empty())//in case orbslam lost track, the drone goes back to it's position before rotation, then tries again and again until it regains track
    	{
    		tello.SendCommand("ccw 15");   
    		while (!(tello.ReceiveResponse()));
    		sleep(1);
    		tello.SendCommand("back 20");
    		while (!(tello.ReceiveResponse()));
    		sleep(1);
    		tello.SendCommand("forward 20");
    		while (!(tello.ReceiveResponse()));
    		sleep(1);
    		for(int i =0; i<3; i++)
    		{
    			tello.SendCommand("cw 5");    
    			while (!(tello.ReceiveResponse()));
    			sleep(1);
    			tello.SendCommand("back 20");
    			while (!(tello.ReceiveResponse()));
    			sleep(1);
    			tello.SendCommand("forward 20");
    			while (!(tello.ReceiveResponse()));
    			sleep(1);
    		}
    	}
    }
    save=true;// letting the scan thread to save map
    while(!exists("/tmp/Result.csv"))//waiting for the algorithim to finish working
    {
    	sleep(1);
    }
    std::ifstream f("/tmp/Result.csv", std::ifstream::binary);//getting the result from the algorithim
    vector<double> des=(readCSV(f))[0];//converting from csv to vector
    vector <double> dest(2);
    dest[0]=des[0];
    dest[1]=des[2];//getting the correct cordinates
    while(distance(getloc(check),dest)>mindist)//getting out of the room process
    {
    	if(check.empty())//checking if orbslam lost track
    	{
    		tello.SendCommand("land");
    		while (!(tello.ReceiveResponse()));
    		sleep(1);
    	}
    	double ang = angtodest(check,dest);//getting the angle which the drone need to rotate in order to face the exit
    	while(ang>15||ang<-15)//rotating slowly in order not to lose track, and while rotating going forward and backwards in order to get a better view of the area
    	{
    		if(ang<0)
	    	{
	    		tello.SendCommand("ccw 15");    
		    	while (!(tello.ReceiveResponse()));
		    	sleep(1);
	    	}
	    	else
	    	{
	    		tello.SendCommand("cw 15");    
		    	while (!(tello.ReceiveResponse()));
		    	sleep(1);
	    	}
	    	tello.SendCommand("back 20");
	    	while (!(tello.ReceiveResponse()));
	    	sleep(1);
	    	tello.SendCommand("forward 20");
	    	while (!(tello.ReceiveResponse()));
	    	sleep(1);
	    	while(check.empty())//checking if orbslam lost track then waiting until it regains track
	    	{
	    		if(ang<0)
	    		{
		    		tello.SendCommand("cw 15");    
		    		while (!(tello.ReceiveResponse()));
		    		sleep(1);
		    		for(int i =0; i<3; i++)
		    		{
		    			tello.SendCommand("ccw 5");    
		    			while (!(tello.ReceiveResponse()));
		    			sleep(1);
		    			tello.SendCommand("back 20");
		    			while (!(tello.ReceiveResponse()));
		    			sleep(1);
		    			tello.SendCommand("forward 20");
		    			while (!(tello.ReceiveResponse()));
		    			sleep(1);
		    		}
		    	}
		    	else
		    	{
		    		tello.SendCommand("ccw 15");    
		    		while (!(tello.ReceiveResponse()));
		    		sleep(1);
		    		for(int i =0; i<3; i++)
		    		{
		    			tello.SendCommand("cw 5");    
		    			while (!(tello.ReceiveResponse()));
		    			sleep(1);
		    			tello.SendCommand("back 20");
		    			while (!(tello.ReceiveResponse()));
		    			sleep(1);
		    			tello.SendCommand("forward 20");
		    			while (!(tello.ReceiveResponse()));
		    			sleep(1);
		    		}
		    	}
	    	}
	    	ang=angtodest(check,dest);//claculating the angle again in order to check if we need to rotate another time before exiting the room
    	}
    	string go="";//a string which we use in order to send the command to the drone
    	if(ang<0)//checking if the angle is negative which means we need to go counter clock wise
    	{
    		ang=-1* ang;
    		go+="ccw " + to_string(ang);
    	}
    	else
    	{
    		go+="cw " + to_string(ang);
    	}
    	tello.SendCommand(go);//sending the command of rotating
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    	tello.SendCommand("forward 50");//going to the exit
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    }
    tello.SendCommand("forward 20");//going forward a little bit in order to be sure that the drone is out of the room
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    tello.SendCommand("land");//landing the drone
    while (!(tello.ReceiveResponse()));
    sleep(1);
    finish=true;// a boolean which states that the drone finished it's process
    scanthread.join();
    picturethread.join();//waiting for the other threads
    return 0;
}
    
void scan(char** argv)//function for orbslam to process the picture, its recieves the arguments which are nessecary for setting up ORBSLAM, and it doesnt return anything.
{ //[ours]
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);//setting up the orbslam
	orbslamr=true;//orbslam is ready
	while(im.empty())//checking if getting images from the drone, and waiting if needed
	{
	    sleep(2);
	}
	while(!finish)//running the orbslam in order to analyze the pictures 
	{
		if (im.empty()) {//in case of an error
		    cerr << "ERROR! blank frame grabbed\n";
		    break;
		}
		// show live and wait for a key with timeout long enough to show images
		// Pass the image to the SLAM system
		check = SLAM.TrackMonocular(im,t);//orblsam analyzing the pictures
		if(save)//if it's time to save the map
		{
			saveMap(SLAM);	
			save=false;//saving only once
		}
	}
	// Stop all threads
	SLAM.Shutdown();
	// the camera will be deinitialized automatically in VideoCapture destructor
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
	
}
void picture()//function to take the picture from the drone, it doesnt receive anything but uses some of the global variables, doesnt return anything.  
{ //[ours]
	cv::VideoCapture cap{TELLO_STREAM_URL,cv::CAP_FFMPEG};//openning the drone camera
	ret=true;//camera started working
	while(!finish)
	{
		cap>>im;//putting the image from the camera to the variable which saves the image
		
	}
	finish=true;
}
double angtodest(cv::Mat newcheck,vector<double> dest)//function which retunrs the angle that the drone need to rotate in order to face the exit [ours]
{
	cv::Mat R = newcheck(cv::Rect(0,0,3,3)).clone();
	cv::Mat T = newcheck(cv::Rect(3,0,1,3)).clone();//getting to vectors from the position of the drone
	cv::Mat pos = -(R.t())*(T);
	cv::Mat getZ(3,1,R.type(),cv::Scalar(0));//a vector of {0,0,1}
	getZ.at<float>(0,2)=1.0;
	cv::Mat ang = (R.t())*(getZ);//calculating the angle matrix which tells us which angle we need to rotate
	pair<double,double> newpos((double)pos.at<float>(0,0),(double)pos.at<float>(0,2));//the position
	pair<double,double> newang((double)ang.at<float>(0,0),(double)ang.at<float>(0,2));//the angle
	pair<double,double> destdir(dest[0]-newpos.first,dest[1]-newpos.second);//destination vector
	pair<double,double> ndestdir = normalize(destdir);//normalized dest vector
	pair<double,double> ncurdir = normalize(newang);
	double angle = angleofnorm(ndestdir,ncurdir);//getting the angle as a numbeer
	if(angleofnorm(ndestdir,normalize(rotateccw(ncurdir,angle)))<angleofnorm(ndestdir,normalize(rotateccw(ncurdir,-angle))))//checking if we need to return a negative angle
		return -angle;
	return angle;
	
}
pair<double,double> getloc(cv::Mat newcheck)//getting location of the drone [ours]
{
	cv::Mat R = newcheck(cv::Rect(0,0,3,3)).clone();
	cv::Mat T = newcheck(cv::Rect(3,0,1,3)).clone();//vectors of the position
	cv::Mat pos = -(R.t())*(T);
	pair<double,double> newpos((double)pos.at<float>(0,0),(double)pos.at<float>(0,2));//cordinates of the drone
	return newpos;
}

double distance(pair<double,double> location,vector<double> dest)//distance function [ours]
{
	return sqrt((dest[0]-location.first)*(dest[0]-location.first)+(dest[1]-location.second)*(dest[1]-location.second));
}

pair<double,double> normalize(pair<double,double> v)//normalizing vector [ours]
{
	double norm = sqrt(v.first*v.first+v.second*v.second);
	pair<double,double> result(v.first/norm,v.second/norm);
	return result;
}
double dot_product(pair<double,double> p1, pair<double,double> p2){//dot product of vectors [ours]

    return p1.first*p2.first + p1.second*p2.second;
}

double det(pair<double, double> p1, pair<double, double> p2){//determinant of matrix [ours]

    return p1.first*p2.second - p1.second*p2.first;
}

double angleofnorm(pair<double, double> p1, pair<double, double> p2)//a function which returns the angle from the normalized vectors [ours]
{
    double thing = dot_product(p1, p2);

    //cout << thing << "\t point: " << p1.first << "," <<p1.second << " and " << p2.first << "," <<p2.second;


    if(thing >= 1) return 0;
    if(thing <= -1) return 180;

    else return acos(thing) * 180 / M_PI;
}
pair<double, double> rotateccw(pair<double, double> v,double angle)//rotating counter clock wise [ours]
{
	pair<double,double> newv(v.first*cos(angle*PI/180)-v.second*sin(angle*PI/180),v.first*sin(angle*PI/180)+v.second*cos(angle*PI/180));
	return newv;
}

