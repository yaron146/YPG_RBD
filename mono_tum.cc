/**
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
const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);


enum class CSVState {//from csv to vector
    UnquotedField,
    QuotedField,
    QuotedQuote
};

vector<std::string> readCSVRow(const std::string& row) {
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

/// Read CSV file, Excel dialect. Accept "quoted fields ""with quotes"""
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
void saveMap(ORB_SLAM2::System &SLAM){
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
cv::Mat im;// global variables
cv::Mat check;
bool ret=false;//camera started working boolean
bool finish=false;
bool orbslamr = false;//orbslamrunning boolean
int timeStamps=0;
double mindist = 0.1;
double t;
bool save=false;
void scan(char **argv);//function for orbslam to process the picture
void picture();//function to take the picture
double angtodest(cv::Mat newcheck,vector<double> dest);
pair<double,double> getloc(cv::Mat newcheck);
double distance(pair<double,double> location,vector<double> dest);
pair<double,double> normalize(pair<double,double> v);
double dot_product(pair<double,double> p1, pair<double,double> p2);
double det(pair<double, double> p1, pair<double, double> p2);
double angleofnorm(pair<double, double> p1, pair<double, double> p2);
pair<double, double> rotateccw(pair<double, double> v,double angle);

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings" << endl;
        return 1;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
        //credit to the daniel's - there was cap.open function which didn't need to be there
    Tello tello;
    if (!tello.Bind())
    {
        return 0;
    }
    tello.SendCommand("streamon");
    while (!(tello.ReceiveResponse()));
	sleep(2);
    std::thread picturethread(picture);//multithreading
    while(!ret)//wiating for the camera to start
    {
	usleep(2000);
    }
    std::thread scanthread(scan,argv);//orbslam thread
    while(!orbslamr)
    {
	usleep(2000);
    }
    tello.SendCommand("takeoff");
    while (!(tello.ReceiveResponse()));
    	sleep(1);

	
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    while(im.empty())//waiting for the images to start loading
    {
        sleep(2);
    }
    for (;timeStamps<25;timeStamps++)
    {
       tello.SendCommand("cw 15");    
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    	tello.SendCommand("back 20");
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    	tello.SendCommand("forward 20");
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    	while(check.empty())
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
    save=true;
    while(!exists("/tmp/Result.csv"))//waiting for the algorithim to finish working
    {
    	sleep(1);
    }
    std::ifstream f("/tmp/Result.csv", std::ifstream::binary);
    vector<double> des=(readCSV(f))[0];
    vector <double> dest(2);
    dest[0]=des[0];
    dest[1]=des[2];
    cout << "finished karen csv\n";
    cout << "mindist: " << mindist << " distance: " << distance(getloc(check),dest) << "\n";
    while(distance(getloc(check),dest)>mindist)
    {
    	cout << "mindist: " << mindist << " distance: " << distance(getloc(check),dest) << "\n";
    	if(check.empty())
    	{
    		tello.SendCommand("land");
    		while (!(tello.ReceiveResponse()));
    		sleep(1);
    	}
    	double ang = angtodest(check,dest);
    	while(ang>15||ang<-15)
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
	    	while(check.empty())
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
	    	ang=angtodest(check,dest);
    	}
    	cout << "angle is : " << ang << "\n";
    	string go="";
    	if(ang<0)
    	{
    		ang=-1* ang;
    		go+="ccw " + to_string(ang);
    	}
    	else
    	{
    		go+="cw " + to_string(ang);
    	}
    	tello.SendCommand(go);
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    	tello.SendCommand("forward 50");
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    }
    cout << "finished getting out of the door \n";
    tello.SendCommand("forward 20");
    	while (!(tello.ReceiveResponse()));
    	sleep(1);
    tello.SendCommand("land");
    while (!(tello.ReceiveResponse()));
    sleep(1);
    cout << "land\n";
    finish=true;
    scanthread.join();
    picturethread.join();
    return 0;
}
    
void scan(char** argv)//function for orbslam to process the picture
{
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
	orbslamr=true;
	while(im.empty())
	{
	    sleep(2);
	}
	while(!finish)
	{
		if (im.empty()) {
		    cerr << "ERROR! blank frame grabbed\n";
		    break;
		}
		// show live and wait for a key with timeout long enough to show images
		// Pass the image to the SLAM system
		check = SLAM.TrackMonocular(im,t);
		if(save)
		{
			saveMap(SLAM);	
			save=false;
		}
	}
	// Stop all threads
	SLAM.Shutdown();
	// the camera will be deinitialized automatically in VideoCapture destructor
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
	
}
void picture()//function to take the picture
{
	cv::VideoCapture cap{TELLO_STREAM_URL,cv::CAP_FFMPEG};
	ret=true;//camera started working
	while(!finish)
	{
		cap>>im;//putting the image from the camera to the variable which saves the image
		
	}
	finish=true;
}
double angtodest(cv::Mat newcheck,vector<double> dest)
{
	cout << "\n\n\n***angtodest function :\nnewcheck:\n" << newcheck << "\n";
	cv::Mat R = newcheck(cv::Rect(0,0,3,3)).clone();
	cout << " R:\n" << R << "\n";
	cv::Mat T = newcheck(cv::Rect(3,0,1,3)).clone();
	cout << "T:\n" << T << "\n";
	cv::Mat pos = -(R.t())*(T);
	cout << "pos:\n" << pos << "\n";
	cv::Mat getZ(3,1,R.type(),cv::Scalar(0));
	getZ.at<float>(0,2)=1.0;
	cout << "getzmat:\n" << getZ << "\n";
	cv::Mat ang = (R.t())*(getZ);
	cout << "angmat:\n" << ang << "\n";
	pair<double,double> newpos((double)pos.at<float>(0,0),(double)pos.at<float>(0,2));
	cout << "newpos pair: " << newpos.first << " , " << newpos.second << "\n";
	pair<double,double> newang((double)ang.at<float>(0,0),(double)ang.at<float>(0,2));
	cout << "newang pair: " << newang.first << " , " << newang.second << "\n";
	pair<double,double> destdir(dest[0]-newpos.first,dest[1]-newpos.second);
	cout << "destdir pair: " << destdir.first << " , " << destdir.second << "\n";
	pair<double,double> ndestdir = normalize(destdir);
	cout << "ndestdir pait: " << ndestdir.first << " , " << ndestdir.second << "\n";
	pair<double,double> ncurdir = normalize(newang);
	cout << "ncurdir pair: " << ncurdir.first << " , " << ncurdir.second << "\n";
	double angle = angleofnorm(ndestdir,ncurdir);
	cout << "angle in function : " << angle << "\n";
	if(angleofnorm(ndestdir,normalize(rotateccw(ncurdir,angle)))<angleofnorm(ndestdir,normalize(rotateccw(ncurdir,-angle))))
		return -angle;
	return angle;
	
}
pair<double,double> getloc(cv::Mat newcheck)
{
	cout << "\n\n\n***getloc function:\n";
	cv::Mat R = newcheck(cv::Rect(0,0,3,3)).clone();
	cout << " R:\n" << R << "\n";
	cv::Mat T = newcheck(cv::Rect(3,0,1,3)).clone();
	cout << "T:\n" << T << "\n";
	cv::Mat pos = -(R.t())*(T);
	cout << "pos:\n" << pos << "\n";
	pair<double,double> newpos((double)pos.at<float>(0,0),(double)pos.at<float>(0,2));
	return newpos;
}

double distance(pair<double,double> location,vector<double> dest)
{
	return sqrt((dest[0]-location.first)*(dest[0]-location.first)+(dest[1]-location.second)*(dest[1]-location.second));
}

pair<double,double> normalize(pair<double,double> v)
{
	double norm = sqrt(v.first*v.first+v.second*v.second);
	pair<double,double> result(v.first/norm,v.second/norm);
	return result;
}
double dot_product(pair<double,double> p1, pair<double,double> p2){

    return p1.first*p2.first + p1.second*p2.second;
}

double det(pair<double, double> p1, pair<double, double> p2){

    return p1.first*p2.second - p1.second*p2.first;
}

double angleofnorm(pair<double, double> p1, pair<double, double> p2)
{
    double thing = dot_product(p1, p2);

    //cout << thing << "\t point: " << p1.first << "," <<p1.second << " and " << p2.first << "," <<p2.second;


    if(thing >= 1) return 0;
    if(thing <= -1) return 180;

    else return acos(thing) * 180 / M_PI;
}
pair<double, double> rotateccw(pair<double, double> v,double angle)
{
	pair<double,double> newv(v.first*cos(angle*PI/180)-v.second*sin(angle*PI/180),v.first*sin(angle*PI/180)+v.second*cos(angle*PI/180));
	return newv;
}

