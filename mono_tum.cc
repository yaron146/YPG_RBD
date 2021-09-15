

/**
credits:
yanir,tom,amit - helping setting up the ctello libary for running the drone from the pc through code
kareen -  function to convert from csv row to a vector which includes the destenation points
daniel,daniel - for solving a bug which stopped orbslam from running with ctello




**/
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


enum class CSVState {//from csv to vector, kareen's function
    UnquotedField,
    QuotedField,
    QuotedQuote
};

cv::Mat im;// global variables
bool ret=false;//camera started working boolean
bool finish=false;
bool orbslamr = false;//orbslamrunning boolean
int timeStamps=0;
double t;
bool save=false;
void scan(char **argv);//function for orbslam to process the picture
void picture();//function to take the picture

vector<std::string> readCSVRow(const std::string& row) { // [kareen's]
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
void saveMap(ORB_SLAM2::System &SLAM){// from the course's moodle
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
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings" << endl;
        return 1;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
        //credit to the daniel's - there was cap.open function which didn't need to be there
    Tello tello;//the drone
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
    tello.SendCommand("up 25");//taking off and going up a little bit to get a better picture
    while (!(tello.ReceiveResponse()));
    	sleep(1);

	
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    while(im.empty())//waiting for the images to start loading
    {
        sleep(2);
    }
    for (;timeStamps<25;timeStamps++)//a loop which in it the drone rotates 360 degrees and goes back and forward while doing it in order to map the room
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
    }
    save=true;//a global variable enabling the saving map function
    while(!exists("/tmp/Result.csv"))//waiting for the algorithim to finish working
    {
    	sleep(1);
    }
    std::ifstream f("/tmp/Result.csv", std::ifstream::binary);//reading the results from the algortihim which runs on python - communication happens through files
    vector<double> dest=(readCSV(f))[0];//getting the resuly into a vector
    double x=dest[0];
    double z=dest[2];//results into points
    double ang=atan2(z,x)*180/PI;//calculating the angle
    ang-=90;
    string go="";
    if(ang<0)//finding out if we need to go clock wise or counterclock wise
    {
    	ang=-1* ang;
    	go+="cw " + to_string(ang);
    }
    else
    {
    	
    	go+="ccw " + to_string(ang);
    }
    tello.SendCommand(go);//making the drone rotate to the exit
    while (!(tello.ReceiveResponse()));
    sleep(1);
    tello.SendCommand("forward 200");//making it go forward in order to get out of the room
    while (!(tello.ReceiveResponse()));
    sleep(1);
    tello.SendCommand("forward 200");
    while (!(tello.ReceiveResponse()));
    sleep(1);
    cout << " send commnad\n";
    tello.SendCommand("land");//landing the drone
    while (!(tello.ReceiveResponse()));
    	sleep(1);
    finish=true;//global variable which states that the run is finished
    scanthread.join();
    picturethread.join();//joining the threads
    return 0;
}
    
void scan(char** argv)//function for orbslam to process the picture
{ //[ours]
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);//setting up the slam
	orbslamr=true;//global variable which states that the slam mode is ready
	while(im.empty())//waiting until we get a picture from the drone
	{
	    sleep(2);
	}
	while(!finish)//running a loop and getting pictures and analyzing it using the orbslam
	{
		if (im.empty()) {//checking if the image is empty in case of an error
		    cerr << "ERROR! blank frame grabbed\n";
		    break;
		}
		// show live and wait for a key with timeout long enough to show images
		// Pass the image to the SLAM system
		SLAM.TrackMonocular(im,t);
		if(save)//checking if its time to save the map
		{
			saveMap(SLAM);	
			save=false;//saving the map only once
		}
	}
	// Stop all threads
	SLAM.Shutdown();
	// the camera will be deinitialized automatically in VideoCapture destructor
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
	
}
void picture()//function to take the picture
{ //[ours]
	cv::VideoCapture cap{TELLO_STREAM_URL,cv::CAP_FFMPEG}//opening the camera of the drone;
	ret=true;//camera started working
	while(!finish)
	{
		cap>>im;//putting the image from the camera to the variable which saves the image
		
	}
	finish=true;
}


