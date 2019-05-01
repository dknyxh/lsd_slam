/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>
#include <stdio.h>

#include "IOWrapper/ROS/ROSOutput3DWrapper.h"
#include "IOWrapper/ROS/rosReconfigure.h"

#include "util/Undistorter.h"
#include <ros/package.h>

#include "opencv2/opencv.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "DataStructures/FramePoseStruct.h"
#include <Eigen/Core>

std::vector<double> calculateRotation(const Eigen::Matrix<double,3,4> & R) {

  double w=0, x=0, y=0, z=0;
  double trace = R(0,0) + R(1,1) + R(2,2); // I removed + 1.0f; see discussion with Ethan
  if( trace > 0 ) {// I changed M_EPSILON to 0
    double s = 0.5f / sqrtf(trace+ 1.0f);
    w = 0.25f / s;
    x = ( R(2,1) - R(1,2) ) * s;
    y = ( R(0,2) - R(2,0) ) * s;
    z = ( R(1,0) - R(0,1) ) * s;
  } else {
    if ( R(0,0) > R(1,1) && R(0,0) > R(2,2) ) {
      double s = 2.0f * sqrtf( 1.0f + R(0,0) - R(1,1) - R(2,2));
      w = (R(2,1) - R(1,2) ) / s;
      x = 0.25f * s;
      y = (R(0,1) + R(1,0) ) / s;
      z = (R(0,2) + R(2,0) ) / s;
    } else if (R(1,1) > R(2,2)) {
      double s = 2.0f * sqrtf( 1.0f + R(1,1) - R(0,0) - R(2,2));
      w = (R(0,2) - R(2,0) ) / s;
      x = (R(0,1) + R(1,0) ) / s;
      y = 0.25f * s;
      z = (R(1,2) + R(2,1) ) / s;
    } else {
      double s = 2.0f * sqrtf( 1.0f + R(2,2) - R(0,0) - R(1,1) );
      w = (R(1,0) - R(0,1) ) / s;
      x = (R(0,2) + R(2,0) ) / s;
      y = (R(1,2) + R(2,1) ) / s;
      z = 0.25f * s;
    }
  }
  std::vector<double> return_vec = {R(0,3), R(1,3), R(2,3), x, y, z, w};
  return return_vec;
}


std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}
std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}
std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}
int getdir (std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL) {
    	std::string name = std::string(dirp->d_name);

    	if(name != "." && name != "..")
    		files.push_back(name);
    }
    closedir(dp);


    std::sort(files.begin(), files.end());

    if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++)
	{
		if(files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

    return files.size();
}

int getFile (std::string source, std::vector<std::string> &files)
{
	std::ifstream f(source.c_str());

	if(f.good() && f.is_open())
	{
		while(!f.eof())
		{
			std::string l;
			std::getline(f,l);

			l = trim(l);

			if(l == "" || l[0] == '#')
				continue;

			files.push_back(l);
		}

		f.close();

		size_t sp = source.find_last_of('/');
		std::string prefix;
		if(sp == std::string::npos)
			prefix = "";
		else
			prefix = source.substr(0,sp);

		for(unsigned int i=0;i<files.size();i++)
		{
			if(files[i].at(0) != '/')
				files[i] = prefix + "/" + files[i];
		}

		return (int)files.size();
	}
	else
	{
		f.close();
		return -1;
	}

}


using namespace lsd_slam;
int main( int argc, char** argv )
{
	ros::init(argc, argv, "LSD_SLAM");

	dynamic_reconfigure::Server<lsd_slam_core::LSDParamsConfig> srv(ros::NodeHandle("~"));
	srv.setCallback(dynConfCb);

	dynamic_reconfigure::Server<lsd_slam_core::LSDDebugParamsConfig> srvDebug(ros::NodeHandle("~Debug"));
	srvDebug.setCallback(dynConfCbDebug);

	packagePath = ros::package::getPath("lsd_slam_core")+"/";



	// get camera calibration in form of an undistorter object.
	// if no undistortion is required, the undistorter will just pass images through.
	std::string calibFile;
	Undistorter* undistorter = 0;
	if(ros::param::get("~calib", calibFile))
	{
		 undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
		 ros::param::del("~calib");
	}

	if(undistorter == 0)
	{
		printf("need camera calibration file! (set using _calib:=FILE)\n");
		exit(0);
	}

	int w = undistorter->getOutputWidth();
	int h = undistorter->getOutputHeight();

	int w_inp = undistorter->getInputWidth();
	int h_inp = undistorter->getInputHeight();

	float fx = undistorter->getK().at<double>(0, 0);
	float fy = undistorter->getK().at<double>(1, 1);
	float cx = undistorter->getK().at<double>(2, 0);
	float cy = undistorter->getK().at<double>(2, 1);
	Sophus::Matrix3f K;
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;


	// make output wrapper. just set to zero if no output is required.
	Output3DWrapper* outputWrapper = new ROSOutput3DWrapper(w,h);


	// make slam system
	SlamSystem* system = new SlamSystem(w, h, K, doSlam);
	system->setVisualization(outputWrapper);



	// open image files: first try to open as file.
	std::string source;
	std::vector<std::string> files;
	if(!ros::param::get("~files", source))
	{
		printf("need source files! (set using _files:=FOLDER)\n");
		exit(0);
	}
	ros::param::del("~files");

	if(getdir(source, files) >= 0)
	{
		printf("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
	}
	else if(getFile(source, files) >= 0)
	{
		printf("found %d image files in file %s!\n", (int)files.size(), source.c_str());
	}
	else
	{
		printf("could not load file list! wrong path / file?\n");
	}

	// open mask files: first try to open as file
	std::string mask_source;
	std::vector<std::string> mask_files;
	if(!ros::param::get("~masks", mask_source))
	{
		printf("need mask files! (set using _mask:=FOLDER)\n");
		exit(0);
	}
	ros::param::del("~masks");

	if(getdir(mask_source, mask_files) >= 0)
	{
		printf("found %d mask files in folder %s!\n", (int)mask_files.size(), mask_source.c_str());
	}
	else if(getFile(mask_source, mask_files) >= 0)
	{
		printf("found %d mask files in file %s!\n", (int)mask_files.size(), mask_source.c_str());
	}
	else
	{
		printf("could not load file list! wrong path / file?\n");
	}



	// get HZ
	double hz = 0;
	if(!ros::param::get("~hz", hz))
		hz = 0;
	ros::param::del("~hz");



	cv::Mat image = cv::Mat(h,w,CV_8U);
	cv::Mat mask = cv::Mat(h,w,CV_8U); // chenfeng
	int runningIDX=0;
	float fakeTimeStamp = 0;

	ros::Rate r(hz);

	for(unsigned int i=0;i<files.size();i++)
	{
		cv::Mat imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);
		cv::Mat maskDist = cv::imread(mask_files[i], CV_LOAD_IMAGE_GRAYSCALE);


		if(imageDist.rows != h_inp || imageDist.cols != w_inp ||
			maskDist.rows != h_inp ||  maskDist.cols != w_inp)
		{
			if(imageDist.rows * imageDist.cols == 0)
				printf("failed to load image %s! skipping.\n", files[i].c_str());
			else
				printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
						files[i].c_str(),
						w,h,imageDist.cols, imageDist.rows);

			if(maskDist.rows * maskDist.cols == 0)
				printf("failed to load mask %s! skipping.\n", mask_files[i].c_str());
			else
				printf("mask %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
						mask_files[i].c_str(),
						w,h,maskDist.cols, maskDist.rows);
			continue;
		}
		assert(imageDist.type() == CV_8U);
		assert(maskDist.type() == CV_8U);

		undistorter->undistort(imageDist, image);
		undistorter->undistort(maskDist, mask);

		assert(image.type() == CV_8U);
		assert(mask.type() == CV_8U);

		if(runningIDX == 0){
			system->randomInit(image.data, mask.data, fakeTimeStamp, runningIDX);
			// TODO-------- should mask do the same thing?
		}
		else{
			//Here we need to use another function
			system->trackFrame(image.data, mask.data, runningIDX ,hz == 0,fakeTimeStamp);
			// system->trackFrame(image.data, runningIDX ,hz == 0,fakeTimeStamp);
		}
		runningIDX++;
		fakeTimeStamp+=0.03;

		if(hz != 0)
			r.sleep();

		if(fullResetRequested)
		{

			printf("FULL RESET!\n");
			delete system;

			system = new SlamSystem(w, h, K, doSlam);
			system->setVisualization(outputWrapper);

			fullResetRequested = false;
			runningIDX = 0;
		}

		ros::spinOnce();

		if(!ros::ok())
			break;
	}


	system->finalize();

// Chenfeng: Below is for outputing the poses ---------------------
	auto poses = system->getAllPoses();
	int N = poses.size();
    int R = 3, C = 4;
    

    // std::string posePath = "./src/lsd_slam/scripts/data/lsd_pose.dat";
    // std::cout << "Writing transforms to:" << posePath << "\n";

    // std::ofstream out;
    // out.open(posePath.c_str(), std::ios::out | std::ios::binary);
    // out.write((char*) &N, sizeof(int));
    // out.write((char*) &R, sizeof(int));
    // out.write((char*) &C, sizeof(int));

    std::string trajectory_path;
    if(!ros::param::get("~trajectory", trajectory_path)){
    	trajectory_path = "./trajectory.txt" ;
    }
    ros::param::del("~trajectory");

    FILE * fp = fopen(trajectory_path.c_str(), "w");

    assert(N == files.size());
    int k = 0;
    for (FramePoseStruct* pose: poses) {
        Sim3 T = pose->getCamToWorld();
        auto mat = T.matrix3x4();

        std::vector<double> q = calculateRotation(mat);

        auto location = files[k].rfind("/");
        std::string timestamp = files[k].substr(location+1, files[k].size());
        timestamp = timestamp.substr(0, timestamp.size() - 4);

        fprintf(fp, "%s ", timestamp.c_str());
        for (int i=0; i < q.size(); i++){
          fprintf(fp, "%f ", q[i]);
		}
		fprintf(fp, "\n");

		k++;
        // out.write()
        // for (int i=0; i<3; ++i) {
        //     for (int j=0; j<4; ++j) {
        //         double val = mat(i,j);
        //         out.write((char*) &val, sizeof(double));
        //     }
        // }
    }
	fclose(fp);
//----------------------------------------------------------------

	delete system;
	delete undistorter;
	delete outputWrapper;
	return 0;
}

