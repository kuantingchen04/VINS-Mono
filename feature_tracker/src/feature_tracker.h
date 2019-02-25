#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img,double _cur_time); //@kev main image process

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file); //@kev store at CameraPtr m_camera

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;  //@kev output corners of goodFeatureToTrack
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts; //@kev [cur_pts,forw_pts]->KLT, [cur_pts,forw_pts]->rejectF
    vector<cv::Point2f> prev_un_pts, cur_un_pts; // normalized 3d points (z=1) in cam-space
    vector<cv::Point2f> pts_velocity; //[cur_pts->forw_pts] speed?
    vector<int> ids; // [1,2,3,4..] @kev 
    vector<int> track_cnt; // [3,3,3,3..] @kev track counts for @kev
    map<int, cv::Point2f> cur_un_pts_map; //@kev { id:<x,y> }, used for velocity
    map<int, cv::Point2f> prev_un_pts_map; //@kev used for velocity
    camodocal::CameraPtr m_camera; //@kev can model & parameter stores
    double cur_time;
    double prev_time;

    static int n_id; //@kev current count id
};
