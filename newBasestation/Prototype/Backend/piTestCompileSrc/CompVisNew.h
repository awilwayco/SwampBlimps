#pragma once

// ============================== INCLUDES ==============================
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>

#include "EnumUtil.h"

using namespace std;
using namespace cv;

// ============================== DEFINES ==============================
#define CAMERA_INDEX       1

#define CAMERA_WIDTH	320
#define CAMERA_HEIGHT	240

#define RECT_WIDTH		320
#define RECT_HEIGHT		240

#define F               420
#define BASELINE        0.062 //0.20341207

#define PRE_FILTER_SIZE	7
#define PRE_FILTER_CAP	2
#define UNIQUENESS_RATIO	5

#define LAMBDA			17.8
#define SIGMA			5.0

#define AVOID_AREA		6000
#define AVOID_DIST		70

#define DISP_WIDTH		160
#define DISP_HEIGHT		120

#define B_CORRECTION	Scalar(29,7,15)
#define B_MIN			Scalar(46,0,0)
#define B_MAX			Scalar(96,74,213)

#define ORANGE_G_CORRECTION    Scalar(0,45,0)
#define ORANGE_G_MIN           Scalar(10,0,0) //orange min changed to 10
#define ORANGE_G_MAX           Scalar(28,255,255) //orange min changed to 35


#define STEREO_CAL_PATH     "/home/corelab-laptop2/Documents/testCams/piTestCompileSrc/stereo_rectify_maps240p.xml"



// ============================== CLASS ========#define AVOID_AREA		6000======================
class CompVisNew {
    private:
        VideoCapture cap;

        // Stereo Calibration
        Mat Left_Stereo_Map1;
        Mat Left_Stereo_Map2;
        Mat Right_Stereo_Map1;
        Mat Right_Stereo_Map2;
        Mat Q;

        // Stereo matcher
        Ptr< StereoBM > stereo;
        Ptr< StereoBM > left_matcher;
        Ptr<ximgproc::DisparityWLSFilter> wls_filter;
        Ptr<StereoMatcher> right_matcher;

        // Identified objects
        std::vector<vector<float> > balloons;
        std::vector<vector<float> > goals;

        // Goal detection
        double pixelDensityL = 0.2;
        double pixelDensityR = 0.2;

    public:
        void init();
        void readCalibrationFiles();
        void update(autoState mode, goalType goalColor);
        void getFrames(Mat &imgL, Mat &imgR);
        bool getBall(float &X, float &Y, float &Z, float &area, Mat imgL, Mat imgR);
        void getGoal(float &X, float &Y, float &Z, float &area, float &angle, Mat imgL, Mat imgR);
        int getAvoidance(Mat imgL, Mat imgR);
        float get_avg_dist_FM(Mat imgL, Mat imgR, String index);
        float get_avg_dist_DM(Mat imgL, Mat imgR, String index);
        float get_avg_dist_CA(Mat imgL, Mat imgR, String index);
};


