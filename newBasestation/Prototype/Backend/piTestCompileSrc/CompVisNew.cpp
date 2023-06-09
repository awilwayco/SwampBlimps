// ============================== INCLUDES ==============================
#include <libv4l2.h>
#include <iostream>
#include "CompVisNew.h"
#include <vector>

using namespace std;

// ============================== CLASS ==============================
void CompVisNew::init()
{
  // Init Camera
  cap.open(CAMERA_INDEX, CAP_V4L);
  if (!cap.isOpened()) {
		CV_Assert("CamL open failed");
	}

  cap.set(CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
	cap.set(CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

  // Init stereo matcher
  left_matcher = StereoBM::create(16, 13); //Num disp, block size
	left_matcher->setPreFilterType(1);
	left_matcher->setPreFilterSize(PRE_FILTER_SIZE);
	left_matcher->setPreFilterCap(PRE_FILTER_CAP);
	left_matcher->setUniquenessRatio(UNIQUENESS_RATIO);

	wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
	right_matcher = ximgproc::createRightMatcher(left_matcher);

	wls_filter->setLambda(LAMBDA);
	wls_filter->setSigmaColor(SIGMA);
}

void CompVisNew::readCalibrationFiles()
{
  cout << "Reading Stereo Camera Parameters" << endl;
	
  FileStorage cv_file2 = FileStorage(STEREO_CAL_PATH, FileStorage::READ);
    if(!cv_file2.isOpened()){
        cout << "Read failed!" << endl;
        return;
    }
	
  cv_file2["Left_Stereo_Map_x"] >> Left_Stereo_Map1;
	cv_file2["Left_Stereo_Map_y"] >> Left_Stereo_Map2;
	cv_file2["Right_Stereo_Map_x"] >> Right_Stereo_Map1;
	cv_file2["Right_Stereo_Map_y"] >> Right_Stereo_Map2;
  cv_file2["Q"] >> Q;


	cv_file2.release();
	cout << "Read Complete" << endl;
}

void CompVisNew::update(autoState mode, goalType goalColor)
{
  // Get Frames
  Mat left, right;
  getFrames(left, right);

  // Coord Variables
  float Ballx, Bally, Ballz = 0;
  float Goalx, Goaly, Goalz = 0;
  float ballArea = 0;
  float goalArea = 0;
  float goalAngle = 0;

  // Object Avoidance
  if (mode == searching || mode == goalSearch) {
    // 
    
  }

  // Ball Detection
  if (mode == searching || mode == approach || mode == catching) {
    getBall(Ballx, Bally, Ballz, ballArea, left, right);

    std::vector<float> balloon;
    balloon.push_back(Ballx);
    balloon.push_back(Bally);
    balloon.push_back(Ballz);
    balloon.push_back(ballArea);
    balloons.push_back(balloon);

  // Goal Detection
  } else if (mode == goalSearch || mode == approachGoal || mode == scoringStart) {
    getGoal(Goalx, Goaly, Goalz, goalArea, goalAngle, left, right);

    std::vector<float> goal;
    goal.push_back(Goalx);
    goal.push_back(Goaly);
    goal.push_back(Goalz);
    goal.push_back(goalArea);
    goal.push_back(goalAngle);
    goals.push_back(goal);

  }
}

void CompVisNew::getFrames(Mat &imgL, Mat &imgR)
{
  // Get Frame
  Mat frame2;
  cap >> frame2;

  // Split Frames
  Rect left_roi(0, 0, frame2.cols/2, frame2.rows);
  Rect right_roi (frame2.cols/2, 0, frame2.cols/2, frame2.rows);

  Mat crop_left(frame2, left_roi);
  Mat crop_right (frame2, right_roi);

  crop_left.copyTo(imgL);
  crop_right.copyTo(imgR);
}

bool CompVisNew::getBall(float &X, float &Y, float &Z, float &area, Mat imgL, Mat imgR)
{
  // Initialize matrix for rectified stereo images
  Mat Left_nice, Right_nice;

  // Applying stereo image rectification on the left image
  remap(imgL,
        Left_nice,
        Left_Stereo_Map1,
        Left_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  // Applying stereo image rectification on the right image
  remap(imgR,
        Right_nice,
        Right_Stereo_Map1,
        Right_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  // Applying blur to reduce noise
  //GaussianBlur(Left_nice, Left_nice, Size(5, 5), 0);
  //GaussianBlur(Right_nice, Right_nice, Size(5, 5), 0);

  // Apply sharpen with laplacian filter
  //Mat imgSharp_L, imgSharp_R;
  //Laplacian(imgL, imgL, CV_16S, 3);
  //Laplacian(imgR, imgR, CV_16S, 3);

  //Mat imgSharp;
  //convertScaleAbs(imgL, imgL);
  //convertScaleAbs(imgR, imgR);

  //Apply correction
  Mat ball_CL, ball_CR;
  Mat balloonCorrected_L, balloonCorrected_R;
  Mat balloonCorrect_L = Mat::zeros(Left_nice.size(), Left_nice.type());
  Mat balloonCorrect_R = Mat::zeros(Right_nice.size(), Right_nice.type());
  balloonCorrect_L.setTo(B_CORRECTION);
  balloonCorrect_R.setTo(B_CORRECTION);

  add(Left_nice, balloonCorrect_L, ball_CL);
  add(Right_nice, balloonCorrect_R, ball_CR);

  //Apply HSV
  Mat left_HSV, right_HSV;
  cvtColor(ball_CL, left_HSV, cv::COLOR_BGR2HSV);
  cvtColor(ball_CR, right_HSV, cv::COLOR_BGR2HSV);

  //Isolate Ball
  Mat bMask_L, bMask_R;
  inRange(left_HSV, B_MIN, B_MAX, bMask_L);
  inRange(right_HSV, B_MIN, B_MAX, bMask_R);

  //Noise reduction
  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
  Mat bMask_L_cleaned, bMask_R_cleaned;
  morphologyEx(bMask_L, bMask_L_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_L_cleaned, bMask_L_cleaned, MORPH_OPEN, kernel);

  morphologyEx(bMask_R, bMask_R_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_R_cleaned, bMask_R_cleaned, MORPH_OPEN, kernel);

  //DEBUG: see mask
  //namedWindow("bMask_L");
  //imshow("bMask_L", bMask_L_cleaned);

  //namedWindow("bMask_R");
  //imshow("bMask_R", bMask_R_cleaned);

  //Find Largest Contour (Largest Ball)
  vector<vector<Point> > contoursL;
  findContours(bMask_L_cleaned, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<Point> largestContour_L;
  float largestArea_L = 0;
  int index_L = 0;

  for (unsigned int i = 0; i < contoursL.size(); i++) {
      double area = contourArea(contoursL[i]);
      if (area > largestArea_L) {
          largestArea_L = area;
          largestContour_L = contoursL[i];
          index_L = 0;
      }
  }

  vector<vector<Point> > contoursR;
  findContours(bMask_R_cleaned, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  vector<Point> largestContour_R;
  float largestArea_R = 0;
  int index_R = 0;

  for (unsigned int i = 0; i < contoursR.size(); i++) {
      double area = contourArea(contoursR[i]);
      if (area > largestArea_R) {
          largestArea_R = area;
          largestContour_R = contoursR[i];
          index_R = i;
      }
  }

  // Exclude contours that are too small
  if (largestArea_L < 150) {
    largestContour_L.clear();
  }

  if (largestArea_R < 150) {
    largestContour_R.clear();
  } 

  // Debug draw contours
  //namedWindow("contL");
  //drawContours(imgL, contoursL, index_L, Scalar(255, 255, 255), -1);
  //imshow("contL", imgL);

  //namedWindow("contR");
  //drawContours(imgR, contoursR, index_R, Scalar(255, 255, 255), -1);
  //imshow("contR", imgR);

  // Center detection with blob centroid
  Moments m_L = moments(largestContour_L, true);
  Point p_L(m_L.m10/m_L.m00, m_L.m01/m_L.m00);

  //cout << largestArea_R << endl;
  //cout << largestArea_L << endl;

  Moments m_R = moments(largestContour_R, true);
  Point p_R(m_R.m10/m_R.m00, m_R.m01/m_R.m00);

  // Reveal area around chosen point in original image
  Mat maskL = Mat::zeros(imgL.size(), CV_8UC1);
  Mat maskR = Mat::zeros(imgL.size(), CV_8UC1);

  // Create Circle masks
  int radius = 50;

  circle(maskL, p_L, radius, Scalar(255, 255, 255), -1);
  circle(maskR, p_R, radius, Scalar(255, 255, 255), -1);

  threshold(maskL, maskL, 127, 255, THRESH_BINARY);
  threshold(maskR, maskR, 127, 255, THRESH_BINARY);

  // Apply mask
  Mat masked_imgL, masked_imgR;
  bitwise_and(Left_nice, Left_nice, masked_imgL, maskL);
  bitwise_and(Right_nice, Right_nice, masked_imgR, maskR);

  // Debug Circular Mask
  //namedWindow("imgL");
  //imshow("imgL",masked_imgL);
  //waitKey(1);

  //namedWindow("imgR");
  //imshow("imgR",masked_imgR);
  //waitKey(1);

  try {

  // Perform ORB feature extraction and matching
  int minHessian = 400;
  Ptr<ORB> orb = ORB::create(minHessian);

  vector<KeyPoint> keypointsL, keypointsR;
  Mat descriptorsL, descriptorsR;

  // Detect keypoints in the image
  orb->detect(masked_imgL, keypointsL);
  orb->detect(masked_imgR, keypointsR);

  //Define radius
  float radius = 15.0f;

  //Filter keypoints
  vector<KeyPoint> kp_filt_L;
  for (const auto& k : keypointsL)
  {
    //Calculate Float
    float dist = sqrt(pow(k.pt.x - p_L.x, 2) + pow(k.pt.y - p_L.y, 2));

      if (dist < radius)
      {
        kp_filt_L.push_back(k);
      }
  }

  vector<KeyPoint> kp_filt_R;
  for (const auto& k : keypointsR)
  {
    //Calculate Float
    float dist = sqrt(pow(k.pt.x - p_R.x, 2) + pow(k.pt.y - p_R.y, 2));

      if (dist < radius)
      {
        kp_filt_R.push_back(k);
      }
  }

  //DEBUG: Filtering
  //cout << "Number of keypoints before filtering: " << keypointsL.size() << endl;
  //cout << "Number of keypoints after filtering: " << kp_filt_L.size() << endl;

  //Compute matches
  orb->compute(masked_imgL, kp_filt_L, descriptorsL);
  orb->compute(masked_imgR, kp_filt_R, descriptorsR);

  // Match descriptors using Brute-Force matcher
  BFMatcher matcher(NORM_HAMMING);
  vector<vector<DMatch>> knn_matches;
  matcher.knnMatch(descriptorsL, descriptorsR, knn_matches, 2);

  // Filter matches using ratio test
  const float ratio_thresh = 0.7f;
  vector<DMatch> good_matches;
  for (size_t i = 0; i < knn_matches.size(); i++) {
    if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
      good_matches.push_back(knn_matches[i][0]);
    }
  }

  // Draw good matches
  Mat img_matches;
  drawMatches(masked_imgL, kp_filt_L, masked_imgR, kp_filt_R, knn_matches, img_matches);

  //namedWindow("ORB Matches");
  //imshow("ORB Matches", img_matches);
  //waitKey(1);

  // Calculate average distance of all matched points
  double avg_distance = 0.0;
  for (const auto& matches : knn_matches) {
    if (matches.size() < 2) continue;  // Skip if not enough matches
    const auto& kp_L = kp_filt_L[matches[0].queryIdx];
    const auto& kp_R1 = kp_filt_R[matches[0].trainIdx];
    const auto& kp_R2 = kp_filt_R[matches[1].trainIdx];
    double disparity = abs(kp_L.pt.x - kp_R1.pt.x);
    double ratio = matches[0].distance / matches[1].distance;
    if (ratio < ratio_thresh) {
      double distance = (F * BASELINE) / disparity;
      avg_distance += distance;
    }
  }

  // Add RANSAC maybe?

  avg_distance /= good_matches.size();

  //Assign XYZ of ball
  Z = avg_distance;
  X = (p_L.x + p_R.x)/2;
  Y = (p_L.y + p_R.y)/2;
  area = (largestArea_L + largestArea_R)/2;

  //DEBUG READ OUTPUT
  //cout << "Distance: " << Z << endl;
  //cout << "X: " << X << endl;
  //cout << "Y: " << Y << endl;

  return true;

  }
  catch (const cv::Exception){
    cout << "Failed" << endl;
    return false;
  }
  
}

void CompVisNew::getGoal(float &X, float &Y, float &Z, float &area, float &angle, Mat imgL, Mat imgR){
  // Applying blur to reduce noise
  Mat imgBlurredL;
  GaussianBlur(imgL, imgBlurredL, Size(5, 5), 2);

  Mat imgBlurredR;
  GaussianBlur(imgR, imgBlurredR, Size(5, 5), 2);

  // Initialize matrix for rectified stereo images
  Mat Left_nice, Right_nice;

  // Applying stereo image rectification on the left image
  remap(imgBlurredL,
        Left_nice,
        Left_Stereo_Map1,
        Left_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  // Applying stereo image rectification on the right image
  remap(imgBlurredR,
        Right_nice,
        Right_Stereo_Map1,
        Right_Stereo_Map2,
        INTER_LANCZOS4,
        BORDER_CONSTANT,
        0);

  //Apply correction
  Mat goal_CL, goal_CR;
  Mat balloonCorrected_L, balloonCorrected_R;
  Mat balloonCorrect_L = Mat::zeros(Left_nice.size(), Left_nice.type());
  Mat balloonCorrect_R = Mat::zeros(Right_nice.size(), Right_nice.type());
  balloonCorrect_L.setTo(ORANGE_G_CORRECTION);
  balloonCorrect_R.setTo(ORANGE_G_CORRECTION);

  add(Left_nice, balloonCorrect_L, goal_CL);
  add(Right_nice, balloonCorrect_R, goal_CR);

  //Apply HSV
  Mat left_HSV, right_HSV;
  cvtColor(goal_CL, left_HSV, cv::COLOR_BGR2HSV);
  cvtColor(goal_CR, right_HSV, cv::COLOR_BGR2HSV);

  //Isolate Goal
  Mat bMask_L, bMask_R;
  inRange(left_HSV, ORANGE_G_MIN, ORANGE_G_MAX, bMask_L);
  inRange(right_HSV, ORANGE_G_MIN, ORANGE_G_MAX, bMask_R);

  //Noise reduction
  Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
  Mat bMask_L_cleaned, bMask_R_cleaned;
  morphologyEx(bMask_L, bMask_L_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_L_cleaned, bMask_L_cleaned, MORPH_OPEN, kernel);

  morphologyEx(bMask_R, bMask_R_cleaned, MORPH_CLOSE, kernel);
  morphologyEx(bMask_R_cleaned, bMask_R_cleaned, MORPH_OPEN, kernel);

  //DEBUG: see mask
  namedWindow("bMask_L");
  imshow("bMask_L", bMask_L_cleaned);
  waitKey(1);

  namedWindow("bMask_R");
  imshow("bMask_R", bMask_R_cleaned);
  waitKey(1);

  //Find Contours
  vector<vector<Point> > contoursL;
  vector<vector<Point> > contoursR;
  findContours(bMask_L_cleaned, contoursL, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  findContours(bMask_R_cleaned, contoursR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  // Apply morphological operations to fill in the gaps and complete the rectangle
  dilate(bMask_L_cleaned, bMask_L_cleaned, kernel);
  erode(bMask_L_cleaned, bMask_L_cleaned, kernel);
  dilate(bMask_R_cleaned, bMask_R_cleaned, kernel);
  erode(bMask_R_cleaned, bMask_R_cleaned, kernel);
  
  // Find the bounding rectangle of the largest contour
  Rect rectL;
  Rect rectR;

  if (!contoursL.empty()) {
    size_t maxAreaIndexL = 0;
    for (size_t i = 1; i < contoursL.size(); i++) {
        if (contourArea(contoursL[i]) > contourArea(contoursL[maxAreaIndexL])) {
            maxAreaIndexL = i;
        }
    }

    if (pixelDensityL < 0.35){
      rectL = boundingRect(contoursL[maxAreaIndexL]);
    } else if (pixelDensityL > 0.35 && maxAreaIndexL != 0) {
      rectL = boundingRect(contoursL[maxAreaIndexL-1]);
    } 
    //std::cout << "index: " << maxAreaIndex << endl;
  }

  if (!contoursR.empty()) {
    size_t maxAreaIndexR = 0;
    for (size_t i = 1; i < contoursR.size(); i++) {
        if (contourArea(contoursR[i]) > contourArea(contoursR[maxAreaIndexR])) {
            maxAreaIndexR = i;
        }
    }

    if (pixelDensityR < 0.35){
      rectR = boundingRect(contoursR[maxAreaIndexR]);
    } else if (pixelDensityR > 0.35 && maxAreaIndexR != 0) {
      rectR = boundingRect(contoursR[maxAreaIndexR-1]);
    } 
    //std::cout << "index: " << maxAreaIndex << endl;
  }

  // Draw a new rectangle with the same aspect ratio and orientation
  if (!rectL.empty() && !rectR.empty()) {
      double aspectRatioL = (double)rectL.width / rectL.height;
      double aspectRatioR = (double)rectR.width / rectR.height;

      int newWidthL = (int)(aspectRatioL * rectL.height);
      int newWidthR = (int)(aspectRatioR * rectR.height);

      //center and 4 corners of the bounding box
      Point centerL = Point(rectL.x + rectL.width / 2, rectL.y + rectL.height / 2);
      Point ltCorner = Point(rectL.x, rectL.y + rectL.height);;
      Point rtCorner = Point(rectL.x + rectL.width, rectL.y + rectL.height);
      Point lbCorner = Point(rectL.x, rectL.y);
      Point rbCorner = Point(rectL.x + rectL.width, rectL.y);;

      Point centerR = Point(rectR.x + rectR.width / 2, rectR.y + rectR.height / 2);

      //show center and 4 corners
      circle(bMask_L_cleaned,centerL,1,Scalar(255,255,0),3,4,0);
      // circle(bMask_L_cleaned,ltCorner,1,Scalar(255,255,0),20,4,0);
      // circle(bMask_L_cleaned,lbCorner,1,Scalar(255,255,0),20,4,0);
      // circle(bMask_L_cleaned,rtCorner,1,Scalar(255,255,0),20,4,0);
      // circle(bMask_L_cleaned,rbCorner,1,Scalar(255,255,0),20,4,0);
      circle(bMask_R_cleaned,centerR,1,Scalar(255,255,0),3,4,0);

      rectL = Rect(centerL.x - newWidthL / 2, rectL.y, newWidthL, rectL.height);
      rectR = Rect(centerR.x - newWidthR / 2, rectR.y, newWidthR, rectR.height);
      // std::cout << "Center(Left) (x,y): " << centerL.x << ", " << centerL.y << endl;
      // std::cout << "Center(Right) (x,y): " << centerR.x << ", " << centerR.y << endl;

      Mat cropedMaskL = bMask_L_cleaned.colRange(rectL.x,rectL.x + rectL.width).rowRange(rectL.y,rectL.y + rectL.height);
      Mat cropedMaskR = bMask_R_cleaned.colRange(rectR.x,rectR.x + rectR.width).rowRange(rectR.y,rectR.y + rectR.height);  

      int whitePixelsL = countNonZero(cropedMaskL);
      int whitePixelsR = countNonZero(cropedMaskR);
      //std::cout << "white pixels: " << whitePixels << endl;

      pixelDensityL = double(whitePixelsL)/double(rectL.width*rectL.height);
      pixelDensityR = double(whitePixelsR)/double(rectR.width*rectR.height);
      //std::cout << "pixel density: " << pixelDensity << endl;

      if (pixelDensityL > 0.1 && pixelDensityL < 0.35 && pixelDensityR > 0.1 && pixelDensityR < 0.35 ){
        rectangle(bMask_L_cleaned, rectL, Scalar(255), 2);
        double widthHeightRatioL = (double)rectL.width / rectL.height;
        double widthHeightRatioR = (double)rectR.width / rectR.height;
        //std::cout << "Ratio: " << widthHeightRatio << endl;
        double areaL = rectL.width*rectL.height;
        double areaR = rectR.width*rectR.height;
        double areaRelDiff =  double((areaL-areaR)/(areaR)*100);

        if (areaRelDiff < 5) {
          //Calculate dist
          double disparity = abs(centerL.x - centerR.x);

          double distance = (F * BASELINE) / disparity;
          
          double distance2 = -0.000037*(areaL+areaR)/2 + 3.371;

          if (distance > 5.0){
            distance = 1000.0;
          } 
          
          //cout << distance2 << endl;
          //std::cout << "Area (Left,Right): " << areaL << ", " << areaR << endl;

          // Set Outputs
          X = (centerL.x + centerR.x)/2;
          Y = (centerL.y + centerR.y)/2;
          Z = distance2;  // For now
          area = (areaL + areaR)/2;
          float ratio = (widthHeightRatioL+widthHeightRatioR)/2;
          //std::cout << "Ratio: " << ratio << endl;
          angle = acosf(ratio)/3.14159*180;
          if (ratio > 1){
            angle = 0;
          } 
          std::cout << "angle" << angle << endl;

        }

      } else {
        pixelDensityL = 0.2; //reinitiate
        pixelDensityR = 0.2;
      }      
  }

  // Display the resulot
  imshow("ApproximationsL", bMask_L_cleaned);
  waitKey(1);
  
  imshow("ApproximationsR", bMask_R_cleaned);
  waitKey(1);

  Mat masked_imgR_;
  bitwise_and(Left_nice, Left_nice, masked_imgR_, bMask_R_cleaned);

  namedWindow("Test");
  imshow("Test", masked_imgR_);
  waitKey(1);
  
  /*
  //Approximate shapes
  std::vector<std::vector<cv::Point>> approximationsL(contoursL.size());
  for (int i = 0; i < contoursL.size(); i++) {
      approxPolyDP(contoursL[i], approximationsL[i], 50, false);
  }

  std::vector<std::vector<cv::Point>> approximationsR(contoursR.size());
  for (int i = 0; i < contoursR.size(); i++) {
      approxPolyDP(contoursR[i], approximationsR[i], 50, false);
  }

  // Visualize approximated contours
  Mat approx_image = bMask_L_cleaned.clone();
  for (int i = 0; i < approximationsL.size(); i++) {
      cv::drawContours(bMask_L_cleaned, approximationsL, i, cv::Scalar(0, 255, 0), 2);
  }

  cv::imshow("Approximations", bMask_L_cleaned);
  cv::waitKey(1);

  std::vector<cv::Point2f> corners;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
    cv::goodFeaturesToTrack(bMask_L_cleaned, corners, 500, qualityLevel, minDistance, cv::Mat(), blockSize, useHarrisDetector, k);

    for (size_t i = 0; i < corners.size(); i++)
    {
        cv::circle(bMask_L_cleaned, corners[i], 5, cv::Scalar(255), -1);
    }

  cv::imshow("Corners", bMask_L_cleaned);
  cv::waitKey(1);

  // Show image with detected incomplete rectangles
  imshow("Detected rectangles", bMask_L_cleaned);
  waitKey(1);

  */
}

// Returns quadrant of nearest item
int CompVisNew::getAvoidance(Mat imgL, Mat imgR)
{

  // Get the dimensions of the images
  int height = imgL.rows;
  int width = imgL.cols;

  // Calculate the width and height of each section
  int section_width = width / 3;
  int section_height = height / 3;

  // Define vectors to store the rectangles and section images for each image
  std::vector<cv::Rect> sectionsL;
  std::vector<cv::Rect> sectionsR;
  std::vector<cv::Mat> section_imgsL;
  std::vector<cv::Mat> section_imgsR;

  // Loop through each section and extract it into a separate Mat object for each image
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          // Calculate the coordinates of the section
          int x = j * section_width;
          int y = i * section_height;

          // Define the rectangle for this section in the left image
          cv::Rect section_rect_L(x, y, section_width, section_height);

          // Store the rectangle in the vector for the left image
          sectionsL.push_back(section_rect_L);

          // Extract the section image from the left image and store it in the vector for the left image
          cv::Mat section_L = imgL(section_rect_L).clone();
          section_imgsL.push_back(section_L);

          // Define the rectangle for this section in the right image
          cv::Rect section_rect_R(x, y, section_width, section_height);

          // Store the rectangle in the vector for the right image
          sectionsR.push_back(section_rect_R);

          // Extract the section image from the right image and store it in the vector for the right image
          cv::Mat section_R = imgR(section_rect_R).clone();
          section_imgsR.push_back(section_R);
      }
  }

  // Show the first section image from the left image
  //cv::imshow("First section image from left image", section_imgsL[0]);
  //cv::waitKey(1);

  // Show the first section image from the right image
  //cv::imshow("First section image from right image", section_imgsR[0]);
  //cv::waitKey(1);
  
  // Define variables to keep track of the section with the smallest distance
  double min_dist = std::numeric_limits<double>::max();
  int min_index = -1;

  // Loop through each section and calculate the average distance between the corresponding pixels in each section
  for (int i = 0; i < 9; i++) {
      double dist = get_avg_dist_FM(section_imgsL[i], section_imgsR[i], to_string(i));

      // If the distance is smaller than the current minimum distance, update the variables
      if (dist < min_dist) {
          min_dist = dist;
          min_index = i;
      }
  }

  cout << "min dist: " << min_dist << endl;
  cout << "min index: " << min_index << endl;

  return 0; 
}

// Feature Mapping
float CompVisNew::get_avg_dist_FM(Mat imgL, Mat imgR, String index)
{
  try {

    // Perform ORB feature extraction and matching
    int minHessian = 400;
    Ptr<ORB> orb = ORB::create(minHessian);

    vector<KeyPoint> keypointsL, keypointsR;
    Mat descriptorsL, descriptorsR;

    // Detect keypoints in the image
    orb->detect(imgL, keypointsL);
    orb->detect(imgR, keypointsR);

    //DEBUG: Filtering
    //cout << "Number of keypoints before filtering: " << keypointsL.size() << endl;
    //cout << "Number of keypoints after filtering: " << kp_filt_L.size() << endl;

    //Compute matches
    orb->compute(imgL, keypointsL, descriptorsL);
    orb->compute(imgR, keypointsR, descriptorsR);

    // Match descriptors using Brute-Force matcher
    BFMatcher matcher(NORM_HAMMING);
    vector<vector<DMatch>> knn_matches;
    matcher.knnMatch(descriptorsL, descriptorsR, knn_matches, 2);

    // Filter matches using ratio test
    //const float ratio_thresh = 0.7f;
    //vector<DMatch> good_matches;
    //for (size_t i = 0; i < knn_matches.size(); i++) {
    //  if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
    //    good_matches.push_back(knn_matches[i][0]);
    //  }
    //}

    // Draw good matches
    Mat img_matches;
    drawMatches(imgL, keypointsL, imgR, keypointsR, knn_matches, img_matches);

    imshow(index, img_matches);
    waitKey(1);

    // Calculate average distance of all matched points
    double avg_distance = 0.0;
    for (const auto& matches : knn_matches) {
      if (matches.size() < 2) continue;  // Skip if not enough matches
      const auto& kp_L = keypointsL[matches[0].queryIdx];
      const auto& kp_R = keypointsR[matches[0].trainIdx];

      double disparity = abs(kp_L.pt.x - kp_R.pt.x);
      double ratio = matches[0].distance / matches[1].distance;

      double distance = (F * BASELINE) / disparity;
      avg_distance += distance;

    }

    // Add RANSAC maybe?

    avg_distance /= knn_matches.size();

    return avg_distance;

  }
  catch (const cv::Exception){
      cout << "Failed" << endl;
      return 1000.00;
  }
}

// Disparity Map
float CompVisNew::get_avg_dist_DM(Mat imgL, Mat imgR, String index)
{
  try {
    // Convert input images to grayscale
    Mat imgL_gray, imgR_gray;
    cvtColor(imgL, imgL_gray, COLOR_BGR2GRAY);
    cvtColor(imgR, imgR_gray, COLOR_BGR2GRAY);

    // Compute the disparity map
    Mat disp;
    int numDisparities = 16; // number of disparity levels
    int blockSize = 15; // block size for matching
    Ptr<StereoBM> bm = StereoBM::create(numDisparities, blockSize);
    bm->compute(imgL_gray, imgR_gray, disp);

    // Display the disparity map
    imshow(index, disp);
    waitKey(1);

    // Compute the average positive disparity
    long count = 0;
    float sum = 0;
    for (int i = 0; i < disp.rows; i++) {
      for (int j = 0; j < disp.cols; j++) {
        float d = disp.at<float>(i, j);
        if (d > 0) {
          sum += d;
          count++;
        }
      }
    }

    float avgDisp = sum / count;
    float distance = (F * BASELINE) / avgDisp;


    return distance; 

  }
  catch (const cv::Exception){
      cout << "Failed" << endl;
      return 1000.00;
  }
}

// Countour Area
float CompVisNew::get_avg_dist_CA(Mat imgL, Mat imgR, String index)
{
  return 0.0f;
}
