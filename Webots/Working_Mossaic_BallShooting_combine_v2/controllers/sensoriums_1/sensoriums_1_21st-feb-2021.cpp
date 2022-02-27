// File:          Sensobot.cpp
// Date:
// Description:
// Author:
// Modifications:
#include <string>
#include <vector>
#include <math.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/PositionSensor.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#define MAX_SPEED 7

using namespace webots;
using namespace std;

int moveForward(float speed, float distance);
void turnRobot(float angle);
vector<vector<int>> locateObjects();
void PID();
float getBearing(double x, double z);
vector<int> getReigion();
void gotoPurpleMat();
void mossaicAreaFunction();
void turnArm(float angle);
void slide(bool reverse=false, bool ball = false);
vector<int> getReigionHole();
vector<int> getReigionBall(char ballColor);
int gotoGrabBall();
void shoot();
void decide(char colour_path);
void Line_following_before_ball_shooting();
void PID(float error_array[3], int case_select);


char motorNames[2][15] = {"leftMotor", "rightMotor"};
double bearing{};
vector<int> objectCount{};
bool cylinderGrabbed = false;
int grabbedObj = -1;
float masterBearing{};
char given_color = 'r';

//line follow
//void Line_follow();
float line_follow_error_array[3] ={0,0,0}; // [past error, present error, accumilated error]
float line_follow_PID_constants[3]= {-0.5,-0.1,0.05};    //{P,I,D}

//PID
float controller_speed =0;

//initializing line following sensors
double max_lf_sensor_reading = 1000; // ?????
DistanceSensor *Line_folow_distance_sensor[8];

Motor *motors[2];
Motor *hinge_mot;
Motor *base1Mot;
Motor *base2Mot;
Display *display;
Camera *camera;
Robot *robot = new Robot();
Compass *compass;
PositionSensor *psensor;
PositionSensor *psensorArm;
PositionSensor *basepos1;
PositionSensor *basepos2;
PositionSensor *actPos;
Motor *actMot;

DistanceSensor *front_distance_sensor;
DistanceSensor *left_distance_sensor;
DistanceSensor *right_distance_sensor;


// get the time step of the current world.
int timeStep = (int)robot->getBasicTimeStep();

int main(int argc, char **argv)
{

  // Initializating both motors
  for (int i = 0; i < 2; i++)
  {
    motors[i] = robot->getMotor(motorNames[i]);
    motors[i]->setPosition(INFINITY);

    motors[i]->setVelocity(0.0);
  }
  robot->step(100);
  // end of initializing

   for (int i=0;i<8;i++)
  {
    string num = to_string(i);
    string name = "LF"+num;
    Line_folow_distance_sensor[i] = robot ->getDistanceSensor(name);
    Line_folow_distance_sensor[i] -> enable(timeStep);
  }
  robot->step(100);
  // initializing hinge motor
  hinge_mot = robot->getMotor("Arm_mot");
  hinge_mot->setPosition(INFINITY);
  hinge_mot->setVelocity(0.0);
  robot->step(100);
  // end of initializing

  // Initializing the compass
  compass = robot->getCompass("compass");
  compass->enable(timeStep);
  robot->step(100);
  // end of initalizing

  // initializing the camera
  camera = robot->getCamera("camera");
  display = robot->getDisplay("display");
  camera->enable(timeStep);
  // end of initializing

  // initializing position sensors
  psensor = robot->getPositionSensor("rightPosSense");
  psensor->enable(timeStep);
  robot->step(100);
  // end of einitalizing
  ///////////////////////////////////////////////////////////////////////////////
  psensorArm = robot->getPositionSensor("Arm_pos");
  psensorArm->enable(timeStep);
  robot->step(100);
  /////////////////////////////////////////////////////////////////////////////////////////////
  basepos1 = robot->getPositionSensor("basepos1");
  basepos1->enable(timeStep);
  robot->step(100);
  ////////////////////////////////////////////////////////////////////////
  basepos2 = robot->getPositionSensor("basepos2");
  basepos2->enable(timeStep);
  robot->step(100);
  //////////////////////////////////////////////////////////////////////
  base1Mot = robot->getMotor("base1Mot");
  base1Mot->setPosition(INFINITY);
  base1Mot->setVelocity(0.0);
  robot->step(100);

  base2Mot = robot->getMotor("base2Mot");
  base2Mot->setPosition(INFINITY);
  base2Mot->setVelocity(0.0);
  robot->step(100);

  actPos = robot->getPositionSensor("actPos");
  actPos->enable(timeStep);
  robot->step(100);

  actMot = robot->getMotor("actMot");
  actMot -> setPosition(INFINITY);
  actMot->setVelocity(0.0);
  robot->step(100);



  front_distance_sensor = robot->getDistanceSensor("ds_f");
  left_distance_sensor = robot->getDistanceSensor("ds_l");
  right_distance_sensor = robot->getDistanceSensor("ds_r");
  front_distance_sensor -> enable(timeStep);
  left_distance_sensor -> enable(timeStep);
  right_distance_sensor -> enable(timeStep);





  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (robot->step(timeStep) != -1)
  {
    turnArm(150);
    mossaicAreaFunction();

    cout << "Yasirus' function is over!!!!!!!!!!!!!!!" << endl;

    Line_following_before_ball_shooting();

    moveForward(1,-1);
    decide('w');
    moveForward(1,-1);
    decide(given_color);
    moveForward(1,7.5);
    if (given_color=='r'){turnRobot(16.4);}
    else if(given_color=='b'){turnRobot(-16.4);}
    turnArm(-150);
    slide(false);
    shoot();
    turnArm(90);


    // ending the scene
    while (robot->step(timeStep) != -1){
    }
  }; // end of main while loop
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  delete robot;
  return 0;
} // end of main function

int moveForward(float speed, float distance = -1)
{

  // NOTE: Here speed is a fraction. It implies the fraction of maximum speed
  float startPos = psensor -> getValue();
  // setting the motor speed
  motors[0]->setVelocity(speed * MAX_SPEED);
  motors[1]->setVelocity(speed * MAX_SPEED);

  if (distance > 0){
    while (robot->step(timeStep) != -1){
      if(abs(psensor -> getValue() - startPos) > distance){
        motors[0]->setVelocity(0);
        motors[1]->setVelocity(0);
        break;
      }
    }
  }

  return 0;
} // end of moveForward function

float getBearing(double x, double z)
{
  // x  --> sin curve
  // z  --> cos curve

  x = compass->getValues()[0];
  z = compass->getValues()[2];

  bearing = asin(x) * 57.29577;

  if (x > z && z < 0)
  {
    bearing = 180 - bearing;
  }
  else if (z < 0)
  {
    // bot in 225,360   0,45
    bearing = -180 - bearing;
  }
  if (bearing < 0)
  {
    bearing = 360 + bearing;
  }

  return bearing;
}

void turnRobot(float angle)
{
  // reading compass direction
  double x = compass->getValues()[0];
  double z = compass->getValues()[2];

  double start_location = getBearing(x, z);

  double end_location = start_location + angle;
  double diff{};
  double I{};

  // if (end_location < 0) {end_location += 360; status = true;}
  // if (end_location > 360) {end_location -= 360; status = true;}
  if (end_location < 0)
  {
    end_location += 360;
  }
  if (end_location > 360)
  {
    end_location -= 360;
  }

  motors[0]->setVelocity((angle / abs(angle)) * 0.4 * MAX_SPEED);
  motors[1]->setVelocity((angle / abs(angle)) * -0.4 * MAX_SPEED);

  while (robot->step(timeStep) != -1)
  {

    x = compass->getValues()[0];
    z = compass->getValues()[2];

    bearing = getBearing(x, z);

    diff = abs(bearing - start_location);
    if (diff > 2.27198)
      diff = 2.27198;

    I += diff;
    if (I > abs(angle))
    {
      motors[0]->setVelocity(0);
      motors[1]->setVelocity(0);
      break;
    }
    start_location = bearing;
  } // end of while loop

} // end of turning

vector<int> getReigion()
{

  //////////////////////////////////////////////////////////////
  // DOCTRING: this function identifies the objects
  // RETURN: a vector < x_center1, y_center1, height1,  x_center2, y_center2, height2  >
  //////////////////////////////////////////////////////////////

  vector<int> processed_data{};

  int hmin = 18, smin = 50, vmin = 20;
  int hmax = 26, smax = 255, vmax = 255;

  int hmin_ = 18, smin_ = 100, vmin_ = 60;
  int hmax_ = 25, smax_ = 255, vmax_ = 120;

  const unsigned char *cam_feed;
  int im_height = 640;
  int im_width = 640;
  cv::Mat cvImage = cv::Mat(cv::Size(im_height, im_width), CV_8UC4);
  cv::Mat imgCopy;
  cv::Mat imgGray, imgBlur, imgDil;
  cv::Mat imgCanny;
  cv::Mat imgHsv, mask, res, mask1;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cam_feed = camera->getImage();

  // camera -> saveImage("mat_new.png", 3);

  cvImage.data = (uchar *)cam_feed;

  //croping the image
  cv::Rect myROI(0,280, im_width, im_height - 280);
  cv::Mat cropMask = cv::Mat::zeros(im_height, im_width, CV_8U);
  cropMask(myROI) = 1;
  cv::bitwise_and(cvImage, cvImage, cvImage, mask=cropMask);


  cv::cvtColor(cvImage, imgHsv, cv::COLOR_BGR2HSV);
  
  cv::Scalar lowerLim(hmin, smin, vmin);
  cv::Scalar upperLim(hmax, smax, vmax);
  cv::Scalar lowerLim_(hmin_, smin_, vmin_);
  cv::Scalar upperLim_(hmax_, smax_, vmax_);
  
  cv::inRange(imgHsv, lowerLim, upperLim, mask);
  cv::inRange(imgHsv, lowerLim_, upperLim_, mask1);

  mask = mask | mask1; // combining two masks

  cv::bitwise_and(cvImage, cvImage, res, mask = mask);


  cv::cvtColor(res, imgGray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(imgGray, imgBlur, cv::Size(3, 3), 3, 0);
  cv::Canny(imgBlur, imgCanny, 25, 75);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  // cv::erode(imgCanny, imgCanny, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::dilate(imgCanny, imgDil, kernel);
  cv::findContours(imgDil, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<vector<cv::Point>> conPoly(contours.size());
  vector<cv::Rect> boundRect(contours.size());

  int objectIdentity {};  // 0 for cylinder 1 for box

  for (long long unsigned int i = 0; i < contours.size(); i++)
  {
    int area = cv::contourArea(contours[i]);
    string objectType;

    if (area > 800)
    {
      float peri = cv::arcLength(contours[i], true);
      cv::approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
      boundRect[i] = cv::boundingRect(conPoly[i]);
      int objCor = (int)conPoly[i].size();
      
      if (objCor <= 7)
      {
        objectType = "BOX";
        objectIdentity = 1;
      }
      else if (objCor > 7)
      {
        objectType = "CYLY";
        objectIdentity = 0;
      }

      cv::drawContours(res, conPoly, i, cv::Scalar(255, 255, 255), 2);
      cv::rectangle(res, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255, 0, 0), 3);
      cv::putText(res, objectType, {boundRect[i].x, boundRect[i].y - 5}, cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255), 3);
      cv::drawContours(res, contours, -1, cv::Scalar(255, 255, 255), 1);

      processed_data.push_back(boundRect[i].x + boundRect[i].width / 2);
      processed_data.push_back(boundRect[i].y + boundRect[i].height / 2);
      processed_data.push_back(boundRect[i].height);
      processed_data.push_back(objectIdentity);
      processed_data.push_back(boundRect[i].width);
    }
  }

  ImageRef *ir = display->imageNew(im_width, im_height, res.data, Display::BGRA);
  display->imagePaste(ir, 0, 0, false);
  display->imageDelete(ir);

  return processed_data;
}

vector<int> getReigionBall(char ballColor)
{

  //////////////////////////////////////////////////////////////
  // DOCTRING: this function identifies the objects
  // RETURN: a vector < x_center1, y_center1, height1,  x_center2, y_center2, height2  >
  //////////////////////////////////////////////////////////////

  vector<int> processed_data{};

  int hmin, smin, vmin, hmax, smax, vmax;

  int hmin_, smin_, vmin_, hmax_ , smax_ , vmax_;

  if(ballColor == 'r'){
    hmin = 0, smin = 100, vmin = 20;
    hmax = 10, smax = 255, vmax = 255;

    hmin_ = 160, smin_ = 100, vmin_ = 20;
    hmax_ = 179, smax_ = 255, vmax_ = 120;
  }else{
    hmin = 110, smin = 150, vmin = 20;
    hmax = 130, smax = 250, vmax = 255;

    hmin_ = 110, smin_ = 150, vmin_ = 20;
    hmax_ = 130, smax_ = 250, vmax_ = 255;
  }

  

  const unsigned char *cam_feed;
  int im_height = 640;
  int im_width = 640;
  cv::Mat cvImage = cv::Mat(cv::Size(im_height, im_width), CV_8UC4);
  cv::Mat imgCopy;
  cv::Mat imgGray, imgBlur, imgDil;
  cv::Mat imgCanny;
  cv::Mat imgHsv, mask, res, mask1;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cam_feed = camera->getImage();

  // camera -> saveImage("mat_new.png", 3);

  cvImage.data = (uchar *)cam_feed;

  //croping the image
  cv::Rect myROI(0,280, im_width, im_height - 280);
  cv::Mat cropMask = cv::Mat::zeros(im_height, im_width, CV_8U);
  cropMask(myROI) = 1;
  cv::bitwise_and(cvImage, cvImage, cvImage, mask=cropMask);


  cv::cvtColor(cvImage, imgHsv, cv::COLOR_BGR2HSV);
  
  cv::Scalar lowerLim(hmin, smin, vmin);
  cv::Scalar upperLim(hmax, smax, vmax);
  cv::Scalar lowerLim_(hmin_, smin_, vmin_);
  cv::Scalar upperLim_(hmax_, smax_, vmax_);
  
  cv::inRange(imgHsv, lowerLim, upperLim, mask);
  cv::inRange(imgHsv, lowerLim_, upperLim_, mask1);

  mask = mask | mask1; // combining two masks

  cv::bitwise_and(cvImage, cvImage, res, mask = mask);


  cv::cvtColor(res, imgGray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(imgGray, imgBlur, cv::Size(3, 3), 3, 0);
  cv::Canny(imgBlur, imgCanny, 25, 75);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  // cv::erode(imgCanny, imgCanny, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::dilate(imgCanny, imgDil, kernel);
  cv::findContours(imgDil, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<vector<cv::Point>> conPoly(contours.size());
  vector<cv::Rect> boundRect(contours.size());

  int objectIdentity {};  // 0 for cylinder 1 for box

  for (long long unsigned int i = 0; i < contours.size(); i++)
  {
    int area = cv::contourArea(contours[i]);
    string objectType;

    if (area > 280)
    {
      float peri = cv::arcLength(contours[i], true);
      cv::approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
      boundRect[i] = cv::boundingRect(conPoly[i]);
      int objCor = (int)conPoly[i].size();
      
      if (objCor <= 7)
      {
        objectType = "BOX";
        objectIdentity = 1;
      }
      else if (objCor > 7)
      {
        objectType = "CYLY";
        objectIdentity = 0;
      }

      cv::drawContours(res, conPoly, i, cv::Scalar(255, 255, 255), 2);
      cv::rectangle(res, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255, 0, 0), 3);
      cv::putText(res, objectType, {boundRect[i].x, boundRect[i].y - 5}, cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255), 3);
      cv::drawContours(res, contours, -1, cv::Scalar(255, 255, 255), 1);

      processed_data.push_back(boundRect[i].x + boundRect[i].width / 2);
      processed_data.push_back(boundRect[i].y + boundRect[i].height / 2);
      processed_data.push_back(boundRect[i].height);
      processed_data.push_back(objectIdentity);
      processed_data.push_back(boundRect[i].width);
    }
  }

  ImageRef *ir = display->imageNew(im_width, im_height, res.data, Display::BGRA);
  display->imagePaste(ir, 0, 0, false);
  display->imageDelete(ir);

  return processed_data;
}

vector<int> getReigionHole()
{

  //////////////////////////////////////////////////////////////
  // DOCTRING: this function identifies the objects
  // RETURN: a vector < x_center1, y_center1, height1,  x_center2, y_center2, height2  >
  //////////////////////////////////////////////////////////////

  vector<int> processed_data{};

  int hmin = 110, smin = 0, vmin = 120;
  int hmax = 130, smax = 50, vmax = 200;

  // int hmin_ = 18, smin_ = 100, vmin_ = 60;
  // int hmax_ = 25, smax_ = 255, vmax_ = 120;

  const unsigned char *cam_feed;
  int im_height = 640;
  int im_width = 640;
  cv::Mat cvImage = cv::Mat(cv::Size(im_height, im_width), CV_8UC4);
  cv::Mat imgCopy;
  cv::Mat imgGray, imgBlur, imgDil;
  cv::Mat imgCanny;
  cv::Mat imgHsv, mask, res, mask1;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cam_feed = camera->getImage();

  // camera -> saveImage("mat_new.png", 3);

  cvImage.data = (uchar *)cam_feed;


  cv::cvtColor(cvImage, imgHsv, cv::COLOR_BGR2HSV);
  
  cv::Scalar lowerLim(hmin, smin, vmin);
  cv::Scalar upperLim(hmax, smax, vmax);
  // cv::Scalar lowerLim_(hmin_, smin_, vmin_);
  // cv::Scalar upperLim_(hmax_, smax_, vmax_);
  
  cv::inRange(imgHsv, lowerLim, upperLim, mask);
  // cv::inRange(imgHsv, lowerLim_, upperLim_, mask1);

  // mask = mask | mask1; // combining two masks

  cv::bitwise_and(cvImage, cvImage, res, mask = mask);


  cv::cvtColor(res, imgGray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(imgGray, imgBlur, cv::Size(3, 3), 3, 0);
  cv::Canny(imgBlur, imgCanny, 25, 75);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  // cv::erode(imgCanny, imgCanny, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::dilate(imgCanny, imgDil, kernel);
  cv::findContours(imgDil, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<vector<cv::Point>> conPoly(contours.size());
  vector<cv::Rect> boundRect(contours.size());

  int objectIdentity {};  // 0 for cylinder 1 for box

  for (long long unsigned int i = 0; i < contours.size(); i++)
  {
    int area = cv::contourArea(contours[i]);
    string objectType;

    if (area > 500)
    {
      float peri = cv::arcLength(contours[i], true);
      cv::approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
      boundRect[i] = cv::boundingRect(conPoly[i]);
      int objCor = (int)conPoly[i].size();
      
      if (objCor <= 10)
      {
        objectType = "BOX";
        objectIdentity = 1;
      }
      else if (objCor > 10)
      {
        objectType = "CYLY";
        objectIdentity = 0;
      }

      cv::drawContours(res, conPoly, i, cv::Scalar(255, 255, 255), 2);
      cv::rectangle(res, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255, 0, 0), 3);
      cv::putText(res, objectType, {boundRect[i].x, boundRect[i].y - 5}, cv::FONT_HERSHEY_PLAIN, 3, cv::Scalar(255, 255, 255), 3);
      cv::drawContours(res, contours, -1, cv::Scalar(255, 255, 255), 1);

      processed_data.push_back(boundRect[i].x + boundRect[i].width / 2);
      processed_data.push_back(boundRect[i].y + boundRect[i].height / 2);
      processed_data.push_back(boundRect[i].height);
      processed_data.push_back(objectIdentity);
      processed_data.push_back(boundRect[i].width);
    }
  }

  ImageRef *ir = display->imageNew(im_width, im_height, res.data, Display::BGRA);
  display->imagePaste(ir, 0, 0, false);
  display->imageDelete(ir);

  return processed_data;
}

void gotoPurpleMat()
{

  // const int CORRECTION_ERR = 2;
  // const double DELTA_SPEED = 0.03;

  int hmin = 147, smin = 150, vmin = 150;
  int hmax = 153, smax = 255, vmax = 255;

  const unsigned char *cam_feed;
  int im_height = 640;
  int im_width = 640;

  int deltaY{};

  moveForward(1);

  while (robot->step(timeStep) != -1)
  {

    cv::Mat cvImage = cv::Mat(cv::Size(im_height, im_width), CV_8UC4);
    cv::Mat imgCopy;
    cv::Mat imgGray, imgBlur, imgDil;
    cv::Mat imgCanny;
    cv::Mat imgHsv, mask, res;
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cam_feed = camera->getImage();

    // camera -> saveImage("New_image.png", 3);

    cvImage.data = (uchar *)cam_feed;
    cv::cvtColor(cvImage, imgHsv, cv::COLOR_BGR2HSV);
    cv::Scalar lowerLim(hmin, smin, vmin);
    cv::Scalar upperLim(hmax, smax, vmax);
    cv::inRange(imgHsv, lowerLim, upperLim, mask);
    cv::bitwise_and(cvImage, cvImage, res, mask = mask);
    cv::cvtColor(res, imgGray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(imgGray, imgBlur, cv::Size(3, 3), 3, 0);
    cv::Canny(imgBlur, imgCanny, 25, 75);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // cv::erode(imgCanny, imgCanny, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(imgCanny, imgDil, kernel);
    cv::findContours(imgDil, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    vector<vector<cv::Point>> conPoly(contours.size());
    vector<cv::Rect> boundRect(contours.size());

    for (long long unsigned int i = 0; i < contours.size(); i++)
    {
      int area = cv::contourArea(contours[i]);
      string objectType;

      if (area > 500)
      {
        float peri = cv::arcLength(contours[i], true);
        cv::approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
        boundRect[i] = cv::boundingRect(conPoly[i]);


        cv::drawContours(res, conPoly, i, cv::Scalar(255, 255, 255), 2);
        cv::rectangle(res, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255, 0, 0), 3);
        cv::putText(res, objectType, { boundRect[i].x,boundRect[i].y - 5 }, cv::FONT_HERSHEY_PLAIN,3, cv::Scalar(255, 255, 255),3);
        cv::drawContours(res, contours, -1, cv::Scalar(255, 255, 255), 1);
      }

      // get maximum y values                    1 and 2nd
      cv::line(res, conPoly[0][1], conPoly[0][2], cv::Scalar(255, 0, 0), 3);
    }

    ImageRef *ir = display->imageNew(im_width, im_height, res.data, Display::BGRA);
    display->imagePaste(ir, 0, 0, false);
    display->imageDelete(ir);

    if (conPoly[0][1].y > 630 || conPoly[0][2].y > 630 || abs(deltaY) > 30)
    {
      moveForward(0);
      break;
    }

  } // end of while loop

  ///////////////////////////
  /// IMPORTANT .... now alignment is over. and starting to enter the mosaic area
  ///////////////////////////
  int starting_position = psensor->getValue();
  const int purple_distance = 16; // this is the distance we should travel after correcting the alignment

  moveForward(1);

  while (robot->step(timeStep) != -1)
  {
    if (psensor->getValue() >= starting_position + purple_distance)
    {
      moveForward(0);
      break;
    }
  }


  //time to save the bearing
  // reading compass direction
  masterBearing = getBearing(compass->getValues()[0], compass->getValues()[2]);

} // end of gotoPurpleMat function

int gotoGrabBall(){
  // int objectCount{}; // this will count the number of object captured
  vector<int> reigionData{};
  int currentAngle{};

  // caling getregion

  // reading object
  //  reigionData = getReigion();


  while (robot->step(timeStep) != -1)
  {
    if (getReigionBall(given_color).size() == 0)
    {
      turnRobot(-10);
      currentAngle += 10;
    }
    else
    {
      turnRobot(-20);
      break;
    }
  }

  moveForward(1.4, 1);

  // at this point we are guarentied to find a object

  // now centering the object
  const int CORRECTION_ERR = 2;
  const double DELTA_SPEED = 0.09;
  int deltaX{};

  bool object_centered = false;

  short objectID {};
  int cylinder {};
  int box {};


  // short centered_object {-1};

  while (robot->step(timeStep) != -1)
  {
    
    reigionData = getReigionBall(given_color);
    // select first object is a wrong thing
    // we have to select the nearby object

    // we have to select only if there are more than 2 objects on the site
    if(reigionData.size() >= 10){
      if (reigionData[5] >= 0 && reigionData[5] <= 640){
        if (reigionData[2] < reigionData[7]){
          objectID = 5; // this means we selected the second object
        }
      }
    }else{
      objectID = 0;
    }

  if(reigionData.size() > 0){

    if (!object_centered)
    {
      
      if(  reigionData[objectID + 3] == 0) cylinder ++;
      else if (reigionData[objectID + 3] == 1) box ++;

      deltaX = 320 - reigionData[objectID];

      if (deltaX < -1 * CORRECTION_ERR)
      {
        // setting the motor speed
        motors[0]->setVelocity((0.40 + DELTA_SPEED) * MAX_SPEED);
        motors[1]->setVelocity(0.40 * MAX_SPEED);
      }
      else if (deltaX > CORRECTION_ERR)
      {
        // setting the motor speed
        motors[0]->setVelocity(0.60 * MAX_SPEED);
        motors[1]->setVelocity((0.60 + DELTA_SPEED) * MAX_SPEED);
      }
      else
      {
        object_centered = true;
        // centered_object = reigionData[objectID + 3];
        motors[0]->setVelocity(0.6 * MAX_SPEED);
        motors[1]->setVelocity(0.6 * MAX_SPEED);

      }
    } // end of object centering

    /////////////////////////  HERE REMEMBER TO EDIT 300 value (height when should be stopped)
    if (reigionData[objectID + 2] > 250)
    {
      // cout << reigionData[objectID + 2] << "," << reigionData[objectID + 4] << endl;
      moveForward(0);
      break;
    }
  }

  } // end of while loop

  if (box > cylinder) return 1;
  else return 0;
}// end of grab object function

int gotoGrabObject(){
  // int objectCount{}; // this will count the number of object captured
  vector<int> reigionData{};
  int currentAngle{};

  // caling getregion

  // reading object
  //  reigionData = getReigion();


  while (robot->step(timeStep) != -1)
  {
    if (getReigion().size() == 0)
    {
      turnRobot(-10);
      currentAngle += 10;
    }
    else
    {
      break;
    }
  }

  // at this point we are guarentied to find a object

  // now centering the object
  const int CORRECTION_ERR = 2;
  const double DELTA_SPEED = 0.09;
  int deltaX{};

  bool object_centered = false;

  short objectID {};
  int cylinder {};
  int box {};


  // short centered_object {-1};

  while (robot->step(timeStep) != -1)
  {

    reigionData = getReigion();

    // select first object is a wrong thing
    // we have to select the nearby object

    // we have to select only if there are more than 2 objects on the site
    if(reigionData.size() >= 10){
      if (reigionData[5] >= 0 && reigionData[5] <= 640){
        if (reigionData[2] < reigionData[7]){
          objectID = 5; // this means we selected the second object
        }
      }
    }else{
      objectID = 0;
    }

    if(reigionData.size() > 0){

    if (!object_centered)
    {
      
      if(  reigionData[objectID + 3] == 0) cylinder ++;
      else if (reigionData[objectID + 3] == 1) box ++;

      deltaX = 320 - reigionData[objectID];

      if (deltaX < -1 * CORRECTION_ERR)
      {
        // setting the motor speed
        motors[0]->setVelocity((0.40 + DELTA_SPEED) * MAX_SPEED);
        motors[1]->setVelocity(0.40 * MAX_SPEED);
      }
      else if (deltaX > CORRECTION_ERR)
      {
        // setting the motor speed
        motors[0]->setVelocity(0.60 * MAX_SPEED);
        motors[1]->setVelocity((0.60 + DELTA_SPEED) * MAX_SPEED);
      }
      else
      {
        object_centered = true;
        // centered_object = reigionData[objectID + 3];
        motors[0]->setVelocity(0.6 * MAX_SPEED);
        motors[1]->setVelocity(0.6 * MAX_SPEED);

      }
    } // end of object centering

    /////////////////////////  HERE REMEMBER TO EDIT 300 value (height when should be stopped)
    if (reigionData[objectID + 2] > 250)
    {
      // cout << reigionData[objectID + 2] << "," << reigionData[objectID + 4] << endl;
      moveForward(0);
      break;
    }
    }
  } // end of while loop

  if (box > cylinder) return 1;
  else return 0;
}// end of grab object function

void checkCylinder(){

  if (!cylinderGrabbed){
    float disatance = 7;

    turnRobot(-90);
    moveForward(1, disatance);
    turnRobot(90);
    moveForward(1, disatance);
    turnRobot(90);
    moveForward(-1, 8);
  }
  
}

void placeInHole(int objectID){

  int stopDist{};

  cout << "We have a ";
  if (objectID) {
    cout << "BOX" << endl;
    stopDist = 100;
  }
  else if(objectID == 0) {
    cout << "Cylinder" << endl;;
    stopDist = 300;
  }
  double currentBearing = getBearing(compass->getValues()[0], compass->getValues()[2]); // this is the current bearing
  double roatational_angle = 180 - (currentBearing - masterBearing);

  if (roatational_angle >= 360){
    roatational_angle = roatational_angle - 360;
  }else if (roatational_angle < 0){
    roatational_angle = 360 - roatational_angle;
  }

  turnRobot(roatational_angle);

  /////////////////////////////////////////////////////////////////////////////////////////////////////////

  float tempReading = left_distance_sensor -> getValue();
  tempReading = 0.0283 * (750 - tempReading);

  if(left_distance_sensor -> getValue() < 750){
    turnRobot(90);
    moveForward(1, tempReading);
    turnRobot(-90);
    moveForward(1,5);
  }


  float distanceBuffer = left_distance_sensor -> getValue();
  
  moveForward(0.7);

  while (robot->step(timeStep) != -1){
    
    tempReading = left_distance_sensor -> getValue();

    if( abs(tempReading - distanceBuffer) < 40){
      if(tempReading - distanceBuffer < 0){
        motors[1]->setVelocity((0.7 - 0.05) * MAX_SPEED);
      }else{
        motors[1]->setVelocity((0.7 + 0.05) * MAX_SPEED);
      }
    }else{
      distanceBuffer = tempReading;
    }

    if (front_distance_sensor -> getValue() < stopDist){
      moveForward(0);
      break;
    }
  }// end of while loop

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  turnRobot(90);

  moveForward(1);


  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  vector<int> reigionData{};
  int objectShifter {};
  
  while (robot->step(timeStep) != -1){
    reigionData = getReigionHole(); 

    // if(reigionData.size() == 10 && reigionData[5] <= 640){
    if(reigionData.size() > 5){
      // this means two object
      // now find the object nearest to the center
      // if(abs(reigionData[5] - 320 ) <  abs( reigionData[0] - 320 )) objectShifter = 5;
      vector<int> tempReig {};

      for(int i = 0; i < reigionData.size() / 5; ++i){
        if( abs(reigionData[5*i] - 320) < 160){
          if(reigionData[5*i + 3] == objectID) tempReig.push_back(i);
        }
      }// end of for loop

      if (tempReig.size() > 1){
          if(reigionData[5 * tempReig[0]] < reigionData[5 * tempReig[1]]){
            objectShifter = 5 * tempReig[0];
          }else{
            objectShifter = 5 * tempReig[1];
          }
      }else{
        objectShifter = 5 * tempReig[0];
      }

    }//end of if size > 5
    else{
      objectShifter = 0;
    }

    if((reigionData[objectShifter] - 320) < 0){
      motors[1]->setVelocity((1 + 0.05) * MAX_SPEED);
    }else{
      motors[1]->setVelocity((1 - 0.05) * MAX_SPEED);
    }
    
    // for (int i = 0; i < reigionData.size(); ++i){
    //   cout << reigionData[i] << ", ";
    // }

    // cout << "\t\t" << reigionData[objectShifter + 4] << "\t" << objectShifter << endl << endl;

    // cout << reigionData[objectShifter + 4] << endl;  //printing width
    if(reigionData[objectShifter + 4] > 220){
      moveForward(0);
      break;
    }

  }// end of while
   



  /////////////////////////////////////////////////////////////////////////////////////////////////////////


  turnArm(-90);
  moveForward(0.4, 2);
  slide(true, false);
  moveForward(-1,2);
  slide(false, false);
  moveForward(0.3,1.5);
  slide(true, false);
  turnArm(150);
}

void mossaicAreaFunction()
{
  short objectID{};
  // goto purple mat
  gotoPurpleMat();

  // grab the first object
  objectID = gotoGrabObject();

  // cout << "Object ready to grab -> " << objectID << endl;

  if (objectID == 1 && !cylinderGrabbed){
    checkCylinder();
    objectID = gotoGrabObject();
  }
  grabbedObj = objectID;
  if (objectID == 0) cylinderGrabbed = true;

  turnArm(-150);
  slide(false, false);
  turnArm(90);
  placeInHole(objectID);

  moveForward(-1, 30);
  turnRobot(90);
  



  objectID = gotoGrabObject();

  // cout << "Object ready to grab -> " << objectID << endl;

  if (objectID == 1 && !cylinderGrabbed){
    checkCylinder();
    objectID = gotoGrabObject();
  }
  grabbedObj = objectID;
  if (objectID == 0) cylinderGrabbed = true;

  turnArm(-150);
  slide(false, false);
  turnArm(90);
  placeInHole(objectID);


  gotoGrabBall();
  turnArm(-150);
  slide(false, true);
  turnArm(150);


  double currentBearing = getBearing(compass->getValues()[0], compass->getValues()[2]); // this is the current bearing
  double roatational_angle = 180 - (currentBearing - masterBearing);

  if (roatational_angle >= 360){
    roatational_angle = roatational_angle - 360;
  }else if (roatational_angle < 0){
    roatational_angle = 360 - roatational_angle;
  }

  turnRobot(roatational_angle);

  float tempReading = left_distance_sensor -> getValue();
  float tempReading__ = front_distance_sensor -> getValue();

  float distanceBuffer = left_distance_sensor -> getValue();
  int multiplier = 1;
  int threshold = 660;  // IMPORTANT EXPERIMENTAL VALUE

  if (tempReading__ < threshold){
    // tempReading = 0.0283 * (750 - tempReading);
    multiplier = -1;
  }

  moveForward(multiplier * 0.7);

  while (robot->step(timeStep) != -1){
    
    tempReading = left_distance_sensor -> getValue();

    if( abs(tempReading - distanceBuffer) < 40){
      if(tempReading - distanceBuffer < 0){
        motors[1]->setVelocity(multiplier * (0.7 - 0.05) * MAX_SPEED);
      }else{
        motors[1]->setVelocity(multiplier * (0.7 + 0.05) * MAX_SPEED);
      }
    }else{
      distanceBuffer = tempReading;
    }

    tempReading__ = front_distance_sensor -> getValue();

    if (tempReading__ < 2000){
      if(multiplier == -1){
        if (tempReading__ > threshold){
          moveForward(0);
          break;
        }
      }
      else if(multiplier == 1){
        if (tempReading__ < threshold){
          moveForward(0);
          break;
        }
      }

    }
    


  }// end of while loop

  
  
  
  tempReading = left_distance_sensor -> getValue();
  distanceBuffer = left_distance_sensor -> getValue();
  float deltaPos = (0.02826 * (1840 - tempReading));
  
  // cout << "Delta Position -> " << deltaPos << endl;
  

  turnRobot(90);

  float startPos = psensor -> getValue();
  // cout << tempReading << "\t" << deltaPos << "\t" << startPos << endl;

  moveForward(0.7);

  while (robot->step(timeStep) != -1){
    
    tempReading = left_distance_sensor -> getValue();

    if( abs(tempReading - distanceBuffer) < 40){
      if(tempReading - distanceBuffer < 0){
        motors[1]->setVelocity( (0.7 - 0.05) * MAX_SPEED);
      }else{
        motors[1]->setVelocity((0.7 + 0.05) * MAX_SPEED);
      }
    }else{
      distanceBuffer = tempReading;
    }

    // cout << abs(psensor -> getValue() - startPos) << endl;
    if( abs(psensor -> getValue() - startPos) > deltaPos ){
      moveForward(0);
      break;
    }

  }










} // end of mossai area function

void turnArm(float angle)
{
  // int starting_position = psensor -> getValue();
  float velocity = angle / abs(angle);
  float startPos = psensorArm->getValue();
  while (robot->step(timeStep) != -1)
  {
    float pos = psensorArm->getValue();
    hinge_mot->setVelocity(velocity);
    if ((abs(startPos - pos) / 3.142) * 180 > abs(angle))
    {
      hinge_mot->setVelocity(0);
      break;
    }

  }
}// end of arm tuurning

void slide(bool reverse, bool ball)
{

  float END_POS;
  if (ball) END_POS = 0.025;
  else END_POS = 0.02;

  base1Mot->setVelocity(0.01); //done
  base2Mot->setVelocity(0.01);

  if(reverse){
    base1Mot -> setPosition(END_POS); //done
    base2Mot -> setPosition(-END_POS);
  }else{
    base1Mot -> setPosition(-END_POS); //done
    base2Mot -> setPosition(END_POS);
  }
  
  

  int count = 0;
  double pos1 = basepos1->getValue();
  double pos2 = basepos2->getValue();
  double prevoiusPos1;
  double prevoiusPos2;

  while (robot->step(timeStep) != -1 && !reverse)
  {
    prevoiusPos1 = pos1;
    prevoiusPos2 = pos2;
    pos1 = basepos1->getValue();
    pos2 = basepos2->getValue();

    if ( abs(prevoiusPos1 - pos1) < 0.001 && abs(prevoiusPos2-pos2) < 0.001 ) {
      count += 1;
    }
    else{
      count = 0;
    }

    if (count > 100) {
      break;
    }
  }
}

void shoot(){

 // actMot->setVelocity(0.01);


  const float END_POS = 0.08;

  actMot->setVelocity(10); //done
 

  actMot -> setPosition(END_POS); //done
  
  

  int count = 0;
  double pos1 = actPos->getValue();
  double prevoiusPos1;
  

  while (robot->step(timeStep) != -1)
  {
    
    //cout << basepos1->getValue() << "       " << basepos2->getValue() << endl;
    // cout << 0.03*2 -abs(basepos1->getValue()) -abs(basepos2->getValue())<<endl;
    // if ( 0.03*2 -abs(basepos1->getValue()) -abs(basepos2->getValue()) == 0.05+0.001){
      // break;
    // }
    prevoiusPos1 = pos1;
    
    pos1 = actPos->getValue();
    

    if ( abs(prevoiusPos1 - pos1) < 0.001) {
      count += 1;
    }
    else{
      count = 0;
    }

    if (count > 100) {
      break;
    }
  }
  cout << "Loop Broke" << endl;
  actMot -> setPosition(0);
}

void decide(char colour_path){
  char condition = 'g';
  

  double prev_max = 0;
  while (robot->step(timeStep) != -1){

  if (condition=='t'){
    break;
  }
  
  int im_width = camera->getWidth();
  int im_height = camera->getHeight();
  const unsigned char* img;
  
  cv::Mat img1 = cv::Mat(cv::Size(im_height, im_width), CV_8UC4);
  img = camera->getImage();
    
  if(img) {
      
      img1.data = (uchar *)img;   //get image
      cv::Mat hsv_image,mask, filtered, filtered_RGB, lower_mask, upper_mask,gray_output;

      if (colour_path == 'w'){
      cout << "white" << endl;
      int low_h = 0, low_s = 0, low_v = 20;
	    int high_h = 255, high_s = 50, high_v = 255;   //filter for white
      cv::cvtColor(img1,hsv_image , cv::COLOR_BGR2HSV);
      cv::inRange(hsv_image, cv::Scalar(low_h, low_s, low_v), cv::Scalar(high_h, high_s, high_v),mask);
      cv::bitwise_and(hsv_image, hsv_image,filtered,mask = mask);
	    cv::cvtColor(filtered, filtered_RGB, cv::COLOR_HSV2RGB);}

      else if(colour_path=='b'){
       
      int low_h = 110, low_s = 150, low_v = 20;
	    int high_h = 130, high_s = 250, high_v = 255;   //filter for white
      cv::cvtColor(img1,hsv_image , cv::COLOR_BGR2HSV);
      cv::inRange(hsv_image, cv::Scalar(low_h, low_s, low_v), cv::Scalar(high_h, high_s, high_v),mask);
      cv::bitwise_and(hsv_image, hsv_image,filtered,mask = mask);
	    cv::cvtColor(filtered, filtered_RGB, cv::COLOR_HSV2RGB);
      }

      else if(colour_path=='r'){    
        cv::cvtColor(img1,hsv_image , cv::COLOR_BGR2HSV);
        int low_h_r = 0, low_s_r = 100, low_v_r = 20;
	      int high_h_r = 10, high_s_r = 255, high_v_r = 255;
        inRange(hsv_image, cv::Scalar(low_h_r, low_s_r, low_v_r), cv::Scalar(high_h_r, high_s_r, high_v_r), lower_mask);
        
        low_h_r = 160, low_s_r = 100, low_v_r = 20;
        high_h_r = 179, high_s_r = 255, high_v_r = 255;
        inRange(hsv_image, cv::Scalar(low_h_r, low_s_r, low_v_r), cv::Scalar(high_h_r, high_s_r, high_v_r), upper_mask);
        
        mask = lower_mask + upper_mask;

        cv::bitwise_and(hsv_image, hsv_image,filtered,mask = mask);
	      cv::cvtColor(filtered, filtered_RGB, cv::COLOR_HSV2RGB);
      }

      cvtColor(filtered_RGB, gray_output, cv::COLOR_RGB2GRAY);  //get gray image


      int no_rows = gray_output.rows;
      int no_columns = gray_output.cols;
      cv::Mat bottom_part = gray_output(cv::Range(no_rows/2+200, no_rows), cv::Range(0, no_columns));
      
      
      double nonzero = cv::countNonZero(bottom_part);
      double total = bottom_part.total() ;

      double minval;
      double maxval;
      cv::Point minloc;
      cv::Point maxloc;
      cv::minMaxLoc(bottom_part, &minval, &maxval, &minloc, &maxloc);

      char condi = 'f';

      if (prev_max <= maxval){
        prev_max = maxval;
      }
      else {
        condi = 't';
      }

      cv::Point org(11, 50);
      string s = to_string(maxval);
      cv::putText(filtered_RGB, s, org,cv::FONT_HERSHEY_SCRIPT_SIMPLEX,1, cv::Scalar(255, 255 , 255), 2, cv::LINE_AA);
      
      ImageRef *ir = display->imageNew(im_width,im_height,img1.data,Display::RGBA);
      
      if (condi=='t'){
        condition = 't';
        if (colour_path == 'w'){
                
                if (given_color=='r'){
                      // robot->step(2250);
                      motors[0]-> setVelocity(0);
                      motors[1]-> setVelocity(0);
                      robot -> step (1000);
                      moveForward(0.5,7.5);
                      turnRobot(-90);
                      robot -> step (1000);
                      motors[0]-> setVelocity(MAX_SPEED);
                      motors[1]-> setVelocity(MAX_SPEED);
                      moveForward(1,11.7);
                      motors[0]-> setVelocity(0);
                      motors[1]-> setVelocity(0);
                      robot -> step(1000);
                      turnRobot(45);
                      robot -> step(1000);
                }
                else if (given_color=='b'){
                      
                      motors[0]-> setVelocity(0);
                      motors[1]-> setVelocity(0);
                      robot->step(1000);
                      moveForward(0.5,7.5);
                      robot -> step (500);
                      motors[0]-> setVelocity(MAX_SPEED);
                      motors[1]-> setVelocity(MAX_SPEED);
                      moveForward(1,11.7);
                      motors[0]-> setVelocity(0);
                      motors[1]-> setVelocity(0);
                      robot -> step(1000);
                      turnRobot(-45);
                      robot -> step(1000);
                }
        }
        else{
          motors[0]-> setVelocity(0);
          motors[1]-> setVelocity(0);
        }
      }
  
      display->imagePaste(ir,0,0,false);
      display->imageDelete(ir);
    }
   }
   cout << "chamith" << endl;
}

void Line_following_before_ball_shooting()
{
  int hmin = 115, smin = 100, vmin = 20;
  int hmax = 125, smax = 255, vmax = 255;

  const unsigned char *cam_feed;
  int im_height = 640;
  int im_width = 640;

  while (robot->step(timeStep) != -1){
   
  double line_sensor_array[8];     //Line follow sensor values
  for (int i=0;i<8;i++)
  {
    line_sensor_array[i]=  Line_folow_distance_sensor[i]->getValue();
  }


  //creating weighted, line follow array and calculating error (avarage)
  float weighted_array[8];        // [past error, present error]
  float sum =0;                   // to get the average (for error)
  int detected_lf_sensor_count =0;             // number sensors above line (to get the average) 
  line_follow_error_array[0]=line_follow_error_array[1];  
  for (int i=0; i<8; i++)
  {
    if (line_sensor_array[i]>900)  // sensor is not above the line
    { weighted_array[i]=0; }
    else                          // sensor is above the line
    { weighted_array[i]=i-3.5;      // weights---> [-3.5,-2.5,-1.5,-0.5,0.5,1.5,2.5,3.5]
      sum = sum + (weighted_array[i]);
      detected_lf_sensor_count = detected_lf_sensor_count + 1;
    }
  }
  if (detected_lf_sensor_count >0){
  line_follow_error_array[1] = sum/detected_lf_sensor_count;}
  else{
  line_follow_error_array[1] =0;}
  line_follow_error_array[2] = line_follow_error_array[2] + line_follow_error_array[1]; //updating the error summation

  PID(line_follow_error_array,0);
  
  //go afer PID
  float speed =1;
  if (controller_speed>0)
  {
    motors[0] -> setVelocity(speed * MAX_SPEED + controller_speed);
    motors[1] -> setVelocity(speed * MAX_SPEED - 2*controller_speed); 
  }
  else{
    motors[0] -> setVelocity(speed * MAX_SPEED + 2*controller_speed);
    motors[1] -> setVelocity(speed * MAX_SPEED - controller_speed); 
  }

  std::cout <<"array"<<endl;
  // int count = 0;
  // float thresh = 500;
  //   for (int i = 0; i <8 ; i++) 
  // {

  //   if(line_sensor_array[i] > thresh){
  //     count += 1;
  //   }
  //   std::cout << i<< "  " << line_sensor_array[i];
  //   //std::cout << line_sensor_array[i]<<endl;
  // }
  // cout << endl;  //done printing


  // if (count ==8){
  //   motors[0] -> setVelocity(0);
  //   motors[1] -> setVelocity(0); 
  //   //moveForward(1,3.5);
  //   break;
  // }




  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cv::Mat cvImage = cv::Mat(cv::Size(im_height, im_width), CV_8UC4);
  cv::Mat imgCopy;
  cv::Mat imgGray, imgBlur, imgDil;
  cv::Mat imgCanny;
  cv::Mat imgHsv, mask, res;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cam_feed = camera->getImage();

  if (cam_feed){

  cvImage.data = (uchar *)cam_feed;
  cv::cvtColor(cvImage, imgHsv, cv::COLOR_BGR2HSV);
  cv::Scalar lowerLim(hmin, smin, vmin);
  cv::Scalar upperLim(hmax, smax, vmax);
  cv::inRange(imgHsv, lowerLim, upperLim, mask);
  cv::bitwise_and(cvImage, cvImage, res, mask = mask);
  cv::cvtColor(res, imgGray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(imgGray, imgBlur, cv::Size(3, 3), 3, 0);
  cv::Canny(imgBlur, imgCanny, 25, 75);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  // cv::erode(imgCanny, imgCanny, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::dilate(imgCanny, imgDil, kernel);
  cv::findContours(imgDil, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<vector<cv::Point>> conPoly(contours.size());
  vector<cv::Rect> boundRect(contours.size());

  for (long long unsigned int i = 0; i < contours.size(); i++)
  {
    int area = cv::contourArea(contours[i]);
    string objectType;

    if (area > 500)
    {
      float peri = cv::arcLength(contours[i], true);
      cv::approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
      boundRect[i] = cv::boundingRect(conPoly[i]);
      int objCor = (int)conPoly[i].size();


      if (objCor <= 7)
      {
        objectType = "BOX";
      }
      else if (objCor > 7)
      {
        objectType = "CYLY";
      }

      // cv::drawContours(res, conPoly, i, cv::Scalar(255, 255, 255), 2);
      // cv::rectangle(res, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(255, 0, 0), 3);
      // cv::putText(res, objectType, { boundRect[i].x,boundRect[i].y - 5 }, cv::FONT_HERSHEY_PLAIN,3, cv::Scalar(255, 255, 255),3);
      cv::drawContours(res, contours, -1, cv::Scalar(255, 255, 255), 1);


    }

 //Print sensor values (remove)
  }

  ImageRef *ir = display->imageNew(im_width, im_height, res.data, Display::BGRA);
  display->imagePaste(ir, 0, 0, false);
  display->imageDelete(ir);

  if (conPoly.size()>0){

  

  if (conPoly[0][2].y >= 375 && conPoly[0][3].y >= 375){


    motors[0] -> setVelocity(0);
    motors[1] -> setVelocity(0); 
    //cout << "Break"<<endl;
    //moveForward(1,5.5);
    break;

  }

  cout <<conPoly[0]<<endl;
  }
  }
}
}
// PID function
void PID(float error_array[3], int case_select)    // error_array= [past error, presesnt error, updated error summation], case_select--> 0-line follow, 1-maze, ...
{
  float P= error_array[1];
  float I= error_array[2];
  float D= P - error_array[0];

  float const_array[3];

  if (case_select==0)       //check for line follow
  {
    const_array[0] = line_follow_PID_constants[0];
    const_array[1] = line_follow_PID_constants[1];
    const_array[2] = line_follow_PID_constants[2];
  }
  /* else if (case_select==1)
  {
  }*/

  controller_speed = const_array[0]*P + const_array[1]*I + const_array[2]*D;
}
