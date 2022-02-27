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
void slide();

char motorNames[2][15] = {"leftMotor", "rightMotor"};
double bearing{};
vector<int> objectCount{};
bool cylinderGrabbed = false;
float masterBearing{};

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

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (robot->step(timeStep) != -1)
  {
    turnArm(150);
    mossaicAreaFunction();

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

  int hmin = 26, smin = 50, vmin = 20;
  int hmax = 35, smax = 255, vmax = 255;

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

void gotoPurpleMat()
{

  const int CORRECTION_ERR = 2;
  const double DELTA_SPEED = 0.03;

  int hmin = 150, smin = 100, vmin = 20;
  int hmax = 155, smax = 255, vmax = 255;

  const unsigned char *cam_feed;
  int im_height = 640;
  int im_width = 640;

  int deltaY{};

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
        // cv::drawContours(res, contours, -1, cv::Scalar(255, 255, 255), 1);
      }

      // get maximum y values                    1 and 2nd
      cv::line(res, conPoly[0][1], conPoly[0][2], cv::Scalar(255, 0, 0), 3);
    }

    ImageRef *ir = display->imageNew(im_width, im_height, res.data, Display::BGRA);
    display->imagePaste(ir, 0, 0, false);
    display->imageDelete(ir);

    deltaY = conPoly[0][1].y - conPoly[0][2].y;

    if (deltaY < -1 * CORRECTION_ERR)
    {
      // setting the motor speed
      motors[0]->setVelocity((0.40 + DELTA_SPEED) * MAX_SPEED);
      motors[1]->setVelocity(0.40 * MAX_SPEED);
    }
    else if (deltaY > CORRECTION_ERR)
    {
      // setting the motor speed
      motors[0]->setVelocity(0.40 * MAX_SPEED);
      motors[1]->setVelocity((0.40 + DELTA_SPEED) * MAX_SPEED);
    }
    else
    {
      motors[0]->setVelocity(0.4 * MAX_SPEED);
      motors[1]->setVelocity(0.4 * MAX_SPEED);
    }

    // cout << deltaY << endl;

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

      // cout << currentAngle << endl;
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

  short centered_object {-1};

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
    }


    if (!object_centered)
    {

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
        centered_object = reigionData[objectID + 3];
        motors[0]->setVelocity(0.6 * MAX_SPEED);
        motors[1]->setVelocity(0.6 * MAX_SPEED);

        // cout << reigionData[objectID + 2] << "," << reigionData[objectID + 4] << endl;
      }
    } // end of object centering

    // cout << deltaX << "         " << reigionData[2] << endl;

    /////////////////////////  HERE REMEMBER TO EDIT 300 value (height when should be stopped)
    if (reigionData[objectID + 2] > 250)
    {
      cout << reigionData[objectID + 2] << "," << reigionData[objectID + 4] << endl;
      moveForward(0);
      break;
    }

  } // end of while loop


  return centered_object;
}// end of grab object function

void checkCylinder(){

  if (!cylinderGrabbed){
    float disatance = 7;

    turnRobot(-90);
    moveForward(1, disatance);
    turnRobot(90);
    moveForward(1, disatance);
    turnRobot(90);
    moveForward(-1, 5);
  }
  
}

void placeInHole(){
  double currentBearing = getBearing(compass->getValues()[0], compass->getValues()[2]); // this is the current bearing
  double roatational_angle = 180 - (currentBearing - masterBearing);

  if (roatational_angle >= 360){
    roatational_angle = roatational_angle - 360;
  }else if (roatational_angle < 0){
    roatational_angle = 360 - roatational_angle;
  }

  turnRobot(roatational_angle);
  moveForward(1, 15);
  turnRobot(90);
  moveForward(1, 20);







}

void mossaicAreaFunction()
{
  short objectID{};
  // goto purple mat
  gotoPurpleMat();

  // grab the first object
  objectID = gotoGrabObject();

  cout << "Object ready to grab -> " << objectID << endl;

  if (objectID == 1){
    checkCylinder();
    objectID = gotoGrabObject();
  }

  if (objectID == 0) cylinderGrabbed = true;

  turnArm(-150);
  slide();
  turnArm(90);
  //turnArm(-90);
  //turnArm(90);
  placeInHole();
  

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
    // cout << (abs(startPos - pos) / 3.142) * 180 << "      " << abs(angle) << endl;
    if ((abs(startPos - pos) / 3.142) * 180 > abs(angle))
    {
      hinge_mot->setVelocity(0);
      break;
    }

    // cout << basepos1->getValue() << "  ,  " << basepos2->getValue() << endl;
  }
}// end of arm tuurning

void slide()
{

  const float END_POS = 0.01;

  base1Mot->setVelocity(0.01); //done
  base2Mot->setVelocity(0.01);

  base1Mot -> setPosition(-END_POS); //done
  base2Mot -> setPosition(END_POS);
  

  int count = 0;
  double pos1 = basepos1->getValue();
  double pos2 = basepos2->getValue();
  double prevoiusPos1;
  double prevoiusPos2;

  while (robot->step(timeStep) != -1)
  {
    
    //cout << basepos1->getValue() << "       " << basepos2->getValue() << endl;
    // cout << 0.03*2 -abs(basepos1->getValue()) -abs(basepos2->getValue())<<endl;
    // if ( 0.03*2 -abs(basepos1->getValue()) -abs(basepos2->getValue()) == 0.05+0.001){
      // break;
    // }
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
  cout << "Loop Broke" << endl;
}