//
// Created by lenovo on 2020/2/22.
//

#ifndef ANN_EA_ROBOT_ROBOT_H
#define ANN_EA_ROBOT_ROBOT_H

#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#define PI 3.1415926
#define ROBOT_RADIOS 25

using namespace std;
using namespace Eigen;
using namespace cv;
namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> DPoint;
typedef bg::model::segment<DPoint> DSegment;

class Sensor
{
public:
    Sensor(double angle,Vector2d init_pose);

public:
    double angle = 0.0;
    double length = 200.0;
    Vector2d init_pose = {0,0};
    double data = 200.0;

public:
    double GetData(DSegment wall);
};

class Robot
{
public:
    Vector2d center_pose = {0,0};
    vector<Vector2d> path;
    double l_speed = 0.0,r_speed = 0.0;
    double direction = 0.0;                 //angle to x-axis
    vector<Sensor> sensors;
    vector<double> sensors_data;
    int collision_times = 0;
    double delta_t = 0.3;                   //time interval
    double v_bound = 40.0;                   //max speed bound

private:
    double speed_step = 10.0;               //Speed increase step
    double l = 40;                         //the distance of the two wheels
    Vector2d ICC;                           //Instantaneou Center of Curvature
    double R = 0.0;                         //from ICC to center
    double omega = 0.0;                     //angle speed

public:
    Robot(Vector2d cp,double ang);

    //Receive keyboard information to control robot speed
    void SpeedControl(char order);

    //robot movement update
    void Move(vector<DSegment> virtual_wall_set);

    //calculate the sensors data
    void GetAllData(vector<DSegment> wall_set);

    //clear all data
    void ClearData();

private:
    //R calculate
    void RCalculate()
    {
        this->R = 0.5 * this->l * (this->l_speed + this->r_speed) / (this->r_speed - this->l_speed);
    }

    //omega calculate
    void OmegaCalculate()
    {
        this->omega = (this->r_speed - this->l_speed) / this->l;
    }
};

#endif //ANN_EA_ROBOT_ROBOT_H
