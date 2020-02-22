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
#define ROBOT_RADIOS 10

using namespace std;
using namespace Eigen;
using namespace cv;
namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> DPoint;
typedef bg::model::segment<DPoint> DSegment;

class Sensor
{
public:
    explicit Sensor(double angle,Vector2d init_pose);

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
    double l_speed = 0.0,r_speed = 0.0;
    double direction = 0.0;                 //angle to x-axis
    vector<Sensor> sensors;
    vector<double> sensors_data;

private:
    double v_bound = 3.0;                   //max speed bound
    double speed_step = 0.1;               //Speed increase step
    double l = 8;                         //the distance of the two wheels
    double delta_t = 1;                   //time interval
    Vector2d ICC;                           //Instantaneou Center of Curvature
    double R = 0.0;                         //from ICC to center
    double omega = 0.0;                     //angle speed

public:
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
        this->R = 0.5 * (l_speed + r_speed) / (r_speed - l_speed);
    }

    //omega calculate
    void OmegaCalculate()
    {
        this->omega = (r_speed - l_speed) / l;
    }
};

#endif //ANN_EA_ROBOT_ROBOT_H
