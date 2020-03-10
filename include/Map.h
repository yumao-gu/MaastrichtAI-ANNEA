//
// Created by lenovo on 2020/2/23.
//

#ifndef ANN_EA_ROBOT_MAP_H
#define ANN_EA_ROBOT_MAP_H

#include "./Robot.h"

struct GridMap
{
    // training maps //
    vector<DSegment> virtual_wall_set = {DSegment(DPoint(0 + ROBOT_RADIOS,400 - ROBOT_RADIOS),DPoint(400 - ROBOT_RADIOS,400 - ROBOT_RADIOS)),
                                               DSegment(DPoint(0 + ROBOT_RADIOS,0 + ROBOT_RADIOS),DPoint(400 - ROBOT_RADIOS,0 + ROBOT_RADIOS)),
                                               DSegment(DPoint(0 + ROBOT_RADIOS,0 + ROBOT_RADIOS),DPoint(0 + ROBOT_RADIOS,400 - ROBOT_RADIOS)),
                                               DSegment(DPoint(400 - ROBOT_RADIOS,0 + ROBOT_RADIOS),DPoint(400 - ROBOT_RADIOS,400 - ROBOT_RADIOS))};
    vector<DSegment> wall_set = {
            DSegment(DPoint(0 ,400),DPoint(400,400)),
            DSegment(DPoint(0 ,0),DPoint(400,0)),
            DSegment(DPoint(0 ,0),DPoint(0 ,400)),
            DSegment(DPoint(400 ,0),DPoint(400,400))};

    void map_show(Mat& img)
    {
        line(img, Point(0 , 0), Point(0 , 400), Scalar(0, 255, 255), 3);
        line(img, Point(0 , 0), Point(400 , 0), Scalar(0, 255, 255), 3);
        line(img, Point( 0, 400), Point(400 , 400), Scalar(0, 255, 255), 3);
        line(img, Point( 400, 0), Point(400 , 400), Scalar(0, 255, 255), 3);
    }

    MatrixXd grid_map = MatrixXd::Zero(400,400);
};

struct GridMap_DoubleTrapeziod
{
    vector<DSegment> virtual_wall_set= {DSegment (DPoint(0+ ROBOT_RADIOS,0+ ROBOT_RADIOS),DPoint(0+ ROBOT_RADIOS,400-ROBOT_RADIOS)),
                                                     DSegment (DPoint(0+ ROBOT_RADIOS,0+ ROBOT_RADIOS),DPoint(300- ROBOT_RADIOS,100+ ROBOT_RADIOS)),
                                                     DSegment (DPoint(300-ROBOT_RADIOS,100+ROBOT_RADIOS),DPoint(300- ROBOT_RADIOS,300-ROBOT_RADIOS)),
                                                     DSegment (DPoint(0+ROBOT_RADIOS,400-ROBOT_RADIOS),DPoint(300-ROBOT_RADIOS,300-ROBOT_RADIOS)),
                                                     DSegment (DPoint(75+ROBOT_RADIOS,100-ROBOT_RADIOS),DPoint(200-ROBOT_RADIOS,150-ROBOT_RADIOS)),
                                                     DSegment (DPoint(75-ROBOT_RADIOS,100+ROBOT_RADIOS),DPoint(75-ROBOT_RADIOS,300-ROBOT_RADIOS)),
                                                     DSegment (DPoint(75+ROBOT_RADIOS,300+ROBOT_RADIOS),DPoint(200-ROBOT_RADIOS,250+ROBOT_RADIOS)),
                                                     DSegment (DPoint(200+ROBOT_RADIOS,250-ROBOT_RADIOS),DPoint(200+ROBOT_RADIOS,150+ROBOT_RADIOS))};
    vector<DSegment> wall_set = {DSegment (DPoint(0,0),DPoint(0,400)),
                                             DSegment (DPoint(0,0),DPoint(300,100)),
                                             DSegment (DPoint(300,100),DPoint(300,300)),
                                             DSegment (DPoint(0,400),DPoint(300,300)),
                                             DSegment (DPoint(75,100),DPoint(200,150)),
                                             DSegment (DPoint(75,100),DPoint(75,300)),
                                             DSegment (DPoint(75,300),DPoint(200,250)),
                                             DSegment (DPoint(200,250),DPoint(200,150))};
    MatrixXd grid_map = MatrixXd::Zero(200,400);
};

// testing maps //

struct GridMap_Trapezoid
{
    vector<DSegment> virtual_wall_set={
            DSegment (DPoint(0+ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(0+ROBOT_RADIOS,400-ROBOT_RADIOS)),
            DSegment (DPoint(0+ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(300-ROBOT_RADIOS,100+ROBOT_RADIOS)),
            DSegment (DPoint(300-ROBOT_RADIOS,100+ROBOT_RADIOS),DPoint(300-ROBOT_RADIOS,300-ROBOT_RADIOS)),
            DSegment (DPoint(0+ROBOT_RADIOS,400-ROBOT_RADIOS),DPoint(300-ROBOT_RADIOS,300-ROBOT_RADIOS))};
    vector<DSegment> wall_set ={
            DSegment (DPoint(0,0),DPoint(0,400)),
            DSegment (DPoint(0,0),DPoint(300,100)),
            DSegment (DPoint(300,100),DPoint(300,300)),
            DSegment (DPoint(0,400),DPoint(300,300))};
    void map_show(Mat& img)
    {
        line(img, Point(0 , 0), Point(0 , 400), Scalar(0, 255, 255), 3);
        line(img, Point(0 , 0), Point(300 , 100), Scalar(0, 255, 255), 3);
        line(img, Point( 300, 100), Point(300 , 300), Scalar(0, 255, 255), 3);
        line(img, Point( 0, 400), Point(300 , 300), Scalar(0, 255, 255), 3);
    }
    MatrixXd grid_map = MatrixXd::Zero(400,400);
};

struct GridMap_DoubleRectangle
{
    vector<DSegment> wall_set ={
            DSegment(DPoint(0 ,200),DPoint(400,200)),
            DSegment(DPoint(0 ,0),DPoint(400,0)),
            DSegment(DPoint(0 ,0),DPoint(0 ,200)),
            DSegment(DPoint(400 ,0),DPoint(400,200)),
            DSegment (DPoint(100,50),DPoint(100,150)),
            DSegment (DPoint(300,50),DPoint(300,150)),
            DSegment (DPoint(100,150),DPoint(300,150)),
            DSegment (DPoint(100,50),DPoint(300,50))};
    vector<DSegment> virtual_wall_set ={
            DSegment(DPoint(0+ROBOT_RADIOS,200-ROBOT_RADIOS),DPoint(400-ROBOT_RADIOS,200-ROBOT_RADIOS)),
            DSegment(DPoint(0+ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(400-ROBOT_RADIOS,0+ROBOT_RADIOS)),
            DSegment(DPoint(0+ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(0+ROBOT_RADIOS,200-ROBOT_RADIOS)),
            DSegment(DPoint(400-ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(400-ROBOT_RADIOS,200-ROBOT_RADIOS)),
            DSegment (DPoint(100-ROBOT_RADIOS,50+ROBOT_RADIOS),DPoint(100-ROBOT_RADIOS,150-ROBOT_RADIOS)),
            DSegment (DPoint(300+ROBOT_RADIOS,50+ROBOT_RADIOS),DPoint(300+ROBOT_RADIOS,150-ROBOT_RADIOS)),
            DSegment (DPoint(100+ROBOT_RADIOS,150+ROBOT_RADIOS),DPoint(300-ROBOT_RADIOS,150+ROBOT_RADIOS)),
            DSegment (DPoint(100+ROBOT_RADIOS,50-ROBOT_RADIOS),DPoint(300-ROBOT_RADIOS,50-ROBOT_RADIOS))};
    MatrixXd grid_map = MatrixXd::Zero(200,400);
};

struct GridMap_Room
{
    vector<DSegment> wall_set ={
            DSegment(DPoint(0 ,200),DPoint(400,200)),
            DSegment(DPoint(0 ,0),DPoint(400,0)),
            DSegment(DPoint(0 ,0),DPoint(0 ,200)),
            DSegment(DPoint(400 ,0),DPoint(400,200)),
            DSegment (DPoint(0,50),DPoint(100,50)),
            DSegment (DPoint(100,50),DPoint(100,100)),
            DSegment (DPoint(0,100),DPoint(100,100)),
            DSegment (DPoint(200,100),DPoint(200,150)),
            DSegment (DPoint(200,100),DPoint(250,100)),
            DSegment (DPoint(200,150),DPoint(250,150)),
            DSegment (DPoint(250,100),DPoint(250,150)),
            DSegment (DPoint(300,50),DPoint(350,50)),
            DSegment (DPoint(300,50),DPoint(300,100)),
            DSegment (DPoint(300,100),DPoint(350,100)),
            DSegment (DPoint(350,100),DPoint(350,50))};
    void map_show(Mat& img)
    {
        line(img, Point(0,50),Point(100,50), Scalar(0, 255, 255), 3);
        line(img, Point(100,50),Point(100,100), Scalar(0, 255, 255), 3);
        line(img,Point(0,100),Point(100,100), Scalar(0, 255, 255), 3);
        line(img,Point(200,100),Point(200,150), Scalar(0, 255, 255), 3);
        line(img, Point(200,100),Point(250,100), Scalar(0, 255, 255), 3);
        line(img, Point(200,150),Point(250,150), Scalar(0, 255, 255), 3);
        line(img,Point(250,100),Point(250,150), Scalar(0, 255, 255), 3);
        line(img,Point(300,50),Point(350,50), Scalar(0, 255, 255), 3);
        line(img, Point(300,50),Point(300,100), Scalar(0, 255, 255), 3);
        line(img,Point(300,100),Point(350,100), Scalar(0, 255, 255), 3);
        line(img,Point(350,100),Point(350,50), Scalar(0, 255, 255), 3);
        line(img, Point(0 ,200),Point(400,200), Scalar(0, 255, 255), 3);
        line(img, Point(0 ,0),Point(400,0), Scalar(0, 255, 255), 3);
        line(img, Point(0 ,0),Point(0 ,200), Scalar(0, 255, 255), 3);
        line(img, Point(400 ,0),Point(400,200), Scalar(0, 255, 255), 3);
    }
    vector<DSegment> virtual_wall_set ={
            DSegment(DPoint(0 +ROBOT_RADIOS,200-ROBOT_RADIOS),DPoint(400-ROBOT_RADIOS,200-ROBOT_RADIOS)),
            DSegment(DPoint(0+ROBOT_RADIOS ,0+ROBOT_RADIOS),DPoint(400-ROBOT_RADIOS,0+ROBOT_RADIOS)),
            DSegment(DPoint(0+ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(0+ROBOT_RADIOS,200-ROBOT_RADIOS)),
            DSegment(DPoint(400 -ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(400-ROBOT_RADIOS,200-ROBOT_RADIOS)),
            DSegment (DPoint(0+ROBOT_RADIOS,50-ROBOT_RADIOS),DPoint(100-ROBOT_RADIOS,50-ROBOT_RADIOS)),
            DSegment (DPoint(100+ROBOT_RADIOS,50+ROBOT_RADIOS),DPoint(100+ROBOT_RADIOS,100-ROBOT_RADIOS)),
            DSegment (DPoint(0+ROBOT_RADIOS,100+ROBOT_RADIOS),DPoint(100-ROBOT_RADIOS,100+ROBOT_RADIOS)),
            DSegment (DPoint(200-ROBOT_RADIOS,100+ROBOT_RADIOS),DPoint(200-ROBOT_RADIOS,150-ROBOT_RADIOS)),
            DSegment (DPoint(200+ROBOT_RADIOS,100-ROBOT_RADIOS),DPoint(250-ROBOT_RADIOS,100-ROBOT_RADIOS)),
            DSegment (DPoint(200+ROBOT_RADIOS,150+ROBOT_RADIOS),DPoint(250-ROBOT_RADIOS,150+ROBOT_RADIOS)),
            DSegment (DPoint(250+ROBOT_RADIOS,100+ROBOT_RADIOS),DPoint(250+ROBOT_RADIOS,150-ROBOT_RADIOS)),
            DSegment (DPoint(300+ROBOT_RADIOS,50-ROBOT_RADIOS),DPoint(350-ROBOT_RADIOS,50-ROBOT_RADIOS)),
            DSegment (DPoint(300-ROBOT_RADIOS,50+ROBOT_RADIOS),DPoint(300-ROBOT_RADIOS,100-ROBOT_RADIOS)),
            DSegment (DPoint(300+ROBOT_RADIOS,100+ROBOT_RADIOS),DPoint(350-ROBOT_RADIOS,100+ROBOT_RADIOS)),
            DSegment (DPoint(350+ROBOT_RADIOS,100-ROBOT_RADIOS),DPoint(350+ROBOT_RADIOS,50+ROBOT_RADIOS))};
    MatrixXd grid_map = MatrixXd::Zero(200,400);
};

struct GridMap_Star
{
    vector<DSegment> wall_set ={
            DSegment (DPoint(0,0),DPoint(0,200)),
            DSegment (DPoint(0,0),DPoint(200,200)),
            DSegment (DPoint(0,200),DPoint(200,200)),
            DSegment (DPoint(200,200),DPoint(200,0)),
            DSegment (DPoint(0,125),DPoint(75,125)),
            DSegment (DPoint(75,125),DPoint(100,200)),
            DSegment (DPoint(100,200),DPoint(125,125)),
            DSegment (DPoint(125,125),DPoint(200,125)),
            DSegment (DPoint(200,125),DPoint(125,75)),
            DSegment (DPoint(125,75),DPoint(150,0)),
            DSegment (DPoint(150,0),DPoint(100,40)),
            DSegment (DPoint(100,40),DPoint(0,50)),
            DSegment (DPoint(0,50),DPoint(75,75)),
            DSegment (DPoint(75,75),DPoint(0,125))};
    vector<DSegment> virtual_wall_set ={
            DSegment (DPoint(0+ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(0+ROBOT_RADIOS,200-ROBOT_RADIOS)),
            DSegment (DPoint(0+ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(200-ROBOT_RADIOS,0+ROBOT_RADIOS)),
            DSegment (DPoint(0+ROBOT_RADIOS,200-ROBOT_RADIOS),DPoint(200-ROBOT_RADIOS,200-ROBOT_RADIOS)),
            DSegment (DPoint(200-ROBOT_RADIOS,200-ROBOT_RADIOS),DPoint(200-ROBOT_RADIOS,0+ROBOT_RADIOS)),
            DSegment (DPoint(0+ROBOT_RADIOS,125-ROBOT_RADIOS),DPoint(75-ROBOT_RADIOS,125-ROBOT_RADIOS)),
            DSegment (DPoint(75+ROBOT_RADIOS,125+ROBOT_RADIOS),DPoint(100-ROBOT_RADIOS,200-ROBOT_RADIOS)),
            DSegment (DPoint(100+ROBOT_RADIOS,200-ROBOT_RADIOS),DPoint(125-ROBOT_RADIOS,125+ROBOT_RADIOS)),
            DSegment (DPoint(125+ROBOT_RADIOS,125-ROBOT_RADIOS),DPoint(200-ROBOT_RADIOS,125-ROBOT_RADIOS)),
            DSegment (DPoint(200-ROBOT_RADIOS,125-ROBOT_RADIOS),DPoint(125+ROBOT_RADIOS,75+ROBOT_RADIOS)),
            DSegment (DPoint(125+ROBOT_RADIOS,75-ROBOT_RADIOS),DPoint(150+ROBOT_RADIOS,0+ROBOT_RADIOS)),
            DSegment (DPoint(150-ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(100+ROBOT_RADIOS,40-ROBOT_RADIOS)),
            DSegment (DPoint(100-ROBOT_RADIOS,40-ROBOT_RADIOS),DPoint(50+ROBOT_RADIOS,0+ROBOT_RADIOS)),
            DSegment (DPoint(50-ROBOT_RADIOS,0+ROBOT_RADIOS),DPoint(75-ROBOT_RADIOS,75-ROBOT_RADIOS)),
            DSegment (DPoint(75-ROBOT_RADIOS,75+ROBOT_RADIOS),DPoint(0+ROBOT_RADIOS,125-ROBOT_RADIOS))};
    MatrixXd grid_map = MatrixXd::Zero(200,400);
};


#endif //ANN_EA_ROBOT_MAP_H
