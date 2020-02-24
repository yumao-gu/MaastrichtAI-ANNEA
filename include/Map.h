//
// Created by lenovo on 2020/2/23.
//

#ifndef ANN_EA_ROBOT_MAP_H
#define ANN_EA_ROBOT_MAP_H

#include "./Robot.h"

struct GridMap
{
    vector<DSegment> virtual_wall_set = {DSegment(DPoint(0 + ROBOT_RADIOS,200 - ROBOT_RADIOS),DPoint(400 - ROBOT_RADIOS,200 - ROBOT_RADIOS)),
                                         DSegment(DPoint(0 + ROBOT_RADIOS,0 + ROBOT_RADIOS),DPoint(400 - ROBOT_RADIOS,0 + ROBOT_RADIOS)),
                                         DSegment(DPoint(0 + ROBOT_RADIOS,0 + ROBOT_RADIOS),DPoint(0 + ROBOT_RADIOS,200 - ROBOT_RADIOS)),
                                         DSegment(DPoint(400 - ROBOT_RADIOS,0 + ROBOT_RADIOS),DPoint(400 - ROBOT_RADIOS,200 - ROBOT_RADIOS))};
    vector<DSegment> wall_set = {DSegment(DPoint(0 ,200),DPoint(400,200)),
                                 DSegment(DPoint(0 ,0),DPoint(400,0)),
                                 DSegment(DPoint(0 ,0),DPoint(0 ,200)),
                                 DSegment(DPoint(400 ,0),DPoint(400,200))};
    MatrixXd grid_map = MatrixXd::Zero(200,400);
};


#endif //ANN_EA_ROBOT_MAP_H
