//
// Created by lenovo on 2020/2/23.
//

#ifndef ANN_EA_ROBOT_MAP_H
#define ANN_EA_ROBOT_MAP_H

#include "./Robot.h"

DSegment virtual_wallN(DPoint(0 + ROBOT_RADIOS,200 - ROBOT_RADIOS),DPoint(400 - ROBOT_RADIOS,200 - ROBOT_RADIOS));
DSegment virtual_wallS(DPoint(0 + ROBOT_RADIOS,0 + ROBOT_RADIOS),DPoint(400 - ROBOT_RADIOS,0 + ROBOT_RADIOS));
DSegment virtual_wallW(DPoint(0 + ROBOT_RADIOS,0 + ROBOT_RADIOS),DPoint(0 + ROBOT_RADIOS,200 - ROBOT_RADIOS));
DSegment virtual_wallE(DPoint(400 - ROBOT_RADIOS,0 + ROBOT_RADIOS),DPoint(400 - ROBOT_RADIOS,200 - ROBOT_RADIOS));
DSegment wallN(DPoint(0 ,200 ),DPoint(400,200 ));
DSegment wallS(DPoint(0 ,0 ),DPoint(400,0));
DSegment wallW(DPoint(0 ,0),DPoint(0 ,200));
DSegment wallE(DPoint(400 ,0),DPoint(400,200));

class GridMap
{
public:
    vector<DSegment> virtual_wall_set = {virtual_wallN,virtual_wallS,virtual_wallW,virtual_wallE};
    vector<DSegment> wall_set = {wallN,wallS,wallW,wallE};
    MatrixXd grid_map = MatrixXd::Zero(200,400);
};


#endif //ANN_EA_ROBOT_MAP_H
