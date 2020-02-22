//
// Created by lenovo on 2020/2/22.
//
#include "../include/Robot.h"

Sensor::Sensor(double angle,Vector2d init_pose)
{
    this->angle = angle;
    this->init_pose = init_pose;
}

double Sensor::GetData(DSegment wall)
{

    DSegment detect_line(DPoint(this->init_pose.x(),this->init_pose.y()),
                         DPoint(this->init_pose.x() + length * cos(angle),this->init_pose.y() + length * sin(angle)));

    std::list<DPoint> lstPoints;
    bg::intersection(detect_line, wall, lstPoints);

    if(!lstPoints.empty())
    {
        return this->data = bg::distance(lstPoints.back(), DPoint(init_pose.x(),init_pose.y()));
    }
    else
    {
        return this->data = 200.0;
    }
}

//Receive keyboard information to control robot speed
void Robot::SpeedControl(char order)
{
    switch(order)
    {
        case 'W':
            l_speed = min(v_bound,l_speed + speed_step);
            break;
        case 'S':
            l_speed = max(-v_bound,l_speed - speed_step);
            break;
        case 'O':
            r_speed = min(v_bound,r_speed + speed_step);
            break;
        case 'L':
            r_speed = max(-v_bound,r_speed - speed_step);
            break;
        case 'X':
            l_speed = 0.0;
            r_speed = 0.0;
            break;
        case 'T':
            l_speed = min(v_bound,l_speed + speed_step);
            r_speed = min(v_bound,r_speed + speed_step);
            break;
        case 'G':
            l_speed = max(-v_bound,l_speed - speed_step);
            r_speed = max(-v_bound,r_speed - speed_step);
            break;
        default:
            break;
    }
}

//robot movement update
void Robot::Move(vector<DSegment> virtual_wall_set)
{
    if(l_speed == 0 && r_speed == 0)
    {
        return;
    }
    else if(l_speed + r_speed == 0)
    {
        OmegaCalculate();
        direction += omega * delta_t;
        return;
    }
    // to use center pose
    double forward_distance = (l_speed + r_speed) / 2 * delta_t;
    DPoint forward_point(center_pose.x() + forward_distance * cos(direction),center_pose.y() + forward_distance * sin(direction));
    DSegment forward_seg(DPoint(center_pose.x(),center_pose.y()),forward_point);
    double forward_angle = atan2((forward_seg.second.y() - forward_seg.first.y()),(forward_seg.second.x() - forward_seg.first.x()));
    int forward_sensor_id =abs((int)((forward_angle - direction)/PI*6)%12);
    //cout<< "forward_sensor_id: " << forward_sensor_id <<endl;
    std::list<DPoint> intersction_Points;
    double virtual_wall_angle;
    for(vector<DSegment>::iterator it = virtual_wall_set.begin(); it != virtual_wall_set.end();it++)
    {
        DSegment virtual_wall = *it;
        virtual_wall_angle = atan((virtual_wall.second.y() - virtual_wall.first.y())/(virtual_wall.second.x() - virtual_wall.first.x()));
        bg::intersection(virtual_wall, forward_seg, intersction_Points);
        if(!intersction_Points.empty())
        {
            virtual_wall_set.erase(it);
            break;
        }
    }

    if(!intersction_Points.empty() && sensors_data[forward_sensor_id] < 5 * ROBOT_RADIOS)
    {
        double collision_distance = bg::distance(intersction_Points.back(), DPoint(center_pose.x(), center_pose.y()));
        double collision_time = collision_distance / l_speed;
        double rest_time = delta_t - collision_time;
        double collision_speed = l_speed * cos(virtual_wall_angle - direction);
        center_pose = {intersction_Points.back().x(),intersction_Points.back().y()};
        Translation2d current_translation(collision_speed * rest_time * cos(virtual_wall_angle),collision_speed * rest_time * sin(virtual_wall_angle));
        Vector2d center_pose_2nd = current_translation * center_pose;
        if(l_speed != r_speed)
        {
            direction += omega * collision_time;
        }
        for(DSegment virtual_wall : virtual_wall_set)
        {
            double virtual_wall_angle_2nd = atan((virtual_wall.second.y() - virtual_wall.first.y())/(virtual_wall.second.x() - virtual_wall.first.x()));
            std::list<DPoint> intersction_Points_2nd;
            DSegment forward_seg_2nd(DPoint(center_pose.x(),center_pose.y()),DPoint(center_pose_2nd.x(),center_pose_2nd.y()));
            bg::intersection(virtual_wall, forward_seg_2nd, intersction_Points_2nd);
//                cout << "forward_seg_2nd : " << "\t" << forward_seg_2nd.first.x() << "\t" << forward_seg_2nd.first.y()
//                << "\t" << forward_seg_2nd.second.x() << "\t" << forward_seg_2nd.second.y() << endl;
            if(intersction_Points_2nd.empty())
            {
                continue;
            }
            else
            {
                double collision_distance_2nd = bg::distance(intersction_Points_2nd.back(), DPoint(center_pose.x(), center_pose.y()));
                double collision_time_2nd = collision_distance_2nd / collision_speed;
                double rest_time_2nd = rest_time - collision_time_2nd;
                double collision_speed_2nd = collision_speed * cos(virtual_wall_angle_2nd - virtual_wall_angle);
                center_pose_2nd = {intersction_Points_2nd.back().x(),intersction_Points_2nd.back().y()};
                Translation2d current_translation_2nd(collision_speed_2nd * rest_time_2nd * cos(virtual_wall_angle_2nd),collision_speed_2nd * rest_time_2nd * sin(virtual_wall_angle_2nd));
                center_pose_2nd = current_translation_2nd * center_pose_2nd;
                break;
            }
        }
        center_pose = center_pose_2nd;
    }
    else
    {
        if(l_speed == r_speed)
        {
            Translation2d current_translation(l_speed * delta_t * cos(direction),l_speed * delta_t * sin(direction));
            center_pose = current_translation * center_pose;
        }
        else
        {
            RCalculate();
            OmegaCalculate();
            Translation2d ICC_to_center(R * cos(direction + 0.5 * PI),R * sin(direction + 0.5 * PI));
            ICC = ICC_to_center * center_pose;
            center_pose = Translation2d(ICC) * Rotation2Dd(omega * delta_t) * Translation2d(ICC).inverse() * center_pose;
            direction += omega * delta_t;
        }
    }

    if(direction >  PI)
    {
        direction -= 2*PI;
    }
    if(direction < -PI)
    {
        direction += 2*PI;
    }
}

//calculate the sensors data
void Robot::GetAllData(vector<DSegment> wall_set)
{
    for(int i = 0; i < 12; i++)
    {
        sensors.push_back(Sensor(direction + i * PI / 6,this->center_pose));
    }
    cout<<"the sensors' data : ";
    for(int i = 0; i < 12; i++)
    {
        double min_sensor_data = 200.0;
        for(DSegment wall : wall_set)
        {
            min_sensor_data = min(min_sensor_data,sensors[i].GetData(wall));
        }
        sensors_data.push_back(min_sensor_data);
        cout << "\t" << min_sensor_data;
    }
    cout<<endl;
}

//clear all data
void Robot::ClearData()
{
    sensors.clear();
    sensors_data.clear();
}
