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
                         DPoint(this->init_pose.x() + this->length * cos(this->angle),this->init_pose.y() + this->length * sin(this->angle)));

    std::list<DPoint> lstPoints;
    bg::intersection(detect_line, wall, lstPoints);

    if(!lstPoints.empty())
    {
        return this->data = bg::distance(lstPoints.back(), DPoint(this->init_pose.x(),this->init_pose.y()));
    }
    else
    {
        return this->data = 200.0;
    }
}

Robot::Robot(Vector2d cp,double ang):center_pose(cp),direction(ang){}

//Receive keyboard information to control robot speed
void Robot::SpeedControl(char order)
{
    switch(order)
    {
        case 'W':
            this->l_speed = min(this->v_bound,this->l_speed + this->speed_step * this->delta_t);
            break;
        case 'S':
            this->l_speed = max(-this->v_bound,this->l_speed - this->speed_step * this->delta_t);
            break;
        case 'O':
            this->r_speed = min(this->v_bound,this->r_speed + this->speed_step * this->delta_t);
            break;
        case 'L':
            this->r_speed = max(-this->v_bound,this->r_speed - this->speed_step * this->delta_t);
            break;
        case 'X':
            this->l_speed = 0.0;
            this->r_speed = 0.0;
            break;
        case 'T':
            this->l_speed = min(this->v_bound,this->l_speed + this->speed_step * this->delta_t);
            this->r_speed = min(this->v_bound,this->r_speed + this->speed_step * this->delta_t);
            break;
        case 'G':
            this->l_speed = max(-this->v_bound,this->l_speed - this->speed_step * this->delta_t);
            this->r_speed = max(-this->v_bound,this->r_speed - this->speed_step * this->delta_t);
            break;
        default:
            break;
    }
}

//robot movement update
void Robot::Move(vector<DSegment> virtual_wall_set)
{
    this->diff_l_r += abs(this->l_speed - this->r_speed);
    this->path.push_back(this->center_pose);
//    cout << "BEFORE :\tl_speed : " <<  this->l_speed << "\tr_speed : " <<  this->r_speed
//         << "\tdirection : " <<  this->direction << "\n" << this->center_pose << endl;
    if(this->l_speed == 0 && this->r_speed == 0)
    {
        return;
    }
    else if(this->l_speed + this->r_speed == 0)
    {
        this->OmegaCalculate();
        this->direction += this->omega * this->delta_t;
        return;
    }
    // to use center pose
    double forward_distance = (this->l_speed + this->r_speed) / 2 * this->delta_t;
    DPoint forward_point(this->center_pose.x() + forward_distance * cos(this->direction),this->center_pose.y() + forward_distance * sin(this->direction));

    DSegment forward_seg(DPoint(this->center_pose.x(),this->center_pose.y()),forward_point);
//    DSegment forward_seg(backward_point,forward_point);
    double forward_angle = atan2((forward_seg.second.y() - forward_seg.first.y()),(forward_seg.second.x() - forward_seg.first.x()));
    int forward_sensor_id =abs((int)((forward_angle - this->direction)/PI*6)%12);
    std::list<DPoint> intersction_Points;
    double virtual_wall_angle;
    DSegment virtual_wall;
    for(vector<DSegment>::iterator it = virtual_wall_set.begin(); it != virtual_wall_set.end();it++)
    {
        virtual_wall = *it;
        virtual_wall_angle = atan((virtual_wall.second.y() - virtual_wall.first.y())/(virtual_wall.second.x() - virtual_wall.first.x()));
        bg::intersection(virtual_wall, forward_seg, intersction_Points);
        double ddistance = 100.0;
        ddistance = bg::distance(DPoint(center_pose.x(), center_pose.y()), virtual_wall);
        if(intersction_Points.empty() && ddistance < 0.01)
        {
            intersction_Points.push_back(DPoint(center_pose.x(), center_pose.y()));
        }

        if(!intersction_Points.empty())
        {
            virtual_wall_set.erase(it);
            break;
        }
    }

    if(!intersction_Points.empty() && this->sensors_data[forward_sensor_id] < 5 * ROBOT_RADIOS)
    {
//        cout<<"collision!!!!!!!!!!!!"<<endl;
//        cout << "BEFORE :\tl_speed : " <<  this->l_speed << "\tr_speed : " <<  this->r_speed
//             << "\tdirection : " <<  this->direction << "\n" << this->center_pose << endl;
        this->collision_times++;
        double collision_distance = bg::distance(intersction_Points.back(), DPoint(this->center_pose.x(), this->center_pose.y()));
        double collision_time = collision_distance / this->l_speed;
        double rest_time = this->delta_t - collision_time;
        double collision_speed = this->l_speed * cos(virtual_wall_angle - this->direction);
        this->center_pose = {intersction_Points.back().x(),intersction_Points.back().y()};
        Translation2d current_translation(collision_speed * rest_time * cos(virtual_wall_angle),collision_speed * rest_time * sin(virtual_wall_angle));
        Vector2d center_pose_2nd = current_translation * this->center_pose;
        if(this->l_speed != this->r_speed)
        {
            this->direction += this->omega * collision_time;
        }
        for(DSegment virtual_wall_2 : virtual_wall_set)
        {
            std::list<DPoint> intersction_Points_2nd;
            DSegment forward_seg_2nd(DPoint(center_pose.x(),center_pose.y()),DPoint(center_pose_2nd.x(),center_pose_2nd.y()));
            bg::intersection(virtual_wall_2, forward_seg_2nd, intersction_Points_2nd);
            if(bg::distance(forward_seg_2nd, virtual_wall_2) < 0.01)
            {
                bg::intersection(virtual_wall_2, virtual_wall, intersction_Points_2nd);
            }
            if(intersction_Points_2nd.empty())
            {
                continue;
            }
            else
            {
                center_pose_2nd = {intersction_Points_2nd.back().x(),intersction_Points_2nd.back().y()};
                break;
            }
        }
        this->center_pose = center_pose_2nd;
    }
    else
    {
        if(this->l_speed == this->r_speed)
        {
            Translation2d current_translation(this->l_speed * this->delta_t * cos(this->direction),this->l_speed * this->delta_t * sin(this->direction));
            this->center_pose = current_translation * this->center_pose;
        }
        else
        {
            RCalculate();
            OmegaCalculate();
            Translation2d ICC_to_center(this->R * cos(this->direction + 0.5 * PI),this->R * sin(this->direction + 0.5 * PI));
            this->ICC = ICC_to_center * this->center_pose;
            this->center_pose = Translation2d(this->ICC) * Rotation2Dd(this->omega * this->delta_t) * Translation2d(this->ICC).inverse() * this->center_pose;
            this->direction += this->omega * this->delta_t;
        }
    }

    if(this->direction >  PI)
    {
        this->direction -= 2*PI;
    }
    if(this->direction < -PI)
    {
        this->direction += 2*PI;
    }
}

//calculate the sensors data
void Robot::GetAllData(vector<DSegment> wall_set)
{
    for(int i = 0; i < 12; i++)
    {
        this->sensors.push_back(Sensor(this->direction + i * PI / 6,this->center_pose));
    }
//    cout<<"the sensors' data : ";
    for(int i = 0; i < 12; i++)
    {
        double min_sensor_data = 200.0;
        for(DSegment wall : wall_set)
        {
            min_sensor_data = min(min_sensor_data,sensors[i].GetData(wall));
        }

        //sensor output transform actual distance
//        min_sensor_data = 10 - 8 * (1 - exp(-min_sensor_data / 200.0));

        this->sensors_data.push_back(min_sensor_data);
//        cout << "\t" << min_sensor_data;
    }
//    cout<<endl;
}

//clear all data
void Robot::ClearData()
{
    this->sensors.clear();
    this->sensors_data.clear();
}
