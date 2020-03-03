#include <iostream>
#include "./include/ANN.h"
#include "./include/Robot.h"
#include "./include/EA.h"

int main()
{
    EA experiment;
    experiment.EAInitialization();
    cout<<"robot group size : "<<experiment.robot_group.size()<<endl;
//    for(int i = 0; i < experiment.robot_group.size(); i++)
//    {
//        cout << "robot id:\t"<<i<<"\tpos:\t"<< experiment.robot_group[i].robot.center_pose.x()
//        <<'\t'<<experiment.robot_group[i].robot.center_pose.y()
//        <<"\tspeed:\t"<<experiment.robot_group[i].robot.l_speed
//        <<'\t'<<experiment.robot_group[i].robot.r_speed<<endl;
//    }

    experiment.Envolution();
    experiment.FitnessAll();
    ANNRobot last_best = experiment.GetBestFitness();

    for(int i = 0; i < experiment.generation; i++)
    {
        cout<<"generation : " << i << "\t";
        experiment.Selection();
        experiment.Crossover();
        experiment.Mutation();
        experiment.Envolution();
        experiment.FitnessAll();
        ANNRobot now_best = experiment.GetBestFitness();
        if(now_best.controller.FirstWeightMatrix == last_best.controller.FirstWeightMatrix
           ||now_best.controller.SecondWeightMatrix == last_best.controller.SecondWeightMatrix)
        {
            cout<<"no improvment!!!!!!!!!!!"<<endl;
        }
    }

    std::cout << "END!" << std::endl;
    return 0;
}
