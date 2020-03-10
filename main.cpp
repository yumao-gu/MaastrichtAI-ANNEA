#include <iostream>
#include "./include/ANN.h"
#include "./include/Robot.h"
#include "./include/EA.h"
#include <fstream>

int main()
{
    EA<GridMap_Trapezoid> experiment;
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
//    ANNRobot<GridMap> last_best = experiment.GetBestControl();

    vector<double> ave_fitness,max_fitness;
    int stop_criteria = 0;
    double upper_limit = 0.0;
    ANNRobot<GridMap_Trapezoid> now_best;

    for(int i = 0; i < experiment.generation; i++)
    {
        bool verbose = false;

        cout<<"generation : " << i << "\t";
        experiment.Selection();
        experiment.Crossover();
        experiment.Mutation();
        experiment.Envolution();
        experiment.FitnessAll();
        ave_fitness.push_back(experiment.GetAveFitness());
        max_fitness.push_back(experiment.GetBestFitness());

        if(i%20 == 0)
        {
            verbose = true;
        }
        now_best = experiment.GetBestControl(verbose);

//        cout<<"now_best First\n"<<now_best.controller.FirstWeightMatrix<<endl;


//        if(now_best.controller.FirstWeightMatrix == last_best.controller.FirstWeightMatrix
//           ||now_best.controller.SecondWeightMatrix == last_best.controller.SecondWeightMatrix)
//        {
//            cout<<"no improvment!!!!!!!!!!!"<<endl;
//        }

        if(1.05 * experiment.GetBestFitness() < upper_limit)
        {
            stop_criteria++;
        }
        else
        {
            stop_criteria = 0;
            upper_limit = 1.05 * experiment.GetBestFitness();
        }

        if(stop_criteria > 30)
        {
            break;
        }
    }

    experiment.GetBestControl(true);

    ofstream output1("/home/lenovo/avefitness.txt");
    for(int i = 0; i < ave_fitness.size();i++)
    {
        output1 << ave_fitness[i]<<endl;
    }
    output1.close();

    ofstream output2("/home/lenovo/maxfitness.txt");
    for(int i = 0; i < ave_fitness.size();i++)
    {
        output2 << max_fitness[i]<<endl;
    }
    output2.close();


    std::cout << "TEST!" << std::endl;
    EA<GridMap_Trapezoid> test;
    test.EAInitialization();
    test.robot_group.clear();
//    cout<<"test_robot First\n"<<now_best.controller.FirstWeightMatrix<<endl;
    for(int i = 0; i < test.population;i++)
    {
        ANNRobot<GridMap_Trapezoid> test_robot;
        test_robot.controller.FirstWeightMatrix = now_best.controller.FirstWeightMatrix;
        test_robot.controller.SecondWeightMatrix = now_best.controller.SecondWeightMatrix;
        test.robot_group.push_back(test_robot);
    }
    test.Envolution();
    test.FitnessAll();
    test.GetBestControl(true);
    std::cout << "END TEST!" << std::endl;

    std::cout << "END!" << std::endl;
    return 0;
}
