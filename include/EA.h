//
// Created by lenovo on 2020/2/23.
//

#ifndef ANN_EA_ROBOT_EA_H
#define ANN_EA_ROBOT_EA_H

#include "./ANN.h"
#include "./Robot.h"
#include "./Map.h"

class ANNRobot
{
public:
    Robot robot = Robot({0,0},0.0);
    Ann controller = Ann(8,2,4);
    GridMap map;
    double fitness = 0.0;
public:
    ANNRobot();
    ANNRobot(const ANNRobot & C);
    void RunOneStep();
    void Fitness();
};

class EA
{
public:
    vector<ANNRobot> robot_group;
    vector<double> fitness_group;
    int generation = 100;
private:
    int population = 20;
    int max_envolution_steps = 100;
    int nth_max_selection_rank = 4;

public:
    void EAInitialization();
    void Selection();
    void FitnessAll();
    void Crossover();
    void PopulationUpdate();
    void Mutation();
    void Envolution();
    void ClearGeneration();
    ANNRobot GetBestFitness();
};



#endif //ANN_EA_ROBOT_EA_H
