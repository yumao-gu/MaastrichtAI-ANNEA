//
// Created by lenovo on 2020/2/22.
//

#ifndef ANN_EA_ROBOT_ANN_H
#define ANN_EA_ROBOT_ANN_H

#include<assert.h>
#include<stdlib.h>
#include<iostream>
#include<string>
#include<cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

struct node
{
    double value = 0.0;
};

class Ann
{
public:
    Ann(int nNIL, int nNOL, int nNHL);
    Ann(const Ann & C);
    ~Ann();
    int numNodesInputLayer;
    int numNodesOutputLayer;
    int numNodesHiddenLayer;
    vector<double> sensorsData;
    MatrixXd FirstWeightMatrix,SecondWeightMatrix;
    vector<node> InputLayer,HiddenLayer,OutputLayer;

public:
    void EvolveOneStep(vector<double> sensors_data);

private:
    double sigmod(double x) { return 1 / (1 + exp(-1 * x)); }
    void ReadCurrentSensors(vector<double> sensors_data);               //read sensors data
    void RecordLastHiddenLayer();                                       //last hidden layer memory
    void TrainHiddenLayer();                                           //calculate the hidden layer
    void TrainOutputLayer();                                           //calculate the output layer
};

#endif //ANN_EA_ROBOT_ANN_H


