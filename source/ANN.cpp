//
// Created by lenovo on 2020/2/22.
//

#include "../include/ANN.h"
#include <ctime>
#include <random>

//random distribution
static default_random_engine eann((unsigned)time(0));
static normal_distribution<double> n_normal(0,1);
static uniform_real_distribution<double> n_uniform(-10,10);

Ann::Ann(int nNIL, int nNOL,int nNHL):numNodesInputLayer(nNIL), numNodesOutputLayer(nNOL),numNodesHiddenLayer(nNHL)
{
    //initialization
    //layer initialization
    this->InputLayer.resize(numNodesInputLayer + 1 + numNodesHiddenLayer);
    this->HiddenLayer.resize(numNodesHiddenLayer + 1);
    this->OutputLayer.resize(numNodesOutputLayer);

    //bias initialization
    this->InputLayer[numNodesInputLayer].value = 1.0;
    this->HiddenLayer[numNodesHiddenLayer].value = 1.0;

    //matrix initialization
    this->FirstWeightMatrix = MatrixXd::Zero(this->InputLayer.size(),this->HiddenLayer.size()- 1).unaryExpr([](double dummy){return n_uniform(eann);});
    this->SecondWeightMatrix = MatrixXd::Zero(this->HiddenLayer.size(),this->OutputLayer.size()).unaryExpr([](double dummy){return n_uniform(eann);});
}

Ann::Ann(const Ann & C)
{
    this->numNodesInputLayer = C.numNodesInputLayer;
    this->numNodesHiddenLayer = C.numNodesHiddenLayer;
    this->numNodesOutputLayer = C.numNodesOutputLayer;

    //initialization
    //layer initialization
    this->InputLayer.resize(numNodesInputLayer + 1 + numNodesHiddenLayer);
    this->HiddenLayer.resize(numNodesHiddenLayer + 1);
    this->OutputLayer.resize(numNodesOutputLayer);

    //bias initialization
    this->InputLayer[numNodesInputLayer].value = 1.0;
    this->HiddenLayer[numNodesHiddenLayer].value = 1.0;

    //matrix initialization
    this->FirstWeightMatrix = C.FirstWeightMatrix;
    this->SecondWeightMatrix = C.SecondWeightMatrix;
}

Ann::~Ann()
{
    this->InputLayer.clear();
    this->HiddenLayer.clear();
    this->OutputLayer.clear();
    this->FirstWeightMatrix.resize(0,0);
    this->SecondWeightMatrix.resize(0,0);
}

void Ann::ReadCurrentSensors(vector<double> sensors_data)
{
    for(int i = 0; i < sensors_data.size(); i++)
    {
        this->InputLayer[i].value = sensors_data[i];
    }
}

void Ann::RecordLastHiddenLayer()
{
    for(int i = 0; i < this->HiddenLayer.size() - 1; i++)
    {
        this->InputLayer[this->numNodesInputLayer + 1 + i].value = this->HiddenLayer[i].value;
    }
}

void Ann::TrainHiddenLayer()
{
    VectorXd input_layer_vector,hidden_layer_vector;
    input_layer_vector.resize(this->InputLayer.size());
    //hidden_layer_vector.resize(this->HiddenLayer.size() - 1);
    for(int i = 0; i < this->InputLayer.size(); i++)
    {
        input_layer_vector[i] = this->InputLayer[i].value;
    }

    hidden_layer_vector = (input_layer_vector.transpose() * this->FirstWeightMatrix).transpose();

    for(int i = 0; i < this->HiddenLayer.size() - 1; i++)
    {
        this->HiddenLayer[i].value = this->sigmod(hidden_layer_vector[i]);
    }
}

void Ann::TrainOutputLayer()
{
    VectorXd hidden_layer_vector,output_layer_vector;
    hidden_layer_vector.resize(this->HiddenLayer.size());
    output_layer_vector.resize(this->OutputLayer.size());
    for(int i = 0; i < this->HiddenLayer.size(); i++)
    {
        hidden_layer_vector[i] = this->HiddenLayer[i].value;
    }
    output_layer_vector = (hidden_layer_vector.transpose() * this->SecondWeightMatrix).transpose();

    for(int i = 0; i < this->OutputLayer.size(); i++)
    {
        this->OutputLayer[i].value = output_layer_vector[i];
    }
}

void Ann::EvolveOneStep(vector<double> sensors_data)
{
    this->ReadCurrentSensors(sensors_data);
    this->RecordLastHiddenLayer();
    this->TrainHiddenLayer();
    this->TrainOutputLayer();
}

