//
// Created by lenovo on 2020/2/23.
//

#include "../include/EA.h"
#include  <omp.h>

static default_random_engine e((unsigned)time(0));
static uniform_real_distribution<double> n_uniform_x(0,400);
static uniform_real_distribution<double> n_uniform_y(0,200);
static uniform_real_distribution<double> n_uniform_ang(-PI,PI);

ANNRobot::ANNRobot()
{
    //random distribution
    this->robot = Robot({n_uniform_x(e),n_uniform_y(e)},n_uniform_ang(e));
}

ANNRobot::ANNRobot(const ANNRobot & C)
{
    //random distribution
    this->robot = Robot({n_uniform_x(e),n_uniform_y(e)},n_uniform_ang(e));
    this->controller = Ann(C.controller);
}

void ANNRobot::RunOneStep()
{
    this->robot.ClearData();
    this->robot.GetAllData(this->map.wall_set);
    this->controller.EvolveOneStep(this->robot.sensors_data);
    this->robot.Move(this->map.virtual_wall_set);
    this->map.grid_map.block((int)this->robot.center_pose.x() - 20,
            (int)this->robot.center_pose.y() - 20,40,40).setOnes();
}

void ANNRobot::Fitness()
{
    this->fitness = this->map.grid_map.sum() - this->robot.collision_times * 1600.0;
}

void EA::EAInitialization()
{
# pragma omp parallel for
    for(int i = 0; i < this->population; i++)
    {
        ANNRobot individual = ANNRobot();
        this->robot_group.push_back(individual);
    }
}

void EA::FitnessAll()
{
# pragma omp parallel for
    for(int i = 0; i < this->robot_group.size(); i++)
    {
        this->robot_group[i].Fitness();
        this->fitness_group[i] = this->robot_group[i].fitness;
    }
}

void EA::Envolution()
{
# pragma omp parallel for
    for(int i = 0; i < this->robot_group.size(); i++)
    {
        for(int j = 0; j < this->max_envolution_steps;j++)
        {
            this->robot_group[i].RunOneStep();
        }
    }
}

void EA::Selection()
{
    nth_element(this->fitness_group.begin(), this->fitness_group.begin() + this->nth_max_selection_rank,
     this->fitness_group.end(), std::greater<double>());
    double nth_max_selection_rank_fitness = this->fitness_group[this->nth_max_selection_rank];
    vector<ANNRobot> robot_group_tmp;

# pragma omp parallel for
    for(int i = 0; i < this->robot_group.size(); i++)
    {
        if(this->robot_group[i].fitness > nth_max_selection_rank_fitness)
        {
            robot_group_tmp.push_back(ANNRobot(this->robot_group[i]));
        }
    }

    this->ClearGeneration();
    this->robot_group = robot_group_tmp;
    robot_group_tmp.clear();
}

void EA::ClearGeneration()
{
    this->fitness_group.clear();
    this->robot_group.clear();
}

void EA::Crossover()
{
# pragma omp parallel for
    for(int i = 0; i < (this->population - this->nth_max_selection_rank) / 2; i++)
    {
        uniform_int_distribution<int> n_uniform_int(0,this->nth_max_selection_rank - 1);
        int parent1 = n_uniform_int(e);
        int parent2 = n_uniform_int(e);
        int first_weight_size = this->robot_group[parent1].controller.FirstWeightMatrix.size();
        int second_weight_size = this->robot_group[parent1].controller.SecondWeightMatrix.size();
        int weight_size = this->robot_group[parent2].controller.FirstWeightMatrix.size() + this->robot_group[parent2].controller.SecondWeightMatrix.size();

        double* p1_firstweight = this->robot_group[parent1].controller.FirstWeightMatrix.data();
        double* p1_secondweight = this->robot_group[parent1].controller.SecondWeightMatrix.data();
        double* p1_weight = new double[weight_size];
        for(int i = 0; i < first_weight_size;i++)
        {
            p1_weight[i] = p1_firstweight[i];
        }
        for(int i = 0; i < second_weight_size;i++)
        {
            p1_weight[i + first_weight_size] = p1_secondweight[i];
        }
        double* p2_firstweight = this->robot_group[parent2].controller.FirstWeightMatrix.data();
        double* p2_secondweight = this->robot_group[parent2].controller.SecondWeightMatrix.data();
        double* p2_weight = new double[weight_size];
        for(int i = 0; i < first_weight_size;i++)
        {
            p2_weight[i] = p2_firstweight[i];
        }
        for(int i = 0; i < second_weight_size;i++)
        {
            p2_weight[i + first_weight_size] = p2_secondweight[i];
        }

        uniform_real_distribution<double> n_uniform_method(0,1);
        double method = n_uniform_method(e);
        if(method < 0.1)
        {
            uniform_int_distribution<int> n_uniform_int(0,weight_size);

        }
        else if(method < 0.55)
        {

        }
        else
        {

        }

    }

}

void weight_swap()



