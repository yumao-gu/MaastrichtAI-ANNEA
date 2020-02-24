//
// Created by lenovo on 2020/2/23.
//

#include "../include/EA.h"
#include  <omp.h>

static default_random_engine e((unsigned)time(0));
static uniform_real_distribution<double> n_uniform_x(50,350);
static uniform_real_distribution<double> n_uniform_y(50,150);
static uniform_real_distribution<double> n_uniform_ang(-PI,PI);

void weight_swap(double* a1, double* a2,int length ,int pos)
{
    double* tmp = new double[length];
    for(int i = 0; i < length - pos; i++)
    {
        tmp[i + pos] = a1[i + pos];
    }
    for(int i = 0; i < length - pos; i++)
    {
        a1[i + pos] = a2[i + pos];
    }
    for(int i = 0; i < length - pos; i++)
    {
        a2[i + pos] = tmp[i + pos];
    }
    delete tmp;
}

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
    this->robot.l_speed = this->controller.OutputLayer[0].value;
    this->robot.l_speed = max(-this->robot.v_bound,min(this->robot.v_bound,this->robot.l_speed));
    this->robot.r_speed = this->controller.OutputLayer[1].value;
    this->robot.r_speed = max(-this->robot.v_bound,min(this->robot.v_bound,this->robot.r_speed));
    this->robot.Move(this->map.virtual_wall_set);
    this->map.grid_map.block(min(159,max(0,(int)this->robot.center_pose.y() - 20)),
                             min(159,max(0,(int)this->robot.center_pose.x() - 20)),40,40).setOnes();
}

void ANNRobot::Fitness()
{
    this->fitness = this->map.grid_map.sum() / 100.0 - this->robot.collision_times;
}

void EA::EAInitialization()
{
    for(int i = 0; i < this->population; i++)
    {
        ANNRobot individual = ANNRobot();
        this->robot_group.push_back(individual);
        //cout << "robot ； " << i << "\tSecond Matrix : \n" <<  individual.controller.SecondWeightMatrix<<endl;
    }
}

void EA::FitnessAll()
{
    this->fitness_group.resize(this->robot_group.size());
# pragma omp parallel for
    for(int i = 0; i < this->robot_group.size(); i++)
    {
        this->robot_group[i].Fitness();
        this->fitness_group[i] = this->robot_group[i].fitness;
//        cout << "robot : " << i << "\tfitness : " << this->robot_group[i].fitness
//        << "\tcollision times : " << this->robot_group[i].robot.collision_times
//        << "\taverage speed : " << (this->robot_group[i].robot.l_speed + this->robot_group[i].robot.r_speed) / 2 << endl;
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
//            cout << "robot : " << i << "\tsteps : " << j
//                 << "\tl_speed : " <<  this->robot_group[i].robot.l_speed << "\tr_speed : " <<  this->robot_group[i].robot.r_speed
//                 << "\tdirection : " <<  this->robot_group[i].robot.direction << "\n" << this->robot_group[i].robot.center_pose << endl;
        }
        //cout << "robot : " << i << "\tgrid map" << this->robot_group[i].map.grid_map.sum() << endl;
    }
}

void EA::Selection()
{
    nth_element(this->fitness_group.begin(), this->fitness_group.begin() + this->nth_max_selection_rank,
     this->fitness_group.end(), std::greater<double>());
    double nth_max_selection_rank_fitness = this->fitness_group[this->nth_max_selection_rank];
    vector<ANNRobot> robot_group_tmp;

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

//    for(int i = 0; i < this->robot_group.size(); i++)
//    {
//        cout << "robot : " << i << "\tSecondWeightMatrix : \n" << this->robot_group[i].controller.SecondWeightMatrix <<endl;
//    }
}

void EA::ClearGeneration()
{
    this->fitness_group.clear();
    this->robot_group.clear();
}

void EA::Crossover()
{
//# pragma omp parallel for
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
            //cout<<"method1"<<endl;
            uniform_int_distribution<int> n_uniform_int(0,weight_size);
            int pos = n_uniform_int(e);
            weight_swap(p1_weight,p2_weight,weight_size,pos);
        }
        else if(method < 0.55)
        {
            //cout<<"method2"<<endl;
            uniform_int_distribution<int> n_uniform_int2(0,10);
            int j_size = n_uniform_int(e);
            for(int j = 0; j < j_size; j++)
            {
                uniform_int_distribution<int> n_uniform_int(0,weight_size);
                int pos = n_uniform_int(e);
                weight_swap(p1_weight,p2_weight,weight_size,pos);
            }
        }
        else
        {
            //cout<<"method3"<<endl;
            for(int i = 0; i < weight_size; i++)
            {
                p1_weight[i] = (2 * p1_weight[i] + p2_weight[i]) / 3.0;
                p2_weight[i] = (p2_weight[i] + p1_weight[i]) / 2.0;
            }
        }

        ANNRobot child1;
        child1.controller.FirstWeightMatrix = Map<MatrixXd>(p1_weight,child1.controller.InputLayer.size(),child1.controller.HiddenLayer.size()- 1);
        child1.controller.SecondWeightMatrix = Map<MatrixXd>(&p1_weight[first_weight_size],child1.controller.HiddenLayer.size(),child1.controller.OutputLayer.size());

        ANNRobot child2;
        child2.controller.FirstWeightMatrix = Map<MatrixXd>(p2_weight,child2.controller.InputLayer.size(),child2.controller.HiddenLayer.size()- 1);
        child2.controller.SecondWeightMatrix = Map<MatrixXd>(&p2_weight[first_weight_size],child2.controller.HiddenLayer.size(),child2.controller.OutputLayer.size());

        this->robot_group.push_back(child1);
        this->robot_group.push_back(child2);

        delete[] p1_weight;
        delete[] p2_weight;

//        if((child1.controller.SecondWeightMatrix!=this->robot_group[parent1].controller.SecondWeightMatrix
//            ||child1.controller.SecondWeightMatrix!=this->robot_group[parent2].controller.SecondWeightMatrix)
//           && (method < 0.55))
//        {
//            cout<<"HAHAHAHAHAHAHAHAHA"<<endl;
//            cout << "parent1 : SecondWeightMatrix : \n" << this->robot_group[parent1].controller.SecondWeightMatrix <<endl;
//            cout << "parent2 : SecondWeightMatrix : \n" << this->robot_group[parent2].controller.SecondWeightMatrix <<endl;
//            cout << "child1 : SecondWeightMatrix : \n" << child1.controller.SecondWeightMatrix <<endl;
//            cout << "child2 : SecondWeightMatrix : \n" << child2.controller.SecondWeightMatrix <<endl;
//        }
    }

//    for(int i = 0; i < this->robot_group.size(); i++)
//    {
//        cout << "robot : " << i << "\tSecondWeightMatrix : \n" << this->robot_group[i].controller.SecondWeightMatrix <<endl;
//    }
}

void EA::PopulationUpdate()
{
    this->population = this->robot_group.size();
}

void EA::Mutation()
{
    this->PopulationUpdate();
    cout<<"population : "<<population<< "\t size : "<< this->robot_group.size()<<endl;
# pragma omp parallel for
    for(int i = this->nth_max_selection_rank; i < population; i++)
    {
        uniform_real_distribution<double> n_uniform_prob(0,1);
        double mutaion_prob = n_uniform_prob(e);
        if(mutaion_prob < 0.8)
        {
            for(int j = 0; j < this->robot_group[i].controller.InputLayer.size(); j++)
            {
                for(int k = 0; k < this->robot_group[i].controller.HiddenLayer.size() - 1 ;k++)
                {
                    double unit_mutation_prob = n_uniform_prob(e);
//                    if(unit_mutation_prob > 0.95)
//                    {
//                        cout<<"before:\t"<<j<<"\t"<<k<<"\t"<<this->robot_group[i].controller.FirstWeightMatrix(j,k)<<endl;
//                    }
                    if(unit_mutation_prob > 0.975)
                    {
                        this->robot_group[i].controller.FirstWeightMatrix(j,k) *= 1.1;
                    }
                    else if(unit_mutation_prob > 0.95)
                    {
                        this->robot_group[i].controller.FirstWeightMatrix(j,k) *= 0.9;
                    }
//                    if(unit_mutation_prob > 0.95)
//                    {
//                        cout<<"after:\t"<<j<<"\t"<<k<<"\t"<<this->robot_group[i].controller.FirstWeightMatrix(j,k)<<endl;
//                    }
                }
            }

            for(int j = 0; j < this->robot_group[i].controller.HiddenLayer.size(); j++)
            {
                for(int k = 0; k < this->robot_group[i].controller.OutputLayer.size() ;k++)
                {
                    double unit_mutation_prob = n_uniform_prob(e);
                    if(unit_mutation_prob > 0.975)
                    {
                        this->robot_group[i].controller.SecondWeightMatrix(j,k) *= 1.1;
                    }
                    else if(unit_mutation_prob > 0.95)
                    {
                        this->robot_group[i].controller.SecondWeightMatrix(j,k) *= 0.9;
                    }
                }
            }
        }
    }
}

void EA::GetBestFitness()
{
    int best_id = 0;
    double best_fitness = 0.0;
    for(int i = 0; i < this->population; i++)
    {
        if(this->robot_group[i].fitness > best_fitness)
        {
            best_fitness = this->robot_group[i].fitness;
            best_id = i;
        }
    }
    cout<<"best id : " << best_id << "\tfitness : " << best_fitness <<"\tcollision times : "<<this->robot_group[best_id].robot.collision_times<<endl;
//    for(auto pos : this->robot_group[best_id].robot.path)
//    {
//        cout<<pos.x()<<"\t"<<pos.y()<<endl;
//    }
}





