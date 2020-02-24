#include <iostream>
#include "./include/ANN.h"
#include "./include/Robot.h"
#include "./include/EA.h"

int main()
{
    EA experiment;
    experiment.EAInitialization();
    cout<<"robot group size : "<<experiment.robot_group.size()<<endl;
    experiment.Envolution();
    experiment.FitnessAll();

    for(int i = 0; i < experiment.generation; i++)
    {
        cout<<"generation : " << i << "\t";
        experiment.Selection();
        experiment.Crossover();
        experiment.Mutation();
        experiment.Envolution();
        experiment.FitnessAll();
        experiment.GetBestFitness();
    }

    std::cout << "END!" << std::endl;
    return 0;
}
