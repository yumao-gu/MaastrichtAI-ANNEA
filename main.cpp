#include <iostream>
#include "./include/ANN.h"
#include "./include/Robot.h"

int main()
{
    Ann ann(8,2,4);
    vector<double> sensors_data = {1.,2.,3.,4.,5.,6.,7.,8.};
    for(int i = 0;i < 2; i++)
    {
        ann.EvolveOneStep(sensors_data);
    }
    cout<<"HiddenLayer\n";
    for(int i = 0; i < ann.HiddenLayer.size(); i++)
    {
        cout<< "\t" << ann.HiddenLayer[i].value <<endl;
    }
    cout<<endl;

    std::cout << "END!" << std::endl;
    return 0;
}
