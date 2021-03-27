#ifndef __LOOP_DATA_HH__
#define __LOOP_DATA_HH__

#include "matrix.hh"
#include <vector>
using namespace std;

vector<float> initialize_6 {0, 0, 0, 0, 0, 0};
vector<float> initialize_4 {0, 0, 0, 0};
vector<float> initialize_2 {0, 0};

struct loop_data
{
    int mode;
    long int timestamp;
    float dt;
    Matrix x=Matrix(6, 1, initialize_6);
    Matrix y=Matrix(4, 1, initialize_4);
    Matrix u=Matrix(2, 1, initialize_2);
    float yaw;
    vector<float> theta_raw_0;
    vector<float> theta_raw_1;
}; 


#endif
