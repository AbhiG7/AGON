#ifndef __LOOP_DATA_HH__
#define __LOOP_DATA_HH__

#include "matrix.hh"
#include <vector>
using namespace std;

struct loop_data
{
    int mode;
    long int time;
    float dt;
    Matrix x;
    Matrix y;
    Matrix u;
    float yaw;
    vector<float> theta_raw_0;
    vector<float> theta_raw_1;
}; 


#endif
