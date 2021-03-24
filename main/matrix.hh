#ifndef __MATRIX_HH__
#define __MATRIX_HH__

#include <cstdbool>
#include <cstdint>
#include <string>
#include <vector>
using namespace std;


class Matrix
{
    public:
        int rows;
        int columns;
        vector<float> values;
        
        Matrix(int row_number, int column_number, vector<float> mat_values)
        {
            rows=row_number;
            columns=column_number;
            values=mat_values;
        }

        float select(int row, int column)
        {
            return values[(row-1)*columns+(column-1)];
        }

        void redefine(vector<float>);
        Matrix operator+(Matrix);
        Matrix operator-(Matrix);
        Matrix scale (float);
        Matrix operator*(Matrix);
};

#endif
