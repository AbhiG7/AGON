#ifndef __MATRIX_HH__
#define __MATRIX_HH__

#include <cstdbool>
#include <cstdint>
#include <string>
using namespace std;


class Matrix
{
    public:
        int rows;
        int columns;
        float * values;
        
        Matrix(int row_number, int column_number, float * mat_values)
        {
            rows=row_number;
            columns=column_number;
            values=mat_values;
        }

        float select(int row, int column)
        {
            return values[(row-1)*columns+(column-1)];
        }

        void redefine(float*);
        Matrix operator+(Matrix);
        Matrix operator-(Matrix);
        Matrix scale (float);
        Matrix operator*(Matrix);
};

string display_matrix(Matrix);

#endif  // __MODING_HH__
