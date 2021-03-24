#include "matrix.hh"
#include <string>
#include <vector>
using namespace std;

//matrix multiplication
Matrix Matrix::operator*(Matrix B)
{
    int m=rows;
    int p=B.columns;
    int n=columns;

    vector<float> new_values (m*p, 0);

    if (columns==B.rows)
    {
        for (int i = 0;  i < m; i++)
        {
            for (int j = 0; j < p; j++)
            {
                for (int k = 0; k < n; k++) 
                {
                    new_values[i*p + j] += values[i*n + k] * B.values[j + p * k];
                }
            }
        }
    }

    return Matrix(m, p, new_values);
}

//matrix addition function
Matrix Matrix::operator+(Matrix B)
{
    const int m_a=rows;
    const int n_b=B.columns;
    const int n_a=columns;
    const int m_b=B.rows;

    vector<float> new_values (m_a*n_a, 0);

    if ((m_a==m_b)&&(n_a==n_b))
    {
        for (int i = 0; i < m_a*n_a; i++) {
            new_values[i] = values[i] + B.values[i];
        }
    }

    return Matrix(m_a, n_a, new_values);
}

//scalar multiplication function
Matrix Matrix :: scale(float k)
{
    vector<float> new_values (rows*columns, 0);
    for (int i = 0; i < rows*columns; i++) {
		new_values[i] = values[i] * k;
	}
    return Matrix(rows, columns, new_values);
}

//matrix subtraction function
Matrix Matrix::operator-(Matrix B)
{
    const int m_a=rows;
    const int n_b=B.columns;
    const int n_a=columns;
    const int m_b=B.rows;

    vector<float> new_values (m_a*n_a, 0);

    if ((m_a==m_b)&&(n_a==n_b))
    {
        for (int i = 0; i < m_a*n_a; i++) {
            new_values[i] = values[i] - B.values[i];
        }
    }

    return Matrix(m_a, n_a, new_values);
}

void Matrix:: redefine(vector<float> update_values)
{
    values=update_values;
}

/** for testing
int main()
{
    float a_values [9] ={1, 0, 1, 0, 1, 1, 1, 0, 1};
    float b_values [9] ={3, 0, 2, 2, 1, 3, 2, 2, -1};
    Matrix A=Matrix(3, 3, a_values);
    Matrix B=Matrix(3, 3, b_values);
    
    float d_values [6]= {-3, -10, 7, 2, 5, 0};
    Matrix D=Matrix(3, 2, d_values);
    Matrix E=(A+B)*D;
    display_matrix(E);

    return 0;
}
*/
