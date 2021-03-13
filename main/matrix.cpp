#include <iostream>
#include "matrix.hh"
using namespace std;

//matrix subtraction helper function
float * m_sub_helper(float * A, float * B, int elements) {
	float * C;
	C = new float[elements];
	for (int i = 0; i < elements; i++) {
		C[i] = A[i] - B[i];
	}
	return C;
}

//scalar multiplication helper function
float * s_mult_helper(float * A, float rho, int elements) {
	float * C;
	C = new float[elements];
	for (int i = 0; i < elements; i++) {
		C[i] = A[i] * rho;
	}
	return C;
}

//matrix multiplication helper function
float * m_mult_helper(float * A, float * B, int m, int n, int p) {
	float * C;
	C = new float[m*p];
	for (int i = 0;  i < m; i++)
	{
		for (int j = 0; j < p; j++)
		{
			C[i*p + j] = 0;
			for (int k = 0; k < n; k++) 
			{
				C[i*p + j] += A[i*n + k] * B[j + p * k];
			}
		}
	}
	return C;
}

//matrix addition helper function
float * m_add_helper(float * A, float * B, int elements) {
	float * C;
	C = new float[elements];
	for (int i = 0; i < elements; i++) {
		C[i] = A[i] + B[i];
	}
	return C;
}

//matrix multiplication
Matrix Matrix::operator*(Matrix B)
{
    const int m=rows;
    const int p=B.columns;
    const int n=columns;
    float * new_values;
    if (columns==B.rows)
    {
        new_values=m_mult_helper(values, B.values, m, n, p);
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
    float * new_values;
    if ((m_a==m_b)&&(n_a==n_b))
    {
        new_values=m_add_helper(values, B.values, m_a*n_a);
    }
    return Matrix(m_a, n_a, new_values);
}

//scalar multiplication function
Matrix Matrix :: scale(float k)
{
    float * new_values;
    new_values=s_mult_helper(values, k, rows*columns);
    return Matrix(rows, columns, new_values);
}

//matrix subtraction function
Matrix Matrix::operator-(Matrix B)
{
    const int m_a=rows;
    const int n_b=B.columns;
    const int n_a=columns;
    const int m_b=B.rows;
    float * new_values;
    if ((m_a==m_b)&&(n_a==n_b))
    {
        new_values=m_sub_helper(values, B.values, m_a*n_a);
    }
    return Matrix(m_a, n_a, new_values);
}

void display_matrix(Matrix A)
{
    for (int i=0; i<A.rows; i++)
    {
        for (int j = 0; j < A.columns; j++)
        {
            cout << A.select(i+1,j+1) << "    ";
        }
        cout << "\n";
    }
}