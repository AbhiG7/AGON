#include <iostream>
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
};

float * m_sub_helper(float * A, float * B, int elements) {
	float * C;
	C = new float[elements];
	for (int i = 0; i < elements; i++) {
		C[i] = A[i] - B[i];
	}
	return C;
}

float * s_mult_helper(float * A, float rho, int elements) {
	float * C;
	C = new float[elements];
	for (int i = 0; i < elements; i++) {
		C[i] = A[i] * rho;
	}
	return C;
}

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

float * m_add_helper(float * A, float * B, int elements) {
	float * C;
	C = new float[elements];
	for (int i = 0; i < elements; i++) {
		C[i] = A[i] + B[i];
	}
	return C;
}

Matrix mMult(Matrix A, Matrix B)
{
    const int m=A.rows;
    const int p=B.columns;
    const int n=A.columns;
    float * values;
    if (A.columns==B.rows)
    {
        values=m_mult_helper(A.values, B.values, m, n, p);
    }
    return Matrix(m, p, values);
}

Matrix mAdd(Matrix A, Matrix B)
{
    const int m_a=A.rows;
    const int n_b=B.columns;
    const int n_a=A.columns;
    const int m_b=B.rows;
    float * values;
    if ((m_a==m_b)&&(n_a==n_b))
    {
        values=m_add_helper(A.values, B.values, m_a*n_a);
    }
    return Matrix(m_a, n_a, values);
}

Matrix sMult(Matrix A, float k)
{
    float * values;
    values=s_mult_helper(A.values, k, A.rows*A.columns);
    return Matrix(A.rows, A.columns, values);
}

Matrix mSubtract(Matrix A, Matrix B)
{
    const int m_a=A.rows;
    const int n_b=B.columns;
    const int n_a=A.columns;
    const int m_b=B.rows;
    float * values;
    if ((m_a==m_b)&&(n_a==n_b))
    {
        values=m_sub_helper(A.values, B.values, m_a*n_a);
    }
    return Matrix(m_a, n_a, values);
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

int main()
{
    float d_values [6]={1, 1,1, 1, 1, 1};
    Matrix D=Matrix(2, 3, d_values);
    float a_values [6] = {1, 2, 3, 4, 5, 6};
    Matrix A=Matrix(2, 3, a_values);
    float b_values [6] = {7, 8, 9, 10, 11, 12};
    Matrix B=Matrix(3, 2, b_values);
    Matrix C=mMult(mAdd(A,D), B);
    display_matrix(C);
    return 0;
}