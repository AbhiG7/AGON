#include <iostream>
using namespace std;

float ** mMult(float ** A, float ** B)
{
    const int m = sizeof(A) / sizeof(A[0]);
    const int n = sizeof(A[0]) / sizeof(A[0][0]);
    const int p = sizeof(B[0]) / sizeof(B[0][0]);
    if (n==sizeof(B) / sizeof(B[0]))
    {
        float ** C;
        C = new float*[m];
        for (int i = 0;  i < m; i++)
        {
            C[i]=new float[p];
            for (int j = 0; j < p; j++)
            {
                C[i][j]=0;
                for (int k = 0; k < n; k++) 
                {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }
    else
    {

    }
}

float ** mScale(float scalar, float ** A)
{
    const int m = sizeof(A) / sizeof(A[0]);
    const int n = sizeof(A[0]) / sizeof(A[0][0]);
    float ** C;
    C = new float*[m];
    for (int i = 0;  i < m; i++)
    {
        C[i]=new float[n];
        for (int j = 0; j < n; j++)
        {
            C[i][j]=scalar*A[i][j];
        }
    }
    return C;
}

float ** mAdd(float ** A, float ** B)
{
    const int m_a = sizeof(A) / sizeof(A[0]);
    const int n_a = sizeof(A[0]) / sizeof(A[0][0]);
    const int m_b = sizeof(B) / sizeof(B[0]);
    const int n_b = sizeof(B[0]) / sizeof(B[0][0]);
    if ((n_a==n_b) && (m_a==m_b))
    {
        float ** C;
        C = new float*[m_a];
        for (int i = 0;  i < m_a; i++)
        {
            C[i]=new float[n_a];
            for (int j = 0; j < n_a; j++)
            {
                C[i][j]=A[i][j]+B[i][j];
            }
        }
        return C;
    }
    else
    {

    }
}

float ** mSubtract(float ** A, float ** B)
{
    const int m_a = sizeof(A) / sizeof(A[0]);
    const int n_a = sizeof(A[0]) / sizeof(A[0][0]);
    const int m_b = sizeof(B) / sizeof(B[0]);
    const int n_b = sizeof(B[0]) / sizeof(B[0][0]);
    if ((n_a==n_b) && (m_a==m_b))
    {
        float ** C;
        C = new float*[m_a];
        for (int i = 0;  i < m_a; i++)
        {
            C[i]=new float[n_a];
            for (int j = 0; j < n_a; j++)
            {
                C[i][j]=A[i][j]-B[i][j];
            }
        }
        return C;
    }
    else
    {

    }
}

float ** create_matrix(float * values, int rows, int columns)
{
    float ** C;
    for (int i=0; i<rows; i++)
    {
        C = new float*[columns];
        for (int j = 0; j < columns; j++)
        {
            C[i][j]=values[i*columns+j];
        }
    }
    return C;
}

void display_matrix(float ** A, int rows, int columns)
{
    for (int i=0; i<rows; i++)
    {
        for (int j = 0; j < columns; j++)
        {
            cout << A[i][j] << "    ";
        }
        cout << "\n";
    }
}

int main()
{
    float** A;
    float** B;
    float** C;

    A=new float *[2];
    A[0]=new float [3] {1, 2, 3};
    A[1]=new float [3] {4, 5, 6};

    //A[0]=new float [] {1, 2, 3}; 
    //A[1]={4, 5, 6};
    //B[0]={7, 8}; 
    //B[1]={9, 10}; 
    //B[2]={11, 12}; 


    C=mMult(A, B);
    display_matrix(C, 2, 2);
    return 0;
}