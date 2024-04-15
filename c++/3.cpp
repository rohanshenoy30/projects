#include <iostream>
#include <vector>

using namespace std;

int** setZeroes(int** matrix, int r,int  c) 
{

    int* zeroRows = new int[r];
    int* zeroCols = new int[c];
    
    for (int i = 0; i < r; i++) {
        zeroRows[i] = false;
    }
    for (int i = 0; i < c; i++) {
        zeroCols[i] = false;
    }

    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) 
        {
            if (matrix[i][j] == 0) {
                zeroRows[i] = true;
                zeroCols[j] = true;
            }
        }
    }
    
    for (int i = 0; i < r; i++) 
    {
        for (int j = 0; j < c; j++) {
            if (zeroRows[i] || zeroCols[j]) 
            {
                matrix[i][j] = 0;
            }
        }
    }

    return matrix;
}

int main() 
{

    cout << "Enter rows and columns of matrix";
    int r, c;
    cin >> r >> c;
    int** matrix = new int*[r]; 
    for (int i = 0; i < r; i++) {
		matrix[i] = new int[c];
		for (int j = 0; j < c; j++) {
			cin >> matrix[i][j];
		}
	}
    
    matrix = setZeroes(matrix,r,c);
    
    for (int i = 0; i < r; i++ ) 
    {
        for (int j = 0; j < c; j++) 
        {
            cout << matrix[i][j] << " ";
        }
        cout << endl;
    }
    
    return 0;
}
