#include <iostream>
using namespace std;

int main()
{
    int row1,row2,col1,col2;
    cout <<"Enter the no. of rows and columns for the 1st matrix";
    cin >> row1 >> col1;
    cout<<"Enter the no. of rows and columns for the 2nd matrix";
    cin >> row2 >> col2;

    if(col1 != row2)
   { cout<<"Matrices are not compatible for multipliction";
    return 1;
   }


  std::vector<std::vector<int> > matrix1(row1, std::vector<int>(col1));
    std::vector<std::vector<int> > matrix2(row2, std::vector<int>(col2));
    std::vector<std::vector<int> > product(row1, std::vector<int>(col2));

    cout << "Enter elements of the first matrix:" << endl;
    for (int i = 0; i < row1; ++i) 
    {
    
        for (int j = 0; j < col1; ++j)
         {
            cin >> matrix1[i][j];
        }
    }

    cout << "Enter elements of the second matrix:" << endl;
    for (int i = 0; i < row2; ++i) 
    {
        for (int j = 0; j < col2; ++j)
         {
            cin >> matrix2[i][j];
        }
    }

    for (int i = 0; i < row1; ++i) 
    {
        for (int j = 0; j < col2; ++j) 
        {
            product[i][j] = 0;
            for (int k = 0; k < col1; ++k) 
            {
                product[i][j] += matrix1[i][k] * matrix2[k][j];
            }
        }
    }

    cout << "Product matrix:" << endl;
    for (int i = 0; i < row1; ++i)
     {
        for (int j = 0; j < col2; ++j)
         {
            cout << product[i][j] << " ";
        }
        cout << endl;
    }

    return 0;

}







