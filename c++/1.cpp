#include <iostream>
#include <cstring>
using namespace std;

void bubbleSort(char arr[], int n) 
{
    bool swapped;
    do {
        swapped=false;
        for (int i = 0; i < n - 1; i++) 
        {
            if (arr[i] > arr[i + 1]) 
            {
                swap(arr[i], arr[i + 1]);
                swapped = true;
            }
        }
    } while (swapped);
}

int main() 
{
    char input[] = "worksheet";
    int length = strlen(input);

    bubbleSort(input, length);

    cout << "Sorted string: " << input << endl;

    return 0;
}