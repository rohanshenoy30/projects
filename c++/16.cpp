#include <iostream>
#include <vector>

using namespace std;

int findMinIndex(const vector<int>& arr, int start, int end) {
    int minIndex = start;
    for (int i = start + 1; i <= end; i++) {
        if (arr[i] < arr[minIndex]) {
            minIndex = i;
        }
    }
    return minIndex;
}

void recursiveSelectionSort(vector<int>& arr, int n, int currentIndex = 0) {
    if (currentIndex == n - 1) {
        return; 
    }

    int minIndex = findMinIndex(arr, currentIndex, n - 1);
    if (minIndex != currentIndex) {
        swap(arr[minIndex], arr[currentIndex]);
    }

    recursiveSelectionSort(arr, n, currentIndex + 1);
}

int main() {
    int n;
    cout << "Enter the number of elements in the array: ";
    cin >> n;

    vector<int> arr(n);

    cout << "Enter " << n << " integers separated by spaces: ";
    for (int i = 0; i < n; i++) {
        cin >> arr[i];
    }

    recursiveSelectionSort(arr, n);

    cout << "Sorted array using recursive selection sort: ";
    for (int num : arr) {
        cout << num << " ";
    }
    cout << endl;

    return 0;
}
