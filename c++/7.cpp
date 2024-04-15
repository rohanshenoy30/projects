#include <iostream>
#include <vector>

using namespace std;

void bubbleSort(vector<int>& arr) {
    int n = arr.size();
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                swap(arr[j], arr[j + 1]);
            }
        }
    }
}

void selectionSort(vector<int>& arr) {
    int n = arr.size();
    for (int i = 0; i < n - 1; i++) {
        int minIndex = i;
        for (int j = i + 1; j < n; j++) {
            if (arr[j] < arr[minIndex]) {
                minIndex = j;
            }
        }
        if (minIndex != i) {
            swap(arr[i], arr[minIndex]);
        }
    }
}

int binarySearch(const vector<int>& arr, int target) {
    int left = 0;
    int right = arr.size() - 1;
    
    while (left <= right) {
        int mid = left + (right - left) / 2;
        if (arr[mid] == target) {
            return mid;
        }
        if (arr[mid] < target) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    
    return -1; 
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

    char choice;
    cout << "Enter 's' for Selection Sort or 'b' for Bubble Sort: ";
    cin >> choice;

    if (choice == 's') {
        selectionSort(arr);
        cout << "Array after Selection Sort: ";
    } else if (choice == 'b') {
        bubbleSort(arr);
        cout << "Array after Bubble Sort: ";
    } else {
        cout << "Invalid choice. Exiting..." << endl;
        return 1;
    }

    for (int num : arr) {
        cout << num << " ";
    }
    cout << endl;

    int target;
    cout << "Enter the number to search for: ";
    cin >> target;

    int result = binarySearch(arr, target);
    if (result != -1) {
        cout << target << " found at position " << result << endl;
    } else {
        cout << target << " not found in the array." << endl;
    }

    return 0;
}
