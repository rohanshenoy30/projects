#include <iostream>
#include <vector>

using namespace std;

int binarySearchRecursive(const vector<int>& arr, int target, int left, int right) {
    if (left > right) {
        return -1; 
    }

    int mid = left + (right - left) / 2;

    if (arr[mid] == target) {
        return mid; 
    } else if (arr[mid] < target) {
        return binarySearchRecursive(arr, target, mid + 1, right);
    } else {
        return binarySearchRecursive(arr, target, left, mid - 1);
    }
}

int main() {
    int n;
    cout << "Enter the number of elements in the array: ";
    cin >> n;

    vector<int> arr(n);

    cout << "Enter " << n << " sorted integers separated by spaces: ";
    for (int i = 0; i < n; i++) {
        cin >> arr[i];
    }

    int target;
    cout << "Enter the number to search for: ";
    cin >> target;

    int result = binarySearchRecursive(arr, target, 0, n - 1);

    if (result != -1) {
        cout << "Element found at position " << result << endl;
    } else {
        cout << "Element not found in the array." << endl;
    }

    return 0;
}
