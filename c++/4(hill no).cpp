#include <iostream>
#include <string>
using namespace std;

bool isHillNumber(int num) 
{
    std::string numStr = std::to_string(num);
    int length = numStr.length();
    
    if (length <= 1)
        return false;

    bool isAscending = true;
    bool isDescending = false;

    for (int i = 1; i < length; ++i) {
        if (numStr[i] > numStr[i - 1]) {
            if (isDescending)
                return false; 
        } else if (numStr[i] < numStr[i - 1]) {
            isDescending = true;
        } else {
            return false; 
        }
    }

    return isAscending && isDescending;
}

int main() {
    int num;

    cout << "Enter a number: ";
    cin >> num;

    if (isHillNumber(num)) {
        cout << num << " is a Hill Number." << endl;
    } else {
        cout << num << " is not a Hill Number." << endl;
    }

    return 0;
}
