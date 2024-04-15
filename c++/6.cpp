#include <iostream>
#include <vector>

using namespace std;

int sumOfDivisors(int num) {
    int sum = 1; 
    for (int i = 2; i * i <= num; i++) {
        if (num % i == 0) {
            sum += i;
            if (i != (num / i)) {
                sum += (num / i);
            }
        }
    }
    return sum;
}

int main() {
    int num1, num2;
    cout << "Enter two numbers: ";
    cin >> num1 >> num2;

    int sum1 = sumOfDivisors(num1);
    int sum2 = sumOfDivisors(num2);

    if (sum1 == num2 && sum2 == num1) {
        cout << num1 << " and " << num2 << " are amicable numbers." << endl;
    } else {
        cout << num1 << " and " << num2 << " are not amicable numbers." << endl;
    }

    return 0;
}
