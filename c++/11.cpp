#include <iostream>
#include <vector>
#include <chrono>

using namespace std;

vector<long long> fib(41, -1); 

long long fibonacci(int n) {
    if (fib[n] != -1) {
        return fib[n]; 
    }
    if (n <= 1) {
        return n;
    }
    fib[n] = fibonacci(n - 1) + fibonacci(n - 2);
    return fib[n];
}

int main() 
{
    time_t s, e;
	time(&s);

    cout << "First 40 terms of the Fibonacci sequence:" << endl;
    for (int i = 0; i < 40; i++) {
        cout << fibonacci(i) << " ";
    }
    cout << endl;

    time(&e);
	int exetime = int(e - s);
    cout << "\nExecution time: " << exetime << " sec\n";
    return 0;
}
