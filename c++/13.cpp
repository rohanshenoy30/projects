#include <iostream>

int factorial(int n) 
{
    if (n == 0 || n == 1) 
    {
        return 1; 
    } else {
        return n * factorial(n - 1); 
    }
}

int main() 
{
    int num;

    
    std::cout << "Enter a non-negative integer: ";
    std::cin >> num;

    if (num < 0)
     {
        std::cout << "Factorial is not defined for negative numbers." << std::endl;
    } else {
        int result = factorial(num);
        std::cout << "The factorial of " << num << " is: " << result << std::endl;
    }

    return 0;
}
