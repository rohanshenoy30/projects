#include <iostream>

void copyString(const char* source, char* destination) 
{
    while (*source != '\0') 
    {
        *destination = *source;
        source++;
        destination++;
    }
    *destination = '\0';  
}

int main() {
    const char* sourceString = "Hello, World!";
    char destinationString[50];  

    copyString(sourceString, destinationString);

    std::cout << "Source String: " << sourceString << std::endl;
    std::cout << "Copied String: " << destinationString << std::endl;

    return 0;
}
