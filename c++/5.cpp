
#include <iostream>
#include <string>
using namespace std;

int main() 
{

	cout << "Enter number\n";
	int n;
	cin >> n;

	int m = n;
	string bin = "";
	while (m != 0) {
		bin = to_string(m % 2) + bin;
		m /= 2;
	}

	m = n;
	string oct = "";
	while (m != 0) {
		oct = to_string(m % 8) + oct;
		m /= 8;
	}

	m = n;
	string hex = "";
	static const char a[6] = { 'A','B','C','D','E','F' };
	while (m != 0) {
		if (m % 16 < 10) {
			hex = to_string(m % 16) + hex;
		}
		else {
			hex = a[m % 16 - 10] + hex;
		}
		m /= 16;
	}

	cout << "binary:      " << bin << "\n";
	cout << "octal:       " << oct << "\n";
	cout << "hexadecimal: " << hex << "\n";
}