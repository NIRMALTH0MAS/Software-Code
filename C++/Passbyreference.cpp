#include <stdio.h>

void swap(int &x, int &y) // By giving the "&" in C++ it will swap the the address of the variable
{
    int temp = x;
    x = y;
    y = temp;
}

int main()
{
    int a = 100, b = 200;
    printf("a is %d and b is %d before swap \n", a, b);
    
    swap (a,b);
    
    printf("a is %d and b is %d after swap \n", a, b);
    
    return 0;
}