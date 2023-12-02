#include <stdio.h>

void print(int arr[])
{
    int n = sizeof(arr)/sizeof(arr[0]);
    // arr is the address of the array, reference to the address is a pointer, size of a pointer in 64 bit system is 8, size of an interger is 4, so 8 /4 = ;
    int i;
    for (i=0; i<n; i++)
    {
        printf("%d \n",arr[i]);
    
    }
}

int main()
{
    int arr[] = {1,2,3,4,5,6,7,8};
    print(arr);
    return 0;
}