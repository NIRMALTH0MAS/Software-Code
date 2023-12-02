#include <stdio.h>

int main()
{
    int arr[] = {10,20,30,40,50};
    printf("Base address of the array arr[] is: %p \n",&arr[0]);

    printf("Value at the address of the array arr[1]+3 is :  %d \n", arr[1]+3);
    printf("Value at the address of the array arr[1+3] is :  %d \n", arr[1+3]);

    return 0;

    // arr --> &arr --> &arr[0] --> arr[] -> all are literaly same, which is the base address of the arr;
    // arr++ will give error, becasue the incremental operator work only for the increminting value;
    // arr+1 will work , becasue it will increment to the next position

}