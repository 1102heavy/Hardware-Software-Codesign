#include <stdio.h>

// Main function: entry point for execution

#define A_COLS = 4 
#define A_ROWS = 2

int matrix_multiply(int A_Matrix[], int B_Matrix[],int result[]) {

    for(int r= 0; r < A_ROWS; r++) {
        int sum = 0;
        for(int c=0; c < A_COLS; c++) {
            sum += A_Matrix[r * A_COLS + c] * B_Matrix[c];
        
            // Storing the result back in result array for demonstration
        }
        result[r] = sum;
    }
    return 0;
}

int main() {
 
    int A_Matrix[8] = {1, 2, 3, 4, 0, 0, 0, 0};
    int B_Matrix[4] = {5, 6, 7, 8};
    int result[2] = {0,0};

    matrix_multiply(A_COLS, A_ROWS, A_Matrix, B_Matrix, result);

    // Print the result of multiplication
    printf("Result of Matrix Multiplication:\n");
    for(int i = 0; i < A_ROWS; i++) {
        printf("%d ", result[i]);
        printf("\n");
    }
    return 0;
}