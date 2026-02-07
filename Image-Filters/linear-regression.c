/*
	Ideas that i came across while writing this :
		How to find inverse of a matrix using gauss jordan elimination
		How to find inverse of a matrix using LU decompositon and extending gauss jordan idea
		How to multiply (mxn) * (nxp) instead of the usual (nxn) * (nxn) that I am used to!!
		How to label matrix multiply loops better!!
		How to solve for Linear regression's closed form
		Possibly use this code on the Arty7 board via HLS
*/

#include <stdio.h>
#include <stdlib.h> 
#include <math.h>

void inversematrix(float matrix[d][d], float inverse[d][d]) {
    for (int i = 0; i < d; i++) {
        for (int j = 0; j < d; j++) {
            if (i == j) inverse[i][j] = 1.0;
            else inverse[i][j] = 0.0;
        }
    }
    for (int i = 0; i < d; i++) {
        int pivot = i;
        for (int j = i + 1; j < d; j++) {
            if (fabs(matrix[j][i]) > fabs(matrix[pivot][i])) {
                pivot = j;
            }
        }
        
        float temp[d]; 
        for (int k = 0; k < d; k++) {
            float t = matrix[i][k];
            matrix[i][k] = matrix[pivot][k];
            matrix[pivot][k] = t;
            
            t = inverse[i][k];
            inverse[i][k] = inverse[pivot][k];
            inverse[pivot][k] = t;
        }

        if (fabs(matrix[i][i]) < 1e-9) {
            printf("The matrix is singular\n");
            exit(1);
        }

        float divisor = matrix[i][i];
        for (int j = 0; j < d; j++) {
            matrix[i][j] /= divisor;
            inverse[i][j] /= divisor;
        }

        for (int k = 0; k < d; k++) {
            if (k != i) {
                float factor = matrix[k][i];
                for (int j = 0; j < d; j++) {
                    matrix[k][j] -= factor * matrix[i][j];
                    inverse[k][j] -= factor * inverse[i][j];
                }
            }
        }
    }
}

int main(){

    // Column 0 is always 1
    float X[n][d];
    
    float y[n][1];

    float X_transpose[d][n];
    for(int i=0; i<n; i++){
        for(int j=0; j<d; j++){
            X_transpose[j][i] = X[i][j];            
        }
    }

    float X_transpose_X_product[d][d];
    float sum = 0; 
    
    for(int i=0; i<d; i++){
        for(int j=0; j<d; j++){
            sum = 0;
            for(int k=0; k<n; k++){
                sum += X_transpose[i][k] * X[k][j];                        
            }
            X_transpose_X_product[i][j] = sum;
        }
    }
    
    float X_transpose_y_product[d][1];
    
    for(int i=0; i<d; i++){
        for(int j=0; j<1; j++){
            sum = 0; 
            for(int k=0; k<n; k++){
                sum += X_transpose[i][k] * y[k][j];                        
            }
            X_transpose_y_product[i][j] = sum;
        }
    }
    
    float X_transpose_X_product_inverse[d][d];
    inversematrix(X_transpose_X_product, X_transpose_X_product_inverse);
    
    float final_weights[d][1]; 
    
    for(int i=0; i<d; i++){
        for(int j=0; j<1; j++){
            sum = 0;
            for(int k=0; k<d; k++){ 
                sum += X_transpose_X_product_inverse[i][k] * X_transpose_y_product[k][j];                        
            }
            final_weights[i][j] = sum;
        }
    }    
    
    int choice;
    float input_val;
    float prediction;

    while(1) {
        printf("1. Predict\n");
        printf("2. Exit\n");
        printf("Enter choice: ");
        scanf("%d", &choice);

        if (choice == 1) {
            printf("Enter value: ");
            scanf("%f", &input_val);

            prediction = final_weights[0][0] + (final_weights[1][0] * input_val);

            printf("Predicted value: %.3f\n", prediction);
        } 
        else if (choice == 2) {
            printf("ngl gang ts pmo\n");
            break;
        } 
        else {
            printf("Invalid\n");
        }
    }
}
