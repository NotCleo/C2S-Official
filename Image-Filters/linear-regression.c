#include<stdio.h>
#include<stdlib.h>
#include<math.h>

#define n 5
#define d 2

void inversematrix(float matrix[d][d], float inverse[d][d]) {
    for (int i = 0; i < d; i++) {
        for (int j = 0; j < d; j++) {
            if (i == j)
                inverse[i][j] = 1.0;
            else
                inverse[i][j] = 0.0;
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
        }
        for (int k = 0; k < d; k++) {
            float t = inverse[i][k];
            inverse[i][k] = inverse[pivot][k];
            inverse[pivot][k] = t;
        }

        if (fabs(matrix[i][i]) < 1e-9) {
            printf("The X vector does not have a valid inverse");
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
	float X[n][d];
	float X_transpose[d][n];
	float y[n][1];
	float w[d][1];
	
	for(int i=0; i<n; i++){
		for(int j=0; j<d; j++){
			X_transpose[j][i] = X[i][j];			
				}
			}
	float X_transpose_X_product[d][d];
	
	// (rows,columns)
	// (dxd) = (dxn) * (nxd) 
	float sum = 0;
	for(int i=0; i<d; i++){
		for(int j=0; j<d; j++){
			for(int k=0; k<n; k++){
			sum += X_transpose[i][k] * X[k][j];						
			}
			X_transpose_X_product[i][j] = sum;
			sum = 0;
		}
	}
	
	float X_transpose_y_product[d][1];
	sum = 0;
	// (rows,columns)
	// (dx1) = (dxn) * (nx1) 
	for(int i=0; i<d; i++){
		for(int j=0; j<1; j++){
			for(int k=0; k<n; k++){
			sum += X_transpose[i][k] * y[k][j];						
			}
			X_transpose_y_product[i][j] = sum;
			sum = 0;
		}
	}
	
	float X_transpose_X_product_inverse[d][d];
	
	inversematrix(X_transpose_X_product, X_transpose_X_product_inverse);
	
	
	float X_transpose_X_product_inverse_X_transpose_y_product_product[d][1];
	
	// (dx1) = (dxd) * (dx1)
	sum = 0;
	for(int i=0; i<d; i++){
		for(int j=0; j<1; j++){
			for(int k=0; k<d; k++){
			sum += X_transpose_X_product_inverse[i][k] * X_transpose_y_product[k][j];						
			}
			X_transpose_X_product_inverse_X_transpose_y_product_product[i][j] = sum;
			sum = 0;
		}
	}	
	
	
	for(int i=0; i<d; i++){
		printf("%f ", X_transpose_X_product_inverse_X_transpose_y_product_product[i][0]);
		}
}









