#include <stdio.h>

#define image_size 7
#define kernel_size 3

int image[image_size][image_size] = {{1,1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1}};
int kernel[kernel_size][kernel_size] = {{1,1,1},{1,1,1},{1,1,1}};
int result[image_size - kernel_size + 1][image_size - kernel_size + 1];

//image is 7 x 7
//kernel is 3 x 3
//result is 5 x 5 

int main() {
			
/*
for(int i = 0; i<kernel_size; i++){
	for(int j = 0; j<kernel_size; j++){
		for(int k = 0; k<image_size - kernel_size + 1; k++){
			for(int l = 0; l<image_size - kernel_size + 1; l++){
			        for(int m = 0; m<image_size - kernel_size + 1; m++){
			                   result[k][m] += image[k][m] * kernel[l][m];
						    }
						}
					}
				}
			}
*/
			

for(int i = 0; i < image_size - kernel_size + 1; i++) {
        for(int j = 0; j < image_size - kernel_size + 1; j++) {
            
            for(int k = 0; k < kernel_size; k++) {
                for(int l = 0; l < kernel_size; l++) {
                    result[i][j] += image[i + k][j + l] * kernel[k][l];
                }
            }
        }
    }
    
for(int i=0; i<image_size - kernel_size + 1; i++){
	for(int j=0; j<image_size - kernel_size + 1; j++){
		    printf("%d ", result[i][j]);
				}
				printf("\n");
			} 
			
}



/*
int a[n][n] = {{1,1,1},{1,1,1},{1,1,1}};
int b[n][n] = {{1,1,1},{1,1,1},{1,1,1}};
int c[n][n];

for(int i=0; i<n; i++){
	for(int j=0; j<n; j++){
		for(int k=0; k<n; k++){
			c[i][k] += a[i][k] * b[j][k];
					}
				}
			}

for(int i=0; i<n; i++){
	for(int j=0; j<n; j++){
		    printf("%d ", c[i][j]);
				}
				printf("\n");
			} 
*/
