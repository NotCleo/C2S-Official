#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#define W 320
#define H 240
#define SIZE (W * H)

// --- Helper: Save Buffer ---
void save_bin(const char *filename, uint8_t *data) {
    FILE *f = fopen(filename, "wb");
    if (f) {
        fwrite(data, 1, SIZE, f);
        fclose(f);
        printf("[SAVED] %s\n", filename);
    } else {
        printf("[ERROR] Could not save %s\n", filename);
    }
}

// --- Helper: Convolution ---
// Applying 3x3 kernel. 'offset' is added to result (128 for emboss).
void convolve(uint8_t *in, uint8_t *out, int k[3][3], int div, int offset) {
    for (int y = 1; y < H-1; y++) {
        for (int x = 1; x < W-1; x++) {
            int sum = 0;
            for (int j = -1; j <= 1; j++) {
                for (int i = -1; i <= 1; i++) {
                    sum += in[(y+j)*W + (x+i)] * k[j+1][i+1];
                }
            }
            if (div != 1) sum /= div;
            sum += offset;
            if (sum < 0) sum = 0; 
            if (sum > 255) sum = 255;
            out[y*W + x] = (uint8_t)sum;
        }
    }
}

// --- 1. Sharpen Filter ---
void filter_sharpen(uint8_t *in, uint8_t *out) {
    // Standard Sharpen Kernel
    int k[3][3] = {
        { 0, -1,  0},
        {-1,  5, -1},
        { 0, -1,  0}
    };
    convolve(in, out, k, 1, 0);
}

// --- 2. Emboss Filter (Fine Tuned) ---
void filter_emboss(uint8_t *in, uint8_t *out) {
    // Tuned Kernel for Rails: Less aggressive than standard
    // Enhances vertical edges (tracks)
    int k[3][3] = {
        {-1,  0,  0},
        { 0,  0,  0},
        { 0,  0,  1}
    };
    convolve(in, out, k, 1, 128); // 128 offset makes flat areas gray
}

// --- 3. Sobel Filter ---
void filter_sobel(uint8_t *in, uint8_t *out) {
    int gx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    int gy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};
    
    for (int y = 1; y < H-1; y++) {
        for (int x = 1; x < W-1; x++) {
            int sumX = 0, sumY = 0;
            for (int j = -1; j <= 1; j++) {
                for (int i = -1; i <= 1; i++) {
                    int val = in[(y+j)*W + (x+i)];
                    sumX += val * gx[j+1][i+1];
                    sumY += val * gy[j+1][i+1];
                }
            }
            int mag = (int)sqrt(sumX*sumX + sumY*sumY);
            if (mag > 255) mag = 255;
            out[y*W + x] = (uint8_t)mag;
        }
    }
}

// --- 4. Canny Edge Detector (Simplified) ---
// Gaussian Blur -> Sobel -> Non-Max Suppression
void filter_canny(uint8_t *in, uint8_t *out) {
    // Step A: Gaussian Blur (Approximate)
    uint8_t *blur = malloc(SIZE);
    int k_gauss[3][3] = {{1, 2, 1}, {2, 4, 2}, {1, 2, 1}};
    convolve(in, blur, k_gauss, 16, 0);

    // Step B: Sobel Magnitude & Direction
    int *mag = malloc(SIZE * sizeof(int));
    float *theta = malloc(SIZE * sizeof(float)); // Gradient direction
    
    int gx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    int gy[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

    for (int y = 1; y < H-1; y++) {
        for (int x = 1; x < W-1; x++) {
            int sumX = 0, sumY = 0;
            for (int j = -1; j <= 1; j++) {
                for (int i = -1; i <= 1; i++) {
                    int val = blur[(y+j)*W + (x+i)];
                    sumX += val * gx[j+1][i+1];
                    sumY += val * gy[j+1][i+1];
                }
            }
            mag[y*W + x] = (int)sqrt(sumX*sumX + sumY*sumY);
            theta[y*W + x] = atan2(sumY, sumX) * 180.0 / 3.14159;
        }
    }
    free(blur);

    // Step C: Non-Maximum Suppression
    // Thin the edges
    memset(out, 0, SIZE);
    for (int y = 1; y < H-1; y++) {
        for (int x = 1; x < W-1; x++) {
            float angle = theta[y*W + x];
            if (angle < 0) angle += 180;
            
            int q = 255, r = 255;
            int m = mag[y*W + x];

            // 0 degrees (Vertical Edge)
            if ((angle >= 0 && angle < 22.5) || (angle >= 157.5 && angle <= 180)) {
                q = mag[y*W + (x+1)]; r = mag[y*W + (x-1)];
            }
            // 45 degrees
            else if (angle >= 22.5 && angle < 67.5) {
                q = mag[(y+1)*W + (x-1)]; r = mag[(y-1)*W + (x+1)];
            }
            // 90 degrees (Horizontal Edge)
            else if (angle >= 67.5 && angle < 112.5) {
                q = mag[(y+1)*W + x]; r = mag[(y-1)*W + x];
            }
            // 135 degrees
            else if (angle >= 112.5 && angle < 157.5) {
                q = mag[(y-1)*W + (x-1)]; r = mag[(y+1)*W + (x+1)];
            }

            if (m >= q && m >= r)
                out[y*W + x] = (uint8_t)(m > 255 ? 255 : m);
            else
                out[y*W + x] = 0;
        }
    }
    free(mag);
    free(theta);
}

int main() {
    printf("[PROCESS] Reading image.bin...\n");
    FILE *f = fopen("image.bin", "rb");
    if (!f) { printf("Error: image.bin not found\n"); return -1; }
    
    // 1. Load YUV and Extract Y (Grayscale)
    uint8_t *yuv = malloc(W * H * 2);
    uint8_t *gray = malloc(SIZE);
    
    fread(yuv, 1, W * H * 2, f);
    fclose(f);

    for (int i = 0; i < SIZE; i++) {
        gray[i] = yuv[i * 2]; // Extract Y
    }
    free(yuv);

    // Buffers
    uint8_t *sharpened = malloc(SIZE);
    uint8_t *embossed = malloc(SIZE);
    uint8_t *sobel = malloc(SIZE);
    uint8_t *canny = malloc(SIZE);

    // 2. Run Pipeline
    printf("[1/4] Sharpening...\n");
    filter_sharpen(gray, sharpened);
    save_bin("sharpened_image.bin", sharpened);

    printf("[2/4] Embossing (on sharpened)...\n");
    filter_emboss(sharpened, embossed); // Uses sharpened input
    save_bin("emboss_image.bin", embossed);

    printf("[3/4] Sobel (on sharpened)...\n");
    filter_sobel(sharpened, sobel);     // Uses sharpened input
    save_bin("sobel_image.bin", sobel);

    printf("[4/4] Canny (on sharpened)...\n");
    filter_canny(sharpened, canny);     // Uses sharpened input
    save_bin("canny_image.bin", canny);

    printf("[DONE] All filters applied.\n");
    
    free(gray); free(sharpened); free(embossed); free(sobel); free(canny);
    return 0;
}
