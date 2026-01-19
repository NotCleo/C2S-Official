#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#define W 320
#define H 240
#define YUV_SIZE (W * H * 2)
#define GRAY_SIZE (W * H)

uint8_t* alloc_gray() { return calloc(GRAY_SIZE, 1); }

void save_bin(const char *filename, uint8_t *data) {
    FILE *f = fopen(filename, "wb");
    if (f) {
        fwrite(data, 1, GRAY_SIZE, f);
        fclose(f);
        printf("[SAVED] %s\n", filename);
    }
}

int clamp(int v) { return (v < 0) ? 0 : (v > 255) ? 255 : v; }

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
            out[y*W + x] = clamp(sum);
        }
    }
}


void filter_sobel_mag(uint8_t *in, uint8_t *out) {
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
            out[y*W + x] = clamp((int)sqrt(sumX*sumX + sumY*sumY));
        }
    }
}

void filter_prewitt(uint8_t *in, uint8_t *out) {
    int gx[3][3] = {{-1, 0, 1}, {-1, 0, 1}, {-1, 0, 1}};
    int gy[3][3] = {{-1, -1, -1}, {0, 0, 0}, {1, 1, 1}};
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
            out[y*W + x] = clamp(abs(sumX) + abs(sumY)); 
        }
    }
}

void filter_laplacian(uint8_t *in, uint8_t *out) {
    int k[3][3] = {{0, 1, 0}, {1, -4, 1}, {0, 1, 0}};
    convolve(in, out, k, 1, 0); 
}


void filter_box_blur(uint8_t *in, uint8_t *out) {
    int k[3][3] = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}};
    convolve(in, out, k, 9, 0);
}

void filter_gaussian_blur(uint8_t *in, uint8_t *out) {
    int k[3][3] = {{1, 2, 1}, {2, 4, 2}, {1, 2, 1}};
    convolve(in, out, k, 16, 0);
}

void sort_array(uint8_t *arr, int n) {
    for (int i = 0; i < n-1; i++) {
        for (int j = 0; j < n-i-1; j++) {
            if (arr[j] > arr[j+1]) {
                uint8_t temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
}

void filter_median(uint8_t *in, uint8_t *out) {
    uint8_t window[9];
    for (int y = 1; y < H-1; y++) {
        for (int x = 1; x < W-1; x++) {
            int idx = 0;
            for (int j = -1; j <= 1; j++) {
                for (int i = -1; i <= 1; i++) {
                    window[idx++] = in[(y+j)*W + (x+i)];
                }
            }
            sort_array(window, 9);
            out[y*W + x] = window[4]; // The middle element
        }
    }
}

void filter_morph(uint8_t *in, uint8_t *out, int mode) {
    for (int y = 1; y < H-1; y++) {
        for (int x = 1; x < W-1; x++) {
            uint8_t val = in[y*W+x];
            for (int j = -1; j <= 1; j++) {
                for (int i = -1; i <= 1; i++) {
                    uint8_t neighbor = in[(y+j)*W + (x+i)];
                    if (mode == 0) { // Erosion
                        if (neighbor < val) val = neighbor;
                    } else { // Dilation
                        if (neighbor > val) val = neighbor;
                    }
                }
            }
            out[y*W + x] = val;
        }
    }
}

// --- 4. INTENSITY FILTERS ---

void filter_histogram_eq(uint8_t *in, uint8_t *out) {
    long hist[256] = {0};
    // 1. Compute Histogram
    for (int i = 0; i < GRAY_SIZE; i++) hist[in[i]]++;
    
    // 2. Compute CDF
    long cdf[256] = {0};
    cdf[0] = hist[0];
    for (int i = 1; i < 256; i++) cdf[i] = cdf[i-1] + hist[i];
    
    // 3. Map
    float alpha = 255.0f / (float)GRAY_SIZE;
    for (int i = 0; i < GRAY_SIZE; i++) {
        out[i] = (uint8_t)(cdf[in[i]] * alpha);
    }
}

void filter_gamma(uint8_t *in, uint8_t *out, float gamma) {
    // Create LUT for speed
    uint8_t lut[256];
    for (int i = 0; i < 256; i++) {
        lut[i] = clamp((int)(pow((float)i / 255.0, gamma) * 255.0));
    }
    // Apply
    for (int i = 0; i < GRAY_SIZE; i++) {
        out[i] = lut[in[i]];
    }
}

// --- MAIN PIPELINE ---

int main() {
    printf("[PROCESS] Loading 'image.bin'...\n");
    FILE *f = fopen("image.bin", "rb");
    if (!f) { printf("Error: File not found.\n"); return -1; }
    
    uint8_t *yuv_in = malloc(YUV_SIZE);
    fread(yuv_in, 1, YUV_SIZE, f);
    fclose(f);

    // 1. EXTRACT Y CHANNEL
    uint8_t *gray = alloc_gray();
    for (int i = 0; i < GRAY_SIZE; i++) gray[i] = yuv_in[i*2];
    free(yuv_in); // Done with color for now

    // 2. RUN FILTERS & SAVE
    uint8_t *temp = alloc_gray();
    uint8_t *temp2 = alloc_gray(); // For two-pass filters (Opening/Closing)

    printf("--- Edge Detection ---\n");
    filter_sobel_mag(gray, temp); save_bin("filter_sobel.bin", temp);
    filter_prewitt(gray, temp);   save_bin("filter_prewitt.bin", temp);
    filter_laplacian(gray, temp); save_bin("filter_laplacian.bin", temp);

    printf("--- Smoothing ---\n");
    filter_box_blur(gray, temp);      save_bin("filter_boxblur.bin", temp);
    filter_gaussian_blur(gray, temp); save_bin("filter_gaussian.bin", temp);
    filter_median(gray, temp);        save_bin("filter_median.bin", temp);

    printf("--- Sharpening ---\n");

    int k_sharp[3][3] = {{0,-1,0}, {-1,5,-1}, {0,-1,0}};
    convolve(gray, temp, k_sharp, 1, 0); save_bin("filter_sharpen.bin", temp);
    
    int k_emboss[3][3] = {{-2,-1,0}, {-1,1,1}, {0,1,2}};
    convolve(gray, temp, k_emboss, 1, 128); save_bin("filter_emboss.bin", temp);

    printf("--- Morphology ---\n");
    filter_morph(gray, temp, 0); save_bin("filter_erosion.bin", temp); // Erode
    filter_morph(gray, temp2, 1); save_bin("filter_dilation.bin", temp2); // Dilate
    
    filter_morph(temp, temp2, 1); save_bin("filter_opening.bin", temp2);
    
    filter_morph(gray, temp, 1); // Dilate original
    filter_morph(temp, temp2, 0); // Erode result
    save_bin("filter_closing.bin", temp2);

    printf("--- Intensity ---\n");
    filter_histogram_eq(gray, temp); save_bin("filter_hist_eq.bin", temp);
    filter_gamma(gray, temp, 0.5);   save_bin("filter_gamma_05.bin", temp); // Brighter
    filter_gamma(gray, temp, 2.0);   save_bin("filter_gamma_20.bin", temp); // Darker

    printf("[DONE] Generated 14 filter outputs.\n");
    
    free(gray); free(temp); free(temp2);
    return 0;
}
