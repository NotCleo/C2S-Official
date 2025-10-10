// sobel filter 

/* #include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <jpeglib.h>
#include <sys/stat.h>  // For mkdir
#include <sys/types.h> // For mkdir

#define KERNEL_SIZE 3
#define MORPH_RADIUS 1  // Adjust for morphology

// Blur kernel
int blur_kernel[KERNEL_SIZE][KERNEL_SIZE] = {
    {0, 1, 0},
    {1, 2, 1},
    {0, 1, 0}
};

// Sobel vertical kernel
int sobel_vert_kernel[KERNEL_SIZE][KERNEL_SIZE] = {
    {-1, 0, 1},
    {-2, 0, 2},
    {-1, 0, 1}
};

// Sobel horizontal kernel
int sobel_horiz_kernel[KERNEL_SIZE][KERNEL_SIZE] = {
    {-1, -2, -1},
    {0, 0, 0},
    {1, 2, 1}
};

// Read JPEG file and output grayscale matrix
unsigned char** read_jpeg_grayscale(const char* filename, int* width, int* height) {
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE* infile = fopen(filename, "rb");
    if (!infile) { perror("Cannot open input file"); exit(1); }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, infile);
    jpeg_read_header(&cinfo, 1);

    jpeg_start_decompress(&cinfo);

    *width = cinfo.output_width;
    *height = cinfo.output_height;
    int channels = cinfo.output_components;

    unsigned char* buffer = (unsigned char*) malloc(*width * channels);
    unsigned char** gray = (unsigned char**) malloc(*height * sizeof(unsigned char*));
    for (int i = 0; i < *height; i++)
        gray[i] = (unsigned char*) malloc(*width * sizeof(unsigned char));

    while (cinfo.output_scanline < cinfo.output_height) {
        unsigned char* rowptr = buffer;
        jpeg_read_scanlines(&cinfo, &rowptr, 1);

        int y = cinfo.output_scanline - 1;
        for (int x = 0; x < *width; x++) {
            unsigned char R = rowptr[x * channels + 0];
            unsigned char G = rowptr[x * channels + 1];
            unsigned char B = rowptr[x * channels + 2];
            gray[y][x] = (unsigned char)(0.299 * R + 0.587 * G + 0.114 * B); // Convert to grayscale
        }
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(infile);
    free(buffer);
    return gray;
}

// Write JPEG file
void write_jpeg(const char* filename, unsigned char** img, int width, int height, int quality) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE* outfile = fopen(filename, "wb");
    if (!outfile) { perror("Cannot open output file"); exit(1); }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, outfile);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 1;  // Grayscale
    cinfo.in_color_space = JCS_GRAYSCALE;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);  // Adjust quality (0-100)

    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW row_pointer[1];
    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0] = img[cinfo.next_scanline];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    fclose(outfile);
}

// General kernel application
unsigned char** apply_kernel(unsigned char** img, int width, int height, int k[KERNEL_SIZE][KERNEL_SIZE], int divisor, int take_abs) {
    unsigned char** out = (unsigned char**) malloc(height * sizeof(unsigned char*));
    for (int i = 0; i < height; i++)
        out[i] = (unsigned char*) calloc(width, sizeof(unsigned char));

    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int sum = 0;
            for (int ky = -1; ky <= 1; ky++) {
                for (int kx = -1; kx <= 1; kx++) {
                    sum += k[ky + 1][kx + 1] * img[y + ky][x + kx];
                }
            }
            if (take_abs) sum = abs(sum);
            sum /= divisor;
            if (sum < 0) sum = 0;
            if (sum > 255) sum = 255;
            out[y][x] = (unsigned char) sum;
        }
    }
    return out;
}

// Apply Sobel edge detection and return magnitude
unsigned char** apply_sobel(unsigned char** img, int width, int height) {
    unsigned char** gx = apply_kernel(img, width, height, sobel_horiz_kernel, 1, 1); // Horizontal gradient
    unsigned char** gy = apply_kernel(img, width, height, sobel_vert_kernel, 1, 1); // Vertical gradient

    unsigned char** magnitude = (unsigned char**) malloc(height * sizeof(unsigned char*));
    for (int i = 0; i < height; i++)
        magnitude[i] = (unsigned char*) calloc(width, sizeof(unsigned char));

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int mag = (int)sqrt((double)(gx[y][x] * gx[y][x] + gy[y][x] * gy[y][x]));
            if (mag > 255) mag = 255;
            magnitude[y][x] = (unsigned char)mag;
        }
    }

    // Cleanup gx and gy
    for (int i = 0; i < height; i++) {
        free(gx[i]);
        free(gy[i]);
    }
    free(gx);
    free(gy);

    return magnitude;
}

int main() {
    // Array of input and output file pairs
    const char* input_files[] = {
        "/home/amrut/Downloads/lefttracktopview_drifted.jpg",
        "/home/amrut/Downloads/righttracktopview_drifted.jpg"
    };
    const char* output_sobel_files[] = {
        "/home/amrut/Downloads/test_output/lefttracktopview_sobel.jpg",
        "/home/amrut/Downloads/test_output/righttracktopview_sobel.jpg"
    };
    const int num_images = 2;
    const int rail_width_estimate = 50; // Fine-tune this: Measure pixel width of rail head in your image

    // Create test_output directory if it doesn't exist
    struct stat st = {0};
    if (stat("/home/amrut/Downloads/test_output", &st) == -1) {
        mkdir("/home/amrut/Downloads/test_output", 0755);
    }

    for (int img_idx = 0; img_idx < num_images; img_idx++) {
        int width, height;

        printf("Reading JPEG image %s...\n", input_files[img_idx]);
        unsigned char** gray = read_jpeg_grayscale(input_files[img_idx], &width, &height);
        printf("Image size: %dx%d\n", width, height);

        // Compute column variances
        float* variance = (float*) malloc(width * sizeof(float));
        for (int x = 0; x < width; x++) {
            float sum = 0.0f;
            for (int y = 0; y < height; y++) sum += gray[y][x];
            float mean = sum / height;
            float var = 0.0f;
            for (int y = 0; y < height; y++) var += powf(gray[y][x] - mean, 2);
            variance[x] = var / height;
        }

        // Find the starting x for the rail (lowest average variance window)
        float min_avg_var = FLT_MAX;
        int rail_start_x = 0;
        for (int start = 0; start <= width - rail_width_estimate; start++) {
            float avg_var = 0.0f;
            for (int i = 0; i < rail_width_estimate; i++) avg_var += variance[start + i];
            avg_var /= rail_width_estimate;
            if (avg_var < min_avg_var) {
                min_avg_var = avg_var;
                rail_start_x = start;
            }
        }
        printf("Detected rail from x=%d to x=%d\n", rail_start_x, rail_start_x + rail_width_estimate - 1);

        // Extract rail region
        unsigned char** rail_gray = (unsigned char**) malloc(height * sizeof(unsigned char*));
        for (int i = 0; i < height; i++)
            rail_gray[i] = (unsigned char*) malloc(rail_width_estimate * sizeof(unsigned char));
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < rail_width_estimate; x++) {
                rail_gray[y][x] = gray[y][rail_start_x + x];
            }
        }

        // Apply blur
        printf("Applying blur...\n");
        unsigned char** blurred = apply_kernel(rail_gray, rail_width_estimate, height, blur_kernel, 6, 0);

        // Apply Sobel
        printf("Applying Sobel...\n");
        unsigned char** sobel_mag = apply_sobel(blurred, rail_width_estimate, height);

        // Create final Sobel image: rail region with Sobel edges, rest as 0
        unsigned char** final_sobel = (unsigned char**) malloc(height * sizeof(unsigned char*));
        for (int i = 0; i < height; i++)
            final_sobel[i] = (unsigned char*) calloc(width, sizeof(unsigned char));
        for (int y = 0; y < height; y++) {
            for (int x = rail_start_x; x < rail_start_x + rail_width_estimate; x++) {
                int rx = x - rail_start_x;
                final_sobel[y][x] = sobel_mag[y][rx]; // Sobel magnitude in rail region
            }
        }

        // Write Sobel output as JPEG
        printf("Writing Sobel JPEG output...\n");
        write_jpeg(output_sobel_files[img_idx], final_sobel, width, height, 90);

        // Cleanup memory for this image
        for (int i = 0; i < height; i++) {
            free(gray[i]);
            free(rail_gray[i]);
            free(blurred[i]);
            free(sobel_mag[i]);
            free(final_sobel[i]);
        }
        free(gray);
        free(rail_gray);
        free(blurred);
        free(sobel_mag);
        free(final_sobel);
        free(variance);

        printf("Done! Output written to %s\n", output_sobel_files[img_idx]);
    }

    return 0;
} */


// binary image


/*#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <jpeglib.h>

#define KERNEL_SIZE 3
#define MORPH_RADIUS 1  // Adjust for larger/smaller morphology (1=3x3, 2=5x5, etc.) to fine-tune noise removal

// Blur kernel
int blur_kernel[KERNEL_SIZE][KERNEL_SIZE] = {
    {0, 1, 0},
    {1, 2, 1},
    {0, 1, 0}
};

// Laplacian kernel
int lap_kernel[KERNEL_SIZE][KERNEL_SIZE] = {
    {0, -1, 0},
    {-1, 4, -1},
    {0, -1, 0}
};

// Read JPEG file and output grayscale matrix
unsigned char** read_jpeg_grayscale(const char* filename, int* width, int* height) {
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE* infile = fopen(filename, "rb");
    if (!infile) { perror("Cannot open input file"); exit(1); }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, infile);
    jpeg_read_header(&cinfo, 1);

    jpeg_start_decompress(&cinfo);

    *width = cinfo.output_width;
    *height = cinfo.output_height;
    int channels = cinfo.output_components;

    unsigned char* buffer = (unsigned char*) malloc(*width * channels);
    unsigned char** gray = (unsigned char**) malloc(*height * sizeof(unsigned char*));
    for (int i = 0; i < *height; i++)
        gray[i] = (unsigned char*) malloc(*width * sizeof(unsigned char));

    while (cinfo.output_scanline < cinfo.output_height) {
        unsigned char* rowptr = buffer;
        jpeg_read_scanlines(&cinfo, &rowptr, 1);

        int y = cinfo.output_scanline - 1;
        for (int x = 0; x < *width; x++) {
            unsigned char R = rowptr[x * channels + 0];
            unsigned char G = rowptr[x * channels + 1];
            unsigned char B = rowptr[x * channels + 2];
            gray[y][x] = (unsigned char)(0.299 * R + 0.587 * G + 0.114 * B); // Convert to grayscale
        }
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(infile);
    free(buffer);
    return gray;
}

// Write JPEG file
void write_jpeg(const char* filename, unsigned char** img, int width, int height, int quality) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE* outfile = fopen(filename, "wb");
    if (!outfile) { perror("Cannot open output file"); exit(1); }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, outfile);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 1;  // Grayscale
    cinfo.in_color_space = JCS_GRAYSCALE;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);  // Adjust quality (0-100)

    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW row_pointer[1];
    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0] = img[cinfo.next_scanline];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    fclose(outfile);
}

// General kernel application
unsigned char** apply_kernel(unsigned char** img, int width, int height, int k[KERNEL_SIZE][KERNEL_SIZE], int divisor, int take_abs) {
    unsigned char** out = (unsigned char**) malloc(height * sizeof(unsigned char*));
    for (int i = 0; i < height; i++)
        out[i] = (unsigned char*) calloc(width, sizeof(unsigned char));

    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int sum = 0;
            for (int ky = -1; ky <= 1; ky++) {
                for (int kx = -1; kx <= 1; kx++) {
                    sum += k[ky + 1][kx + 1] * img[y + ky][x + kx];
                }
            }
            if (take_abs) sum = abs(sum);
            sum /= divisor;
            if (sum < 0) sum = 0;
            if (sum > 255) sum = 255;
            out[y][x] = (unsigned char) sum;
        }
    }
    return out;
}

// Apply erosion (square structuring element)
unsigned char** apply_erosion(unsigned char** img, int width, int height) {
    unsigned char** out = (unsigned char**) malloc(height * sizeof(unsigned char*));
    for (int i = 0; i < height; i++)
        out[i] = (unsigned char*) calloc(width, sizeof(unsigned char));

    for (int y = MORPH_RADIUS; y < height - MORPH_RADIUS; y++) {
        for (int x = MORPH_RADIUS; x < width - MORPH_RADIUS; x++) {
            unsigned char min_val = 255;
            for (int ky = -MORPH_RADIUS; ky <= MORPH_RADIUS; ky++) {
                for (int kx = -MORPH_RADIUS; kx <= MORPH_RADIUS; kx++) {
                    if (img[y + ky][x + kx] < min_val) min_val = img[y + ky][x + kx];
                }
            }
            out[y][x] = (min_val == 255) ? 255 : 0;
        }
    }
    return out;
}

// Apply dilation (square structuring element)
unsigned char** apply_dilation(unsigned char** img, int width, int height) {
    unsigned char** out = (unsigned char**) malloc(height * sizeof(unsigned char*));
    for (int i = 0; i < height; i++)
        out[i] = (unsigned char*) calloc(width, sizeof(unsigned char));

    for (int y = MORPH_RADIUS; y < height - MORPH_RADIUS; y++) {
        for (int x = MORPH_RADIUS; x < width - MORPH_RADIUS; x++) {
            unsigned char max_val = 0;
            for (int ky = -MORPH_RADIUS; ky <= MORPH_RADIUS; ky++) {
                for (int kx = -MORPH_RADIUS; kx <= MORPH_RADIUS; kx++) {
                    if (img[y + ky][x + kx] > max_val) max_val = img[y + ky][x + kx];
                }
            }
            out[y][x] = (max_val == 255) ? 255 : 0;
        }
    }
    return out;
}

// Otsu threshold calculation
int otsu_threshold(unsigned char** img, int width, int height) {
    int hist[256] = {0};
    int total = 0;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            hist[img[y][x]]++;
            total++;
        }
    }

    int thresh = 0;
    double max_sigma = -1.0;

    for (int t = 0; t < 256; t++) {
        int w1 = 0;
        double sum1 = 0.0;
        for (int i = 0; i < t; i++) {
            w1 += hist[i];
            sum1 += (double) i * hist[i];
        }
        if (w1 == 0) continue;
        double mu1 = sum1 / w1;

        int w2 = total - w1;
        if (w2 == 0) continue;
        double sum2 = 0.0;
        for (int i = t; i < 256; i++) {
            sum2 += (double) i * hist[i];
        }
        double mu2 = sum2 / w2;

        double sigma = (double) w1 / total * (double) w2 / total * (mu1 - mu2) * (mu1 - mu2);
        if (sigma > max_sigma) {
            max_sigma = sigma;
            thresh = t;
        }
    }
    return thresh;
}

int main() {
    // Array of input and output file pairs
    const char* input_files[] = {
        "/home/amrut/Downloads/lefttracktopview.jpg",
        "/home/amrut/Downloads/righttracktopview.jpg"
    };
    const char* output_files[] = {
        "output_left_filterized.jpg",
        "output_right_filterized.jpg"
    };
    const int num_images = 3;
    const int rail_width_estimate = 50; // Fine-tune this: Measure pixel width of rail head in your image

    for (int i = 0; i < num_images; i++) {
        int width, height;

        printf("Reading JPEG image %s...\n", input_files[i]);
        unsigned char** gray = read_jpeg_grayscale(input_files[i], &width, &height);
        printf("Image size: %dx%d\n", width, height);

        // Compute column variances
        float* variance = (float*) malloc(width * sizeof(float));
        for (int x = 0; x < width; x++) {
            float sum = 0.0f;
            for (int y = 0; y < height; y++) sum += gray[y][x];
            float mean = sum / height;
            float var = 0.0f;
            for (int y = 0; y < height; y++) var += powf(gray[y][x] - mean, 2);
            variance[x] = var / height;
        }

        // Find the starting x for the rail (lowest average variance window)
        float min_avg_var = FLT_MAX;
        int rail_start_x = 0;
        for (int start = 0; start <= width - rail_width_estimate; start++) {
            float avg_var = 0.0f;
            for (int i = 0; i < rail_width_estimate; i++) avg_var += variance[start + i];
            avg_var /= rail_width_estimate;
            if (avg_var < min_avg_var) {
                min_avg_var = avg_var;
                rail_start_x = start;
            }
        }
        printf("Detected rail from x=%d to x=%d\n", rail_start_x, rail_start_x + rail_width_estimate - 1);

        // Extract rail region
        unsigned char** rail_gray = (unsigned char**) malloc(height * sizeof(unsigned char*));
        for (int i = 0; i < height; i++)
            rail_gray[i] = (unsigned char*) malloc(rail_width_estimate * sizeof(unsigned char));
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < rail_width_estimate; x++) {
                rail_gray[y][x] = gray[y][rail_start_x + x];
            }
        }

        // Apply blur
        printf("Applying blur...\n");
        unsigned char** blurred = apply_kernel(rail_gray, rail_width_estimate, height, blur_kernel, 6, 0);

        // Apply Laplacian
        printf("Applying Laplacian...\n");
        unsigned char** lap = apply_kernel(blurred, rail_width_estimate, height, lap_kernel, 1, 1);

        // Compute Otsu threshold on Laplacian
        int thresh = otsu_threshold(lap, rail_width_estimate, height);
        printf("Otsu threshold: %d\n", thresh);

        // Binarize (fine-tune by adjusting thresh manually if Otsu isn't optimal, e.g., thresh += 10;)
        unsigned char** binary = (unsigned char**) malloc(height * sizeof(unsigned char*));
        for (int i = 0; i < height; i++)
            binary[i] = (unsigned char*) calloc(rail_width_estimate, sizeof(unsigned char));
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < rail_width_estimate; x++) {
                binary[y][x] = (lap[y][x] > thresh) ? 255 : 0;
            }
        }

        // Apply opening (erosion + dilation) for noise removal
        printf("Removing noise...\n");
        unsigned char** eroded = apply_erosion(binary, rail_width_estimate, height);
        unsigned char** opened = apply_dilation(eroded, rail_width_estimate, height);

        // Create final binary image: rail highlighted (255), cracks as 0, background 0
        unsigned char** final_binary = (unsigned char**) malloc(height * sizeof(unsigned char*));
        for (int i = 0; i < height; i++)
            final_binary[i] = (unsigned char*) calloc(width, sizeof(unsigned char));
        for (int y = 0; y < height; y++) {
            for (int x = rail_start_x; x < rail_start_x + rail_width_estimate; x++) {
                int rx = x - rail_start_x;
                final_binary[y][x] = (opened[y][rx] == 255) ? 0 : 255; // Cracks as black on white rail
            }
        }

        // Write output as JPEG
        printf("Writing binary JPEG output...\n");
        write_jpeg(output_files[i], final_binary, width, height, 90); // Quality set to 90

        // Cleanup memory for this image
        for (int i = 0; i < height; i++) {
            free(gray[i]);
            free(rail_gray[i]);
            free(blurred[i]);
            free(lap[i]);
            free(binary[i]);
            free(eroded[i]);
            free(opened[i]);
            free(final_binary[i]);
        }
        free(gray);
        free(rail_gray);
        free(blurred);
        free(lap);
        free(binary);
        free(eroded);
        free(opened);
        free(final_binary);
        free(variance);

        printf("Done! Output written to %s\n", output_files[i]);
    }

    return 0;
}*/




