#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <jpeglib.h>
#include <sys/stat.h>
#include <sys/types.h>

#define KERNEL_SIZE 3
#define RAIL_WIDTH_ESTIMATE 50
#define OUTPUT_DIR "/home/amrut/Downloads/test_output"

int blur_kernel[3][3] = {
    {1, 2, 1},
    {2, 4, 2},
    {1, 2, 1}
}; 

int sobel_h_kernel[3][3] = {
    {-1, 0, 1},
    {-2, 0, 2},
    {-1, 0, 1}
};

int sobel_v_kernel[3][3] = {
    {-1, -2, -1},
    {0,  0,  0},
    {1,  2,  1}
};

int emboss_kernel[3][3] = {
    {-2, -1, 0},
    {-1,  1, 1},
    { 0,  1, 2}
};

typedef struct {
    int width;
    int height;
    unsigned char **data;
} Image;

Image* create_image(int width, int height) {
    Image *img = (Image*)malloc(sizeof(Image));
    img->width = width;
    img->height = height;
    img->data = (unsigned char**)malloc(height * sizeof(unsigned char*));
    for(int i=0; i<height; i++) {
        img->data[i] = (unsigned char*)calloc(width, sizeof(unsigned char));
    }
    return img;
}

void free_image(Image *img) {
    if (!img) return;
    for(int i=0; i<img->height; i++) {
        free(img->data[i]);
    }
    free(img->data);
    free(img);
}

Image* clone_image(Image *src) {
    Image *dst = create_image(src->width, src->height);
    for(int y=0; y<src->height; y++) {
        memcpy(dst->data[y], src->data[y], src->width * sizeof(unsigned char));
    }
    return dst;
}

Image* read_jpeg(const char* filename) {
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE* infile = fopen(filename, "rb");
    if (!infile) {
        fprintf(stderr, "Error opening %s\n", filename);
        return NULL;
    }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, infile);
    jpeg_read_header(&cinfo, 1);
    jpeg_start_decompress(&cinfo);

    int w = cinfo.output_width;
    int h = cinfo.output_height;
    int ch = cinfo.output_components;

    Image *img = create_image(w, h);
    unsigned char* row_buffer = (unsigned char*)malloc(w * ch);

    while (cinfo.output_scanline < h) {
        int y = cinfo.output_scanline;
        jpeg_read_scanlines(&cinfo, &row_buffer, 1);
        for (int x = 0; x < w; x++) {
            // Convert to Grayscale immediately
            unsigned char R = row_buffer[x * ch];
            unsigned char G = row_buffer[x * ch + 1];
            unsigned char B = row_buffer[x * ch + 2];
            img->data[y][x] = (unsigned char)(0.299 * R + 0.587 * G + 0.114 * B);
        }
    }

    free(row_buffer);
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(infile);
    return img;
}

void write_jpeg(const char* filename, Image *img, int quality) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE* outfile = fopen(filename, "wb");
    if (!outfile) {
        fprintf(stderr, "Error creating %s\n", filename);
        return;
    }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, outfile);

    cinfo.image_width = img->width;
    cinfo.image_height = img->height;
    cinfo.input_components = 1;
    cinfo.in_color_space = JCS_GRAYSCALE;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW row_pointer[1];
    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0] = img->data[cinfo.next_scanline];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    fclose(outfile);
    printf("Saved: %s\n", filename);
}

Image* apply_convolution(Image *src, int kernel[3][3], int divisor, int offset) {
    Image *dst = create_image(src->width, src->height);
    int h = src->height;
    int w = src->width;

    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            int sum = 0;
            for (int ky = -1; ky <= 1; ky++) {
                for (int kx = -1; kx <= 1; kx++) {
                    sum += kernel[ky+1][kx+1] * src->data[y+ky][x+kx];
                }
            }
            if (divisor != 0) sum /= divisor;
            sum += offset;
            
            if (sum < 0) sum = 0;
            if (sum > 255) sum = 255;
            dst->data[y][x] = (unsigned char)sum;
        }
    }
    return dst;
}

Image* filter_sobel(Image *src) {
    Image *gx = apply_convolution(src, sobel_h_kernel, 1, 0);
    Image *gy = apply_convolution(src, sobel_v_kernel, 1, 0);
    Image *mag = create_image(src->width, src->height);

    for (int y = 0; y < src->height; y++) {
        for (int x = 0; x < src->width; x++) {
            double val = sqrt(pow(gx->data[y][x], 2) + pow(gy->data[y][x], 2));
            if (val > 255) val = 255;
            mag->data[y][x] = (unsigned char)val;
        }
    }
    free_image(gx);
    free_image(gy);
    return mag;
}

Image* filter_emboss(Image *src) {
    return apply_convolution(src, emboss_kernel, 1, 128);
}

Image* filter_blur(Image *src) {
    return apply_convolution(src, blur_kernel, 16, 0);
}

Image* crop_image(Image *src, int x_start, int crop_w) {
    if (x_start < 0) x_start = 0;
    if (x_start + crop_w > src->width) x_start = src->width - crop_w;

    Image *dst = create_image(crop_w, src->height);
    for(int y=0; y<src->height; y++) {
        for(int x=0; x<crop_w; x++) {
            dst->data[y][x] = src->data[y][x_start + x];
        }
    }
    return dst;
}


int detect_rail_start(Image *src, int rail_w) {
    float min_avg_var = FLT_MAX;
    int best_x = 0;

    // Pre-calculate column variances
    float *col_vars = (float*)calloc(src->width, sizeof(float));
    for(int x=0; x<src->width; x++) {
        float sum = 0, sq_sum = 0;
        for(int y=0; y<src->height; y++) {
            sum += src->data[y][x];
            sq_sum += src->data[y][x] * src->data[y][x];
        }
        float mean = sum / src->height;
        col_vars[x] = (sq_sum / src->height) - (mean * mean);
    }

    for (int x = 0; x <= src->width - rail_w; x++) {
        float window_var_sum = 0;
        for(int k=0; k<rail_w; k++) {
            window_var_sum += col_vars[x+k];
        }
        if (window_var_sum < min_avg_var) {
            min_avg_var = window_var_sum;
            best_x = x;
        }
    }
    free(col_vars);
    return best_x;
}

int main() {
    struct stat st = {0};
    if (stat(OUTPUT_DIR, &st) == -1) {
        mkdir(OUTPUT_DIR, 0755);
    }

    const char* input_files[] = {
        "/home/amrut/Downloads/topviewtrack.jpeg"
    };
    int num_files = 1;
    char out_path[512];

    for(int i=0; i<num_files; i++) {
        printf("\nProcessing %s...\n", input_files[i]);
        
        char *base_name = strrchr(input_files[i], '/');
        base_name = (base_name) ? base_name + 1 : (char*)input_files[i];
        char name_only[100];
        strncpy(name_only, base_name, 99);
        char *dot = strrchr(name_only, '.');
        if(dot) *dot = '\0';

        Image *original = read_jpeg(input_files[i]);
        if(!original) continue;

        snprintf(out_path, sizeof(out_path), "%s/%s_01_original.jpg", OUTPUT_DIR, name_only);
        write_jpeg(out_path, original, 95);

        Image *full_emboss = filter_emboss(original);
        snprintf(out_path, sizeof(out_path), "%s/%s_02_full_emboss.jpg", OUTPUT_DIR, name_only);
        write_jpeg(out_path, full_emboss, 90);
        free_image(full_emboss);

        int rail_x = detect_rail_start(original, RAIL_WIDTH_ESTIMATE);
        printf("Detected rail at X: %d\n", rail_x);

        Image *crop_raw = crop_image(original, rail_x, RAIL_WIDTH_ESTIMATE);
        snprintf(out_path, sizeof(out_path), "%s/%s_03_crop_raw.jpg", OUTPUT_DIR, name_only);
        write_jpeg(out_path, crop_raw, 95);

        Image *crop_blurred = filter_blur(crop_raw);

        Image *crop_sobel = filter_sobel(crop_blurred);
        snprintf(out_path, sizeof(out_path), "%s/%s_04_crop_sobel.jpg", OUTPUT_DIR, name_only);
        write_jpeg(out_path, crop_sobel, 90);

        Image *crop_emboss = filter_emboss(crop_raw);
        snprintf(out_path, sizeof(out_path), "%s/%s_05_crop_emboss.jpg", OUTPUT_DIR, name_only);
        write_jpeg(out_path, crop_emboss, 90);

        free_image(original);
        free_image(crop_raw);
        free_image(crop_blurred);
        free_image(crop_sobel);
        free_image(crop_emboss);
    }
    return 0;
}
