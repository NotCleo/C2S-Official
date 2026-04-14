#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

// Import your custom 64-layer YOLOv8 network definitions
#include "model_arch.h"

// --- MEMORY MAP FOR ARTIX-7 ---
#define RESERVED_RAM_BASE   0x8E000000 
#define RESERVED_RAM_SIZE   (16 * 1024 * 1024) 

#define WEIGHTS_OFFSET      0x00000000 
#define INPUT_OFFSET        0x00500000 

#define DMA_CTRL_BASE       0x1011A000
#define NPU_CTRL_BASE       0x10140000

// --- NPU REGISTERS ---
#define MM2S_DMACR          0x00 
#define MM2S_DMASR          0x04 
#define MM2S_SA             0x18 
#define MM2S_LENGTH         0x28 

#define NPU_CFG_CIN         0x00
#define NPU_CFG_COUT        0x04
#define NPU_CFG_HIN         0x08
#define NPU_CFG_WIN         0x0C
#define NPU_CFG_HOUT        0x10
#define NPU_CFG_WOUT        0x14
#define NPU_CFG_K           0x18
#define NPU_CFG_STRIDE      0x1C
#define NPU_CFG_PAD         0x20
#define NPU_CFG_QSCALE      0x24
#define NPU_CFG_QSHIFT      0x28
#define NPU_CFG_RELU        0x2C
#define NPU_LAYER_START     0x30
#define NPU_ROUTING_CTRL    0x3C

// --- IMAGE SETTINGS ---
#define FULL_W 320
#define FULL_H 240
#define PATCH_SIZE 64
#define IN_CHANNELS 3

#define PATCH_OUT_SIZE 2
#define FINAL_CHANNELS 4 

float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

int main() {
    printf("\n====================================================\n");
    printf("   ARTIX-7 NPU - YOLOv8 HYBRID TILING ENGINE\n");
    printf("====================================================\n\n");

    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) return -1;

    void *ram_map = mmap(NULL, RESERVED_RAM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, RESERVED_RAM_BASE);
    volatile uint32_t *dma_ctrl = (volatile uint32_t *)mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, DMA_CTRL_BASE);
    volatile uint32_t *npu_ctrl = (volatile uint32_t *)mmap(NULL, 256 * 1024, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, NPU_CTRL_BASE);

    printf("[TRACE] Hard resetting Transmit DMA...\n");
    dma_ctrl[MM2S_DMACR / 4] = 0x04; 
    while (dma_ctrl[MM2S_DMACR / 4] & 0x04);
    dma_ctrl[MM2S_DMACR / 4] = 0x01; 

    // --- 1. LOAD FULL IMAGE & WEIGHTS ---
    printf("[TRACE] Loading FULL 320x240 Image into ARM RAM...\n");
    uint8_t *full_img = (uint8_t *)malloc(FULL_W * FULL_H * IN_CHANNELS);
    FILE *fi = fopen("input_test.bin", "rb");
    if (fi) { fread(full_img, 1, FULL_W * FULL_H * IN_CHANNELS, fi); fclose(fi); }
    else { printf("[ERROR] Could not find input_test.bin\n"); return -1; }

    FILE *fw = fopen("weights.bin", "rb");
    if (fw) { fread(ram_map + WEIGHTS_OFFSET, 1, 4 * 1024 * 1024, fw); fclose(fw); }
    else { printf("[ERROR] Could not find weights.bin\n"); return -1; }

    // --- 2. PREPARE THE FULL OUTPUT CANVAS ---
    int final_w = FULL_W / 32;
    int final_h = FULL_H / 32;
    float *stitched_output = (float *)calloc(final_w * final_h * FINAL_CHANNELS, sizeof(float));

    int grid_w = (FULL_W + PATCH_SIZE - 1) / PATCH_SIZE; 
    int grid_h = (FULL_H + PATCH_SIZE - 1) / PATCH_SIZE; 

    printf("[TRACE] Executing Hardware Partitioning (%dx%d Grid)...\n", grid_w, grid_h);

    // --- 3. HARDWARE TILING LOOP ---
    for (int ty = 0; ty < grid_h; ty++) {
        for (int tx = 0; tx < grid_w; tx++) {
            printf("\n[*] Processing Tile [%d, %d]\n", tx, ty);
            printf("--------------------------------------------------\n");
            
            // A. Extract 64x64 patch from full image
            uint8_t *dma_buffer = (uint8_t *)(ram_map + INPUT_OFFSET);
            memset(dma_buffer, 0, PATCH_SIZE * PATCH_SIZE * IN_CHANNELS); 
            
            int start_x = tx * PATCH_SIZE;
            int start_y = ty * PATCH_SIZE;
            
            for (int y = 0; y < PATCH_SIZE; y++) {
                if (start_y + y >= FULL_H) continue;
                for (int x = 0; x < PATCH_SIZE; x++) {
                    if (start_x + x >= FULL_W) continue;
                    for (int c = 0; c < IN_CHANNELS; c++) {
                        int full_idx = ((start_y + y) * FULL_W + (start_x + x)) * IN_CHANNELS + c;
                        int patch_idx = (y * PATCH_SIZE + x) * IN_CHANNELS + c;
                        dma_buffer[patch_idx] = full_img[full_idx];
                    }
                }
            }

            // B. Route DMA to Act Buffer and Send Patch
            npu_ctrl[NPU_ROUTING_CTRL / 4] = 0x02; // Reset stream
            npu_ctrl[NPU_ROUTING_CTRL / 4] = 0x00; 

            npu_ctrl[NPU_ROUTING_CTRL / 4] = 0x01; // Route to Activation BRAM
            dma_ctrl[MM2S_SA / 4] = RESERVED_RAM_BASE + INPUT_OFFSET;
            dma_ctrl[MM2S_LENGTH / 4] = PATCH_SIZE * PATCH_SIZE * IN_CHANNELS; 
            while ((dma_ctrl[MM2S_DMASR / 4] & 0x02) == 0); 

            // C. Execute all 64 layers for this specific patch
            int curr_w = PATCH_SIZE;
            int curr_h = PATCH_SIZE;
            
            for (int i = 0; i < NUM_LAYERS; i++) {
                layer_config_t l = model_layers[i];
                int next_w = (curr_w + 2 * l.pad - l.k_w) / l.stride + 1;
                int next_h = (curr_h + 2 * l.pad - l.k_h) / l.stride + 1;

                // Print layer progress
                if (i % 8 == 0 || i == NUM_LAYERS - 1) {
                    printf("  -> Layer %2d: %dx%dx%d  =>  %dx%dx%d\n", 
                           i, curr_w, curr_h, l.in_ch, next_w, next_h, l.out_ch);
                }

                if (l.weight_size > 0) {
                    npu_ctrl[NPU_ROUTING_CTRL / 4] = 0x00; 
                    dma_ctrl[MM2S_SA / 4] = RESERVED_RAM_BASE + WEIGHTS_OFFSET + l.weight_offset;
                    dma_ctrl[MM2S_LENGTH / 4] = l.weight_size; 
                    while ((dma_ctrl[MM2S_DMASR / 4] & 0x02) == 0); 
                }

                npu_ctrl[NPU_CFG_CIN / 4]    = l.in_ch;   
                npu_ctrl[NPU_CFG_COUT / 4]   = l.out_ch;  
                npu_ctrl[NPU_CFG_HIN / 4]    = curr_h;  
                npu_ctrl[NPU_CFG_WIN / 4]    = curr_w;  
                npu_ctrl[NPU_CFG_HOUT / 4]   = next_h;  
                npu_ctrl[NPU_CFG_WOUT / 4]   = next_w;  
                npu_ctrl[NPU_CFG_K / 4]      = l.k_h;   
                npu_ctrl[NPU_CFG_STRIDE / 4] = l.stride;   
                npu_ctrl[NPU_CFG_PAD / 4]    = l.pad;     
                
                // Note: Ensure your header provides qscale/qshift if your model relies on them!
                npu_ctrl[NPU_CFG_QSCALE / 4] = 1; 
                npu_ctrl[NPU_CFG_QSHIFT / 4] = 0; 
                npu_ctrl[NPU_CFG_RELU / 4]   = 1;   

                // Fire NPU
                npu_ctrl[NPU_LAYER_START / 4] = 1; 
                npu_ctrl[NPU_LAYER_START / 4] = 0; 
                
                // Fast poll loop instead of static usleep
                // Assuming NPU_LAYER_START clears itself when done, or you have an interrupt
                usleep(5000); 
                
                curr_w = next_w;
                curr_h = next_h;
            }

            // D. Pull the 2x2 hardware output and stitch it into the canvas
            volatile int8_t *npu_act_buffer = (volatile int8_t *)npu_ctrl + 0x20000;
            int out_start_x = tx * PATCH_OUT_SIZE;
            int out_start_y = ty * PATCH_OUT_SIZE;

            for (int y = 0; y < PATCH_OUT_SIZE; y++) {
                if (out_start_y + y >= final_h) continue;
                for (int x = 0; x < PATCH_OUT_SIZE; x++) {
                    if (out_start_x + x >= final_w) continue;
                    
                    for (int c = 0; c < FINAL_CHANNELS; c++) {
                        int st_idx = ((out_start_y + y) * final_w + (out_start_x + x)) * FINAL_CHANNELS + c;
                        int hd_idx = (y * PATCH_OUT_SIZE + x) * FINAL_CHANNELS + c;
                        
                        stitched_output[st_idx] = (float)npu_act_buffer[hd_idx]; 
                    }
                }
            }
        }
    }

    // --- 4. SOFTWARE DECODER: EXTRACT BOUNDING BOXES ---
    printf("\n[TRACE] Tiling Complete. Decoding YOLOv8 Confidence Scores...\n");
    FILE *f_out = fopen("coordinates.csv", "w");
    if (!f_out) return -1;
    
    fprintf(f_out, "x_min, y_min, x_max, y_max, confidence\n");
    int boxes_found = 0;
    
    for (int y = 0; y < final_h; y++) {
        for (int x = 0; x < final_w; x++) {
            int idx = (y * final_w + x) * FINAL_CHANNELS;
            
            // Assume the 4 channels are [Objectness, dX, dY, dWidth]
            float raw_obj = stitched_output[idx + 0];
            float raw_dx  = stitched_output[idx + 1];
            float raw_dy  = stitched_output[idx + 2];
            float raw_dw  = stitched_output[idx + 3];

            float obj_score = sigmoid(raw_obj);
            
            // Look for cars/people with high confidence
            if (obj_score > 0.50f) {
                float center_x = (x + sigmoid(raw_dx)) * 32.0f;
                float center_y = (y + sigmoid(raw_dy)) * 32.0f;
                
                float box_w = expf(raw_dw) * 32.0f; 
                float box_h = box_w * 1.5f; // People are generally taller than they are wide
                
                int x_min = (int)(center_x - (box_w / 2.0f));
                int y_min = (int)(center_y - (box_h / 2.0f));
                int x_max = (int)(center_x + (box_w / 2.0f));
                int y_max = (int)(center_y + (box_h / 2.0f));

                fprintf(f_out, "%d, %d, %d, %d, %.3f\n", x_min, y_min, x_max, y_max, obj_score);
                boxes_found++;
            }
        }
    }
    
    if (boxes_found == 0) {
        fprintf(f_out, "0, 0, 0, 0, 0.000\n");
    }

    fclose(f_out);
    printf("[SUCCESS] Found %d targets! Coordinates saved to 'coordinates.csv'.\n", boxes_found);

    free(full_img);
    free(stitched_output);
    munmap(ram_map, RESERVED_RAM_SIZE);
    munmap((void*)dma_ctrl, 4096);
    munmap((void*)npu_ctrl, 256 * 1024);
    close(mem_fd);
    return 0;
}
