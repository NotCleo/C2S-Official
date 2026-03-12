#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

#define NPU_PHYS_ADDR       0x10140000
#define DMA_PHYS_ADDR       0x1011A000
#define DMA_RAM_PHYS_ADDR   0x8F000000 
#define DMA_RAM_SIZE        0x01000000 

typedef struct {
    volatile uint32_t MM2S_CR;       volatile uint32_t MM2S_SR;
    volatile uint32_t _pad0[4];      volatile uint32_t MM2S_SA;
    volatile uint32_t _pad1;         volatile uint32_t _pad2[2];
    volatile uint32_t MM2S_LENGTH;   volatile uint32_t _pad3[1];
    volatile uint32_t S2MM_CR;       volatile uint32_t S2MM_SR;
    volatile uint32_t _pad4[4];      volatile uint32_t S2MM_DA;
    volatile uint32_t _pad5;         volatile uint32_t _pad6[2];
    volatile uint32_t S2MM_LENGTH;
} axi_dma_regs_t;

static double get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000.0) + (ts.tv_nsec / 1000000.0);
}

int main(void) {
    printf("\n====================================================\n");
    printf("  ARCHWAY NPU - MAXIMUM PERFORMANCE MODE\n");
    printf("====================================================\n\n");

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) return -1;

    volatile uint32_t* NPU = (volatile uint32_t*)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, NPU_PHYS_ADDR);
    axi_dma_regs_t* DMA = (axi_dma_regs_t*)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, DMA_PHYS_ADDR);
    void* dma_ram_virt = mmap(NULL, DMA_RAM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, DMA_RAM_PHYS_ADDR);

    uint32_t* virt_weights = (uint32_t*)((uint8_t*)dma_ram_virt + 0x00000);
    uint32_t* virt_pixels  = (uint32_t*)((uint8_t*)dma_ram_virt + 0x10000);
    uint32_t* virt_rx      = (uint32_t*)((uint8_t*)dma_ram_virt + 0x20000);

    uint32_t phys_weights = DMA_RAM_PHYS_ADDR + 0x00000;
    uint32_t phys_pixels  = DMA_RAM_PHYS_ADDR + 0x10000;
    uint32_t phys_rx      = DMA_RAM_PHYS_ADDR + 0x20000;

    for (int i = 0; i < 4096; i++) virt_weights[i] = 0x01010101;
    for (int i = 0; i < 32; i++)   virt_pixels[i]  = 0x01010101;
    for (int i = 0; i < 128; i++)  virt_rx[i]      = 0xAAAAAAAA;

    DMA->MM2S_CR = 0x04; while (DMA->MM2S_CR & 0x04);
    DMA->S2MM_CR = 0x04; while (DMA->S2MM_CR & 0x04);

    printf("[*] RAM and Hardware Initialized. Executing...\n");

    // ==========================================================
    // CRITICAL PATH - NO PRINTF ALLOWED
    // ==========================================================
    double t_start = get_time_ms();

    // 1. Weights Bank 0
    NPU[15] = 0x02; NPU[15] = 0x00; 
    DMA->MM2S_CR |= 0x01; DMA->MM2S_SA = phys_weights; DMA->MM2S_LENGTH = 2048 * 4;  
    while (!(DMA->MM2S_SR & 0x02));

    // 2. Weights Bank 1
    DMA->MM2S_SA = phys_weights + (2048 * 4); DMA->MM2S_LENGTH = 2048 * 4;  
    while (!(DMA->MM2S_SR & 0x02));

    // 3. Pixels
    NPU[15] = 0x02; NPU[15] = 0x01; 
    DMA->MM2S_SA = phys_pixels; DMA->MM2S_LENGTH = 32 * 4;
    while (!(DMA->MM2S_SR & 0x02));

    // 4. Compute
    NPU[0] = 128; NPU[1] = 128; NPU[2] = 1; NPU[3] = 1; NPU[4] = 1; NPU[5] = 1; NPU[6] = 1; NPU[7] = 1; NPU[8] = 0; NPU[9] = 0x7FFF; NPU[10] = 15; NPU[11] = 1;
    NPU[12] = 1; volatile uint32_t rb = NPU[12]; (void)rb; NPU[12] = 0; 
    while (NPU[14] != 0); 

    // 5. Unload
    DMA->S2MM_CR |= 0x01; DMA->S2MM_DA  = phys_rx; DMA->S2MM_LENGTH = 512; 
    NPU[13] = 16384; NPU[14] = 128; NPU[15] = 0x05;  
    while (!(DMA->S2MM_SR & 0x02));
    NPU[15] = 0x01; 

    double t_end = get_time_ms();
    // ==========================================================
    // CRITICAL PATH END
    // ==========================================================

    int hits = 0;
    for (int i = 0; i < 16; i++) {
        if ((virt_rx[i] & 0xFF) == 0x7F) hits++;
    }

    if (hits == 16) {
        printf("\n[SUCCESS] Execution complete!\n");
        printf("End-to-End Latency: %.3f ms\n", t_end - t_start);
    } else {
        printf("\n[WARNING] Math mismatch.\n");
    }

    munmap(dma_ram_virt, DMA_RAM_SIZE); munmap((void*)DMA, 0x1000); munmap((void*)NPU, 0x1000);
    close(fd);
    return 0;
}
