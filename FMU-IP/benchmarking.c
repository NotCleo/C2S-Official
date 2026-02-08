#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#define MATH_PHY_ADDR   0x1011A000
#define MAP_SIZE        4096
#define NUM_SAMPLES     100

#define OFF_DIV_A       0x00
#define OFF_DIV_B       0x04
#define OFF_DIV_RES     0x08
#define OFF_ATAN_PACK   0x10
#define OFF_ATAN_RES    0x14
#define OFF_SINCOS_IN   0x20
#define OFF_SINCOS_RES  0x24
#define OFF_SQRT_IN     0x30
#define OFF_SQRT_RES    0x34

int32_t input_A[NUM_SAMPLES];
int32_t input_B[NUM_SAMPLES]; 
int32_t results_cpu[NUM_SAMPLES]; 
int32_t results_fpga[NUM_SAMPLES];

long long get_time_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

void generate_data() {
    srand(time(NULL));
    for(int i=0; i<NUM_SAMPLES; i++) {
        input_A[i] = (rand() % 20000) - 10000;
        input_B[i] = (rand() % 20000) - 10000;
        if(input_B[i] == 0) input_B[i] = 1; 
    }
}

int main() {
    int fd;
    volatile uint32_t *reg;
    long long start, end;
    long long t_cpu_div, t_fpga_div;
    long long t_cpu_atan, t_fpga_atan;
    long long t_cpu_sincos, t_fpga_sincos;
    long long t_cpu_sqrt, t_fpga_sqrt;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) {
        perror("Error opening /dev/mem");
        return -1;
    }
    reg = (volatile uint32_t *)mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MATH_PHY_ADDR);
    if (reg == MAP_FAILED) {
        perror("mmap failed");
        close(fd);
        return -1;
    }

    generate_data();
    printf("VENCHMARK: CPU vs FPGA (%d samples)\n\n", NUM_SAMPLES);

    start = get_time_ns();
    for(int i=0; i<NUM_SAMPLES; i++) {
        results_cpu[i] = input_A[i] / input_B[i];
    }
    end = get_time_ns();
    t_cpu_div = end - start;

    start = get_time_ns();
    for(int i=0; i<NUM_SAMPLES; i++) {
        reg[OFF_DIV_A/4] = input_A[i];
        reg[OFF_DIV_B/4] = input_B[i];
        results_fpga[i] = reg[OFF_DIV_RES/4];
    }
    end = get_time_ns();
    t_fpga_div = end - start;

    start = get_time_ns();
    for(int i=0; i<NUM_SAMPLES; i++) {
        results_cpu[i] = (int32_t)(atan2((double)input_A[i], (double)input_B[i]) * 1000.0);
    }
    end = get_time_ns();
    t_cpu_atan = end - start;

    start = get_time_ns();
    for(int i=0; i<NUM_SAMPLES; i++) {
        int16_t y = (int16_t)input_A[i];
        int16_t x = (int16_t)input_B[i];
        reg[OFF_ATAN_PACK/4] = ((uint32_t)y << 16) | (uint16_t)x;
        results_fpga[i] = reg[OFF_ATAN_RES/4];
    }
    end = get_time_ns();
    t_fpga_atan = end - start;


    start = get_time_ns();
    for(int i=0; i<NUM_SAMPLES; i++) {
        double angle = input_A[i] * 0.001; 
        results_cpu[i] = (int32_t)((sin(angle) + cos(angle)) * 1000.0); 
    }
    end = get_time_ns();
    t_cpu_sincos = end - start;

    start = get_time_ns();
    for(int i=0; i<NUM_SAMPLES; i++) {
        reg[OFF_SINCOS_IN/4] = input_A[i]; 
        results_fpga[i] = reg[OFF_SINCOS_RES/4]; 
    }
    end = get_time_ns();
    t_fpga_sincos = end - start;


    for(int i=0; i<NUM_SAMPLES; i++) if(input_A[i] < 0) input_A[i] = -input_A[i];

    start = get_time_ns();
    for(int i=0; i<NUM_SAMPLES; i++) {
        results_cpu[i] = (int32_t)sqrt((double)input_A[i]);
    }
    end = get_time_ns();
    t_cpu_sqrt = end - start;

    start = get_time_ns();
    for(int i=0; i<NUM_SAMPLES; i++) {
        reg[OFF_SQRT_IN/4] = input_A[i];
        results_fpga[i] = reg[OFF_SQRT_RES/4];
    }
    end = get_time_ns();
    t_fpga_sqrt = end - start;


    #define CALC_OPS(t) ((double)(NUM_SAMPLES * 1000000000LL) / (double)(t))

    printf("%-10s | %-13s | %-12s | %-13s | %-12s | %s\n", 
           "Operation", "CPU Time(ns)", "CPU OPS", "FPGA Time(ns)", "FPGA OPS", "Speedup");
    
    printf("%-10s | %13lld | %12.0f | %13lld | %12.0f | %.2fx\n", 
           "DIVISION", 
           t_cpu_div, CALC_OPS(t_cpu_div), 
           t_fpga_div, CALC_OPS(t_fpga_div), 
           (double)t_cpu_div / t_fpga_div);
           
    printf("%-10s | %13lld | %12.0f | %13lld | %12.0f | %.2fx\n", 
           "ATAN2", 
           t_cpu_atan, CALC_OPS(t_cpu_atan), 
           t_fpga_atan, CALC_OPS(t_fpga_atan), 
           (double)t_cpu_atan / t_fpga_atan);
           
    printf("%-10s | %13lld | %12.0f | %13lld | %12.0f | %.2fx\n", 
           "SIN/COS", 
           t_cpu_sincos, CALC_OPS(t_cpu_sincos), 
           t_fpga_sincos, CALC_OPS(t_fpga_sincos), 
           (double)t_cpu_sincos / t_fpga_sincos);
           
    printf("%-10s | %13lld | %12.0f | %13lld | %12.0f | %.2fx\n", 
           "SQRT", 
           t_cpu_sqrt, CALC_OPS(t_cpu_sqrt), 
           t_fpga_sqrt, CALC_OPS(t_fpga_sqrt), 
           (double)t_cpu_sqrt / t_fpga_sqrt);

    munmap((void*)reg, MAP_SIZE);
    close(fd);
    return 0;
}
