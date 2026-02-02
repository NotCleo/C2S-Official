#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>a
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <math.h>
#include <time.h>

// ==========================================
// CONFIGURATION
// ==========================================
#define I2C_PHYS_BASE   0x10116000UL 
#define MAP_SIZE        4096UL
#define MAP_MASK        (MAP_SIZE - 1)
#define MPU6050_ADDR    0x68

// AXI IIC Registers
#define REG_CR          0x100
#define REG_SR          0x104
#define REG_TX_FIFO     0x108
#define REG_RX_FIFO     0x10C

// MPU6050 Registers
#define REG_PWR_MGMT_1  0x6B
#define REG_ACCEL_X     0x3B
#define REG_GYRO_Z      0x47 
#define REG_ACCEL_CFG   0x1C
#define REG_GYRO_CFG    0x1B

volatile uint32_t *i2c_base = NULL;
#define I2C_REG(off) (*(volatile uint32_t *)((uint8_t *)i2c_base + (off)))

// ==========================================
// OPTIMIZED I2C DRIVER
// ==========================================
void *map_mem(uint32_t phys) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if(fd < 0) { perror("mem"); exit(1); }
    void *map = mmap(0, MAP_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, phys & ~MAP_MASK);
    close(fd);
    return (void *)((uint8_t *)map + (phys & MAP_MASK));
}

void i2c_init() {
    I2C_REG(0x40) = 0x0A; // Reset
    usleep(1000);
    I2C_REG(REG_CR) = 0x01; // Enable
}

void i2c_write(uint8_t reg, uint8_t val) {
    I2C_REG(REG_TX_FIFO) = 0x100 | (MPU6050_ADDR << 1); 
    I2C_REG(REG_TX_FIFO) = reg;                  
    I2C_REG(REG_TX_FIFO) = 0x200 | val;          
    usleep(1000); // Small wait for setup
}

// Optimized Read (Your requested modification)
void i2c_read_burst(uint8_t reg, uint8_t *data, int len) {
    I2C_REG(REG_TX_FIFO) = 0x100 | (MPU6050_ADDR << 1); 
    I2C_REG(REG_TX_FIFO) = reg;                  
    I2C_REG(REG_TX_FIFO) = 0x100 | (MPU6050_ADDR << 1) | 1; 
    I2C_REG(REG_TX_FIFO) = 0x200 | len;          
    
    for(int i=0; i<len; i++) {
        int timeout = 0;
        // Busy wait (Fast poll)
        while((I2C_REG(REG_SR) & 0x40) && timeout++ < 1000000) {
             for(volatile int k=0; k<10; k++); // Tiny CPU burn
        }
        
        if(timeout >= 1000000) data[i] = 0;
        else data[i] = (uint8_t)(I2C_REG(REG_RX_FIFO) & 0xFF);
    }
}

// ==========================================
// MAIN LOOP
// ==========================================
int main() {
    i2c_base = (volatile uint32_t *)map_mem(I2C_PHYS_BASE);
    i2c_init();
    
    // --- SETUP MPU6050 ---
    printf("Configuring MPU6050...\n");
    i2c_write(REG_PWR_MGMT_1, 0x00); // Wake up
    i2c_write(REG_ACCEL_CFG, 0x00);  // Accel +/- 2g
    i2c_write(REG_GYRO_CFG, 0x00);   // Gyro +/- 250 dps
    
    FILE *fp = fopen("mpu_flight.csv", "w");
    if(!fp) { perror("File"); return -1; }
    
    // Header
    fprintf(fp, "time_s,ax,ay,az,pitch,roll,yaw\n");
    
    struct timespec start, now, last_t;
    clock_gettime(CLOCK_MONOTONIC, &start);
    last_t = start;
    
    printf("Recording High-Speed Data for 10 seconds...\n");
    
    double yaw_accumulated = 0.0;
    int samples = 0;
    
    while(1) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec)/1e9;
        
        // Calculate dt for Gyro integration (Yaw)
        double dt = (now.tv_sec - last_t.tv_sec) + (now.tv_nsec - last_t.tv_nsec)/1e9;
        last_t = now;
        
        if(elapsed >= 10.0) break;
        
        // --- READ 14 BYTES (Accel + Temp + Gyro) ---
        // Registers 0x3B to 0x48 contain all data sequentially
        uint8_t d[14];
        i2c_read_burst(0x3B, d, 14);
        
        int16_t ax = (int16_t)((d[0] << 8) | d[1]);
        int16_t ay = (int16_t)((d[2] << 8) | d[3]);
        int16_t az = (int16_t)((d[4] << 8) | d[5]);
        // Skip Temp (d[6], d[7])
        // Gyro
        int16_t gx = (int16_t)((d[8] << 8) | d[9]);
        int16_t gy = (int16_t)((d[10] << 8) | d[11]);
        int16_t gz = (int16_t)((d[12] << 8) | d[13]);
        
        // --- MATH ---
        double x = (double)ax; double y = (double)ay; double z = (double)az;
        
        // Pitch/Roll from Accelerometer
        double pitch = -(atan2(x, sqrt(y*y + z*z)) * 180.0) / M_PI;
        double roll  = (atan2(y, z) * 180.0) / M_PI;
        
        // Yaw from Gyro Z Integration (Relative Heading)
        // Sensitivity for +/- 250dps is 131 LSB/deg/s
        double gyro_z_dps = gz / 131.0;
        
        // Simple filter to remove drift when stationary
        if(fabs(gyro_z_dps) > 1.0) { 
            yaw_accumulated += gyro_z_dps * dt;
        }
        
        // --- LOG ---
        fprintf(fp, "%.4f,%d,%d,%d,%.2f,%.2f,%.2f\n", elapsed, ax, ay, az, pitch, roll, yaw_accumulated);
        
        samples++;
        
        // No usleep() here for maximum speed. 
        // We rely on I2C bus speed to limit us naturally.
    }
    
    printf("Done. Saved %d samples (~%d Hz).\n", samples, samples/10);
    fclose(fp);
    return 0;
}
