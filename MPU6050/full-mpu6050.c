#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <math.h>
#include <time.h>

// ==========================================
// 1. HARDWARE MAPPING
// ==========================================
#define I2C_PHYS_BASE   0x10116000UL
#define MAP_SIZE        4096UL
#define MAP_MASK        (MAP_SIZE - 1)

volatile uint32_t *i2c_base_virt = NULL;

#define I2C_REG(offset)  (*(volatile uint32_t *)((uint8_t *)i2c_base_virt + (offset)))

// AXI IIC Register Offsets
#define REG_CR          0x100
#define REG_SR          0x104
#define REG_TX_FIFO     0x108
#define REG_RX_FIFO     0x10C

// MPU6050 Registers
#define MPU6050_ADDR    0x68
#define PWR_MGMT_1      0x6B
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_XOUT_H    0x3B
#define GYRO_XOUT_H     0x43

// ==========================================
// 2. I2C DRIVERS
// ==========================================
void *map_physical_memory(uint32_t phys_addr) {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) { perror("Error opening /dev/mem"); exit(EXIT_FAILURE); }
    void *mapped_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, phys_addr & ~MAP_MASK);
    if (mapped_base == MAP_FAILED) { perror("Error mapping memory"); close(mem_fd); exit(EXIT_FAILURE); }
    close(mem_fd);
    return (void *)((uint8_t *)mapped_base + (phys_addr & MAP_MASK));
}

void i2c_init() {
    I2C_REG(0x40) = 0x0A; 
    usleep(1000);
    I2C_REG(REG_CR) = 0x01; 
}

void i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t val) {
    I2C_REG(REG_TX_FIFO) = 0x100 | (dev_addr << 1); 
    I2C_REG(REG_TX_FIFO) = reg_addr;                
    I2C_REG(REG_TX_FIFO) = 0x200 | val;             
    usleep(5000); 
}

void i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, int length) {
    I2C_REG(REG_TX_FIFO) = 0x100 | (dev_addr << 1);
    I2C_REG(REG_TX_FIFO) = reg_addr;
    I2C_REG(REG_TX_FIFO) = 0x100 | (dev_addr << 1) | 1;
    I2C_REG(REG_TX_FIFO) = 0x200 | length;

    for(int i=0; i<length; i++) {
        int timeout = 0;
        while ((I2C_REG(REG_SR) & 0x40) && (timeout < 100000)) {
            timeout++;
            usleep(10);
        }
        if (timeout >= 100000) data[i] = 0;
        else data[i] = (uint8_t)(I2C_REG(REG_RX_FIFO) & 0xFF);
    }
}

// ==========================================
// 3. MPU6050 FUNCTIONS
// ==========================================
void mpu_init() {
    printf("Initializing MPU6050...\n");
    // Wake up
    i2c_write_reg(MPU6050_ADDR, PWR_MGMT_1, 0x00);
    usleep(10000);
    // Config Accel +/- 2g
    i2c_write_reg(MPU6050_ADDR, ACCEL_CONFIG, 0x00);
    // Config Gyro +/- 250 deg/s
    i2c_write_reg(MPU6050_ADDR, GYRO_CONFIG, 0x00);
    usleep(10000);
}

void mpu_read_all(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buffer[14];
    // Burst read 14 bytes starting from ACCEL_XOUT_H (0x3B)
    // 0-5: Accel, 6-7: Temp, 8-13: Gyro
    i2c_read_bytes(MPU6050_ADDR, ACCEL_XOUT_H, buffer, 14);

    *ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    *ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    *az = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Skip Temp (buffer[6], buffer[7])
    
    *gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    *gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    *gz = (int16_t)((buffer[12] << 8) | buffer[13]);
}

// ==========================================
// 4. MAIN LOOP
// ==========================================
int main() {
    printf("[INIT] 6-DOF Sensor Fusion...\n");
    i2c_base_virt = (volatile uint32_t *)map_physical_memory(I2C_PHYS_BASE);

    i2c_init();
    mpu_init();

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    double pitch, roll;
    double yaw = 0.0;
    
    // Time tracking for Yaw integration
    struct timespec ts_now, ts_prev;
    clock_gettime(CLOCK_MONOTONIC, &ts_prev);

    printf("Starting Loop (100 Hz Target)...\n");

    while (1) {
        // 1. Read All Sensors
        mpu_read_all(&ax, &ay, &az, &gx, &gy, &gz);

        // 2. Calculate Pitch & Roll (Accelerometer)
        // Convert to G-force (not strictly needed for atan2 but good for displaying)
        double ax_g = ax / 16384.0;
        double ay_g = ay / 16384.0;
        double az_g = az / 16384.0;

        pitch = -(atan2(ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180.0) / M_PI;
        roll  = (atan2(ay_g, az_g) * 180.0) / M_PI;

        // 3. Calculate Yaw (Gyroscope Integration)
        // Get Delta Time (dt)
        clock_gettime(CLOCK_MONOTONIC, &ts_now);
        double dt = (ts_now.tv_sec - ts_prev.tv_sec) + 
                    (ts_now.tv_nsec - ts_prev.tv_nsec) / 1000000000.0;
        ts_prev = ts_now;

        // Gyro Z Raw -> Degrees/sec (Scale factor for +/- 250dps is 131.0)
        double gz_dps = gz / 131.0;
        
        // Simple Integration: Angle = Angle + (Rate * Time)
        // Basic drift reduction: Ignore very small values (deadzone)
        if (fabs(gz_dps) > 1.0) { 
            yaw += gz_dps * dt;
        }

        // 4. Print All 6 Values
        // \033[2K clears the line, \r returns cursor to start (clean overwriting)
        printf("\033[2K\rA[X,Y,Z]: %.2f, %.2f, %.2f | R/P/Y: %.2f, %.2f, %.2f", 
               ax_g, ay_g, az_g, roll, pitch, yaw);
        fflush(stdout);

        // Sampling Rate: 100 Hz (10ms delay)
        usleep(10000); 
    }

    return 0;
}
