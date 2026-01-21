#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <math.h>

// ==========================================
// 1. HARDWARE MAPPING
// ==========================================
#define I2C_PHYS_BASE   0x10116000UL
#define MAP_SIZE        4096UL
#define MAP_MASK        (MAP_SIZE - 1)

volatile uint32_t *i2c_base_virt = NULL;

#define I2C_REG(offset)  (*(volatile uint32_t *)((uint8_t *)i2c_base_virt + (offset)))

// AXI IIC Register Offsets
#define REG_CR          0x100 // Control Register
#define REG_SR          0x104 // Status Register
#define REG_TX_FIFO     0x108 // Transmit FIFO
#define REG_RX_FIFO     0x10C // Receive FIFO

// MPU6050 Registers
#define MPU6050_ADDR    0x68
#define PWR_MGMT_1      0x6B
#define ACCEL_CONFIG    0x1C
#define ACCEL_XOUT_H    0x3B

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
    // Soft Reset (0xA) to 0x40 (SOFTR) is common, but basic enable is at 0x100
    I2C_REG(0x40) = 0x0A; 
    usleep(1000);
    // Enable Core
    I2C_REG(REG_CR) = 0x01; 
}

// Write a single byte to a register
void i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t val) {
    // 1. Start + Address(Write)
    I2C_REG(REG_TX_FIFO) = 0x100 | (dev_addr << 1); 
    // 2. Register Address
    I2C_REG(REG_TX_FIFO) = reg_addr;                
    // 3. Data + Stop
    I2C_REG(REG_TX_FIFO) = 0x200 | val;             
    
    usleep(5000); // Blind wait for completion
}

// Read multiple bytes from a register (Burst Read)
void i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, int length) {
    // 1. Start + Address(Write)
    I2C_REG(REG_TX_FIFO) = 0x100 | (dev_addr << 1);
    // 2. Register Address
    I2C_REG(REG_TX_FIFO) = reg_addr;
    // 3. Restart + Address(Read)
    I2C_REG(REG_TX_FIFO) = 0x100 | (dev_addr << 1) | 1;
    // 4. Stop + Byte Count (This tells controller to read 'length' bytes)
    I2C_REG(REG_TX_FIFO) = 0x200 | length;

    // Read loop
    for(int i=0; i<length; i++) {
        // Wait until RX FIFO is not empty (SR bit 6 = 0 means not empty, 1 means empty)
        // Safety timeout included
        int timeout = 0;
        while ((I2C_REG(REG_SR) & 0x40) && (timeout < 100000)) {
            timeout++;
            usleep(10);
        }
        
        if (timeout >= 100000) {
            printf("[Error] I2C Read Timeout\n");
            data[i] = 0;
        } else {
            data[i] = (uint8_t)(I2C_REG(REG_RX_FIFO) & 0xFF);
        }
    }
}

// ==========================================
// 3. MPU6050 FUNCTIONS
// ==========================================

void mpu_init() {
    printf("Initializing MPU6050...\n");
    // Wake up MPU6050 (Write 0 to PWR_MGMT_1)
    i2c_write_reg(MPU6050_ADDR, PWR_MGMT_1, 0x00);
    usleep(10000);
    
    // Config Accelerometer to +/- 2g (Write 0 to ACCEL_CONFIG)
    i2c_write_reg(MPU6050_ADDR, ACCEL_CONFIG, 0x00);
    usleep(10000);
}

void mpu_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buffer[6];
    // Burst read 6 bytes starting from ACCEL_XOUT_H (0x3B)
    i2c_read_bytes(MPU6050_ADDR, ACCEL_XOUT_H, buffer, 6);

    // Combine high and low bytes
    *ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    *ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    *az = (int16_t)((buffer[4] << 8) | buffer[5]);
}

// ==========================================
// 4. MAIN LOOP
// ==========================================

int main() {
    printf("[INIT] Mapping Physical Memory...\n");
    i2c_base_virt = (volatile uint32_t *)map_physical_memory(I2C_PHYS_BASE);

    i2c_init();
    mpu_init();

    printf("Starting Measurement Loop (Ctrl+C to stop)...\n");

    int16_t raw_ax, raw_ay, raw_az;
    double pitch, roll;

    while (1) {
        // 1. Read Raw Data
        mpu_read_accel(&raw_ax, &raw_ay, &raw_az);

        // 2. Normalize (Optional for atan2, but good practice)
        // Range is 2g, scale factor is 16384.0
        // Note: atan2 works with raw ratios, so strict G conversion isn't mandatory for angle.
        
        // 3. Calculate Pitch & Roll (Arduino Library Algo)
        // Pitch = atan2(X, sqrt(Y*Y + Z*Z))
        // Roll  = atan2(Y, Z)
        
        // Note: Arduino uses float inputs. Casting raw ints works.
        double x_buff = (double)raw_ax;
        double y_buff = (double)raw_ay;
        double z_buff = (double)raw_az;

        pitch = -(atan2(x_buff, sqrt(y_buff*y_buff + z_buff*z_buff)) * 180.0) / M_PI;
        roll  = (atan2(y_buff, z_buff) * 180.0) / M_PI;

        // 4. Output
        printf(" Pitch = %6.2f \t Roll = %6.2f \r", pitch, roll);
        fflush(stdout);

        usleep(100000); // 100ms delay
    }

    return 0;
}
