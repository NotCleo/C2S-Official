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

// ==========================================
// 2. SENSOR ADDRESSES & REGISTERS
// ==========================================

// --- BMP280 ---
#define BMP280_ADDR     0x76  
#define REG_CALIB_START 0x88
#define REG_CHIP_ID     0xD0
#define REG_CTRL_MEAS   0xF4
#define REG_CONFIG      0xF5
#define REG_DATA_START  0xF7

// --- MPU6050 ---
#define MPU6050_ADDR    0x68
#define PWR_MGMT_1      0x6B
#define ACCEL_CONFIG    0x1C
#define ACCEL_XOUT_H    0x3B

// ==========================================
// 3. GLOBAL VARIABLES
// ==========================================

// BMP280 Calibration Data
struct bmp280_calib {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
    int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
} cal;

int32_t t_fine; 

// ==========================================
// 4. I2C DRIVER
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
    I2C_REG(0x40) = 0x0A; // Soft Reset
    usleep(1000);
    I2C_REG(REG_CR) = 0x01; // Enable
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
    I2C_REG(REG_TX_FIFO) = 0x100 | (dev_addr << 1) | 1; // Restart + Read
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
// 5. BMP280 FUNCTIONS
// ==========================================

int32_t compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)cal.dig_T1 << 1))) * ((int32_t)cal.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)cal.dig_T1))) >> 12) * ((int32_t)cal.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)cal.dig_P6;
    var2 = var2 + ((var1 * (int64_t)cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)cal.dig_P3) >> 8) + ((var1 * (int64_t)cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)cal.dig_P1) >> 33;
    
    if (var1 == 0) return 0;
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)cal.dig_P7) << 4);
    return (uint32_t)p;
}

void bmp_setup() {
    uint8_t chip_id;
    i2c_read_bytes(BMP280_ADDR, REG_CHIP_ID, &chip_id, 1);
    printf("BMP280 Chip ID: 0x%02X\n", chip_id);

    uint8_t buf[24];
    i2c_read_bytes(BMP280_ADDR, REG_CALIB_START, buf, 24);

    cal.dig_T1 = (buf[1] << 8) | buf[0];
    cal.dig_T2 = (buf[3] << 8) | buf[2];
    cal.dig_T3 = (buf[5] << 8) | buf[4];
    cal.dig_P1 = (buf[7] << 8) | buf[6];
    cal.dig_P2 = (buf[9] << 8) | buf[8];
    cal.dig_P3 = (buf[11] << 8) | buf[10];
    cal.dig_P4 = (buf[13] << 8) | buf[12];
    cal.dig_P5 = (buf[15] << 8) | buf[14];
    cal.dig_P6 = (buf[17] << 8) | buf[16];
    cal.dig_P7 = (buf[19] << 8) | buf[18];
    cal.dig_P8 = (buf[21] << 8) | buf[20];
    cal.dig_P9 = (buf[23] << 8) | buf[22];

    i2c_write_reg(BMP280_ADDR, REG_CONFIG, 0xA0);
    i2c_write_reg(BMP280_ADDR, REG_CTRL_MEAS, 0xB7);
    usleep(100000);
}

// ==========================================
// 6. MPU6050 FUNCTIONS
// ==========================================

void mpu_setup() {
    printf("Initializing MPU6050...\n");
    i2c_write_reg(MPU6050_ADDR, PWR_MGMT_1, 0x00);
    usleep(10000);
    i2c_write_reg(MPU6050_ADDR, ACCEL_CONFIG, 0x00);
    usleep(10000);
}

// ==========================================
// 7. MAIN
// ==========================================

int main() {
    printf("[INIT] Mapping I2C at 0x10116000...\n");
    i2c_base_virt = (volatile uint32_t *)map_physical_memory(I2C_PHYS_BASE);
    i2c_init();

    // Setup both sensors
    bmp_setup();
    mpu_setup();

    printf("Sensors Ready. Starting Loop...\n");
    printf("==============================================================\n");

    // Variables for BMP280
    uint8_t bmp_data[6];
    int32_t adc_P, adc_T;
    double temp_final, press_final, altitude;

    // Variables for MPU6050
    uint8_t mpu_data[6];
    int16_t ax, ay, az;
    double pitch, roll;

    while (1) {
        // --- READ BMP280 ---
        i2c_read_bytes(BMP280_ADDR, REG_DATA_START, bmp_data, 6);
        adc_P = (int32_t)((((uint32_t)bmp_data[0]) << 12) | (((uint32_t)bmp_data[1]) << 4) | (bmp_data[2] >> 4));
        adc_T = (int32_t)((((uint32_t)bmp_data[3]) << 12) | (((uint32_t)bmp_data[4]) << 4) | (bmp_data[5] >> 4));

        int32_t t_int = compensate_T(adc_T);
        uint32_t p_int = compensate_P(adc_P);

        temp_final = t_int / 100.0;
        press_final = p_int / 256.0;
        altitude = 44330.0 * (1.0 - pow(press_final / 101325.0, 0.1903));

        // --- READ MPU6050 ---
        i2c_read_bytes(MPU6050_ADDR, ACCEL_XOUT_H, mpu_data, 6);
        ax = (int16_t)((mpu_data[0] << 8) | mpu_data[1]);
        ay = (int16_t)((mpu_data[2] << 8) | mpu_data[3]);
        az = (int16_t)((mpu_data[4] << 8) | mpu_data[5]);

        double x_buff = (double)ax;
        double y_buff = (double)ay;
        double z_buff = (double)az;

        pitch = -(atan2(x_buff, sqrt(y_buff*y_buff + z_buff*z_buff)) * 180.0) / M_PI;
        roll  = (atan2(y_buff, z_buff) * 180.0) / M_PI;

        // --- PRINT COMBINED DATA ---
        // Using \033[2K\r to overwrite the line cleanly
        printf("\033[2K\r[BMP] T: %.2f C | Alt: %.2f m   [MPU] Pitch: %6.2f | Roll: %6.2f", 
               temp_final, altitude, pitch, roll);
        fflush(stdout);

        usleep(100000); // 10Hz update rate
    }

    return 0;
}
