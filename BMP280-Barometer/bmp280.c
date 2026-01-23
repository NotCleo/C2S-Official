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
// Corrected Base Address per your latest instruction
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
// 2. BMP280 REGISTERS
// ==========================================
#define BMP280_ADDR     0x76  // Default for HW-611 (SDO=GND). Try 0x77 if 0x76 fails.

#define REG_CALIB_START 0x88
#define REG_CHIP_ID     0xD0
#define REG_CTRL_MEAS   0xF4
#define REG_CONFIG      0xF5
#define REG_DATA_START  0xF7

// Global Calibration Struct
struct bmp280_calib {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} cal;

// Global "t_fine" required for pressure compensation
int32_t t_fine; 

// ==========================================
// 3. I2C DRIVER (From MPU6050 Reference)
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
// 4. BMP280 COMPENSATION LOGIC
// ==========================================
// Source: Bosch BMP280 Datasheet

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
    
    if (var1 == 0) return 0; // Avoid divide by zero
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)cal.dig_P7) << 4);
    return (uint32_t)p;
}

// ==========================================
// 5. MAIN
// ==========================================

void bmp_setup() {
    uint8_t chip_id;
    i2c_read_bytes(BMP280_ADDR, REG_CHIP_ID, &chip_id, 1);
    printf("BMP280 Chip ID: 0x%02X (Expected 0x58)\n", chip_id);

    // Read Calibration Data (24 bytes starting at 0x88)
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

    // Configure: Normal Mode, Oversampling x16, Standby 500ms
    // Config Reg (0xF5): t_sb=100 (500ms), filter=100 (x16) -> 0xA0
    i2c_write_reg(BMP280_ADDR, REG_CONFIG, 0xA0);
    // Ctrl Reg (0xF4): osrs_t=101 (x16), osrs_p=101 (x16), mode=11 (Normal) -> 0xB7
    i2c_write_reg(BMP280_ADDR, REG_CTRL_MEAS, 0xB7);
    usleep(100000);
}

int main() {
    printf("[INIT] Mapping I2C at 0x10116000...\n");
    i2c_base_virt = (volatile uint32_t *)map_physical_memory(I2C_PHYS_BASE);

    i2c_init();
    bmp_setup();

    printf("Starting Measurement Loop...\n");

    uint8_t data[6];
    int32_t adc_P, adc_T;
    double temp_final, press_final, altitude;

    while (1) {
        // Read 6 bytes of data (Press_MSB ... Temp_XLSB)
        i2c_read_bytes(BMP280_ADDR, REG_DATA_START, data, 6);

        // Construct 20-bit raw ADC values
        adc_P = (int32_t)((((uint32_t)data[0]) << 12) | (((uint32_t)data[1]) << 4) | (data[2] >> 4));
        adc_T = (int32_t)((((uint32_t)data[3]) << 12) | (((uint32_t)data[4]) << 4) | (data[5] >> 4));

        // Compensate
        int32_t t_int = compensate_T(adc_T);
        uint32_t p_int = compensate_P(adc_P);

        // Scale Results
        temp_final = t_int / 100.0;
        press_final = p_int / 256.0; // Pascals

        // Calculate Altitude (Standard Sea Level = 1013.25 hPa)
        altitude = 44330.0 * (1.0 - pow(press_final / 101325.0, 0.1903));

        // Print
        printf("\033[2K\rTemp: %.2f C \t Pressure: %.2f Pa \t Alt: %.2f m", 
               temp_final, press_final, altitude);
        fflush(stdout);

        usleep(500000); // 0.5s delay
    }

    return 0;
}
