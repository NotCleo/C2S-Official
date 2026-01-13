#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

#define SPI_PHYS_BASE   0x10115000UL 
#define I2C_PHYS_BASE   0x10116000UL
#define MAP_SIZE        4096UL
#define MAP_MASK        (MAP_SIZE - 1)

volatile uint32_t *spi_base_virt = NULL;
volatile uint32_t *i2c_base_virt = NULL;

#define SPI_REG(offset)  (*(volatile uint32_t *)((uint8_t *)spi_base_virt + (offset)))
#define I2C_REG(offset)  (*(volatile uint32_t *)((uint8_t *)i2c_base_virt + (offset)))

#define ARDUCHIP_TEST1       0x00
#define ARDUCHIP_FIFO        0x04
#define ARDUCHIP_TRIG        0x41
#define CAP_DONE_MASK        0x08
#define ARDUCHIP_FIFO_SIZE1  0x42
#define ARDUCHIP_FIFO_SIZE2  0x43
#define ARDUCHIP_FIFO_SIZE3  0x44
#define BURST_FIFO_READ      0x3C
#define OV2640_ADDR          0x30

void *map_physical_memory(uint32_t phys_addr) {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) { perror("Error opening /dev/mem"); exit(EXIT_FAILURE); }
    void *mapped_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, phys_addr & ~MAP_MASK);
    if (mapped_base == MAP_FAILED) { perror("Error mapping memory"); close(mem_fd); exit(EXIT_FAILURE); }
    close(mem_fd);
    return (void *)((uint8_t *)mapped_base + (phys_addr & MAP_MASK));
}

void spi_init() {
    SPI_REG(0x40) = 0x0A;       
    usleep(1000);
    SPI_REG(0x60) = 0x186;      
    SPI_REG(0x70) = 0xFFFFFFFF; 
}

uint8_t spi_transfer(uint8_t data) {
    SPI_REG(0x68) = data;       
    SPI_REG(0x60) &= ~0x100;    
    
    int timeout = 0;
    while((SPI_REG(0x64) & 0x01) != 0) {
        if(++timeout > 100000) break; 
    }
    return (uint8_t)SPI_REG(0x6C); 
}

void arducam_write_reg(uint8_t addr, uint8_t data) {
    SPI_REG(0x70) = 0xFFFFFFFE; 
    spi_transfer(addr | 0x80);
    spi_transfer(data);
    SPI_REG(0x70) = 0xFFFFFFFF; 
    usleep(100);
}

uint8_t arducam_read_reg(uint8_t addr) {
    SPI_REG(0x70) = 0xFFFFFFFE; 
    spi_transfer(addr & 0x7F);
    uint8_t val = spi_transfer(0x00);
    SPI_REG(0x70) = 0xFFFFFFFF; 
    usleep(100);
    return val;
}

void i2c_init() {
    I2C_REG(0x40) = 0x0A; 
    usleep(1000);
    I2C_REG(0x100) = 0xC8;
}

void i2c_write_reg(uint8_t reg, uint8_t val) {
    I2C_REG(0x40) = 0x0A;
    usleep(200); 
    I2C_REG(0x100) = 0xC8;

    I2C_REG(0x108) = 0x100 | (OV2640_ADDR << 1); 
    I2C_REG(0x108) = reg;                        
    I2C_REG(0x108) = 0x200 | val;                

    usleep(10000); 
}

const uint8_t OV2640_YUV422_QVGA[][2] = {
    {0xff, 0x00}, {0x2c, 0xff}, {0x2e, 0xdf}, {0xff, 0x01}, {0x3c, 0x32}, {0x11, 0x00},
    {0x09, 0x02}, {0x04, 0x28}, {0x13, 0xe5}, {0x14, 0x48}, {0x2c, 0x0c}, {0x33, 0x78},
    {0x3a, 0x33}, {0x3b, 0xfb}, {0x3e, 0x00}, {0x43, 0x11}, {0x16, 0x10}, {0x39, 0x92},
    {0x35, 0xda}, {0x22, 0x1a}, {0x37, 0xc3}, {0x23, 0x00}, {0x34, 0xc0}, {0x36, 0x1a},
    {0x06, 0x88}, {0x07, 0xc0}, {0x0d, 0x87}, {0x0e, 0x41}, {0x4c, 0x00}, {0xff, 0x00},
    {0xe0, 0x04}, {0xc0, 0x64}, {0xc1, 0x4b}, {0x86, 0x35}, {0x50, 0x89}, {0x51, 0xc8},
    {0x52, 0x96}, {0x53, 0x00}, {0x54, 0x00}, {0x55, 0x00}, {0x57, 0x00}, {0x5a, 0x50},
    {0x5b, 0x3c}, {0x5c, 0x00}, {0xe0, 0x00},
    {0xff, 0x00}, {0xda, 0x10}, {0xd7, 0x03}, {0xdf, 0x00}, {0x33, 0x80}, {0x3c, 0x00},
    {0x3d, 0x00}, {0x3e, 0x00}, {0x3f, 0x00}, {0x40, 0x00}, {0xff, 0xff}
};

void sensor_write_array(const uint8_t data[][2]) {
    int i = 0;
    while (1) {
        uint8_t reg = data[i][0];
        uint8_t val = data[i][1];
        if (reg == 0xFF && val == 0xFF) break;
        i2c_write_reg(reg, val);
        i++;
    }
}

int main() {
    printf("[INIT] Raw YUV Capture (Blind I2C)...\n");
    spi_base_virt = (volatile uint32_t *)map_physical_memory(SPI_PHYS_BASE);
    i2c_base_virt = (volatile uint32_t *)map_physical_memory(I2C_PHYS_BASE);

    spi_init();
    i2c_init();

    printf("[1/5] SPI Check... ");
    arducam_write_reg(ARDUCHIP_TEST1, 0x55);
    uint8_t test = arducam_read_reg(ARDUCHIP_TEST1);
    if(test != 0x55) { printf("FAIL (0x%02X)\n", test); return -1; }
    printf("PASS\n");

    printf("[2/5] Configuring YUV422... ");
    fflush(stdout);
    
    i2c_write_reg(0xFF, 0x01); 
    i2c_write_reg(0x12, 0x80); 
    usleep(200000); 

    sensor_write_array(OV2640_YUV422_QVGA);
    printf("DONE\n");

    printf("[3/5] Capturing... ");
    fflush(stdout);
    arducam_write_reg(ARDUCHIP_FIFO, 0x01);
    arducam_write_reg(ARDUCHIP_FIFO, 0x02);
    
    int attempts = 0;
    while (!(arducam_read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK)) {
        usleep(100000);
        if(++attempts > 500) { printf("TIMEOUT\n"); return -1; }
    }
    printf("DONE\n");

    uint32_t len1 = arducam_read_reg(ARDUCHIP_FIFO_SIZE1);
    uint32_t len2 = arducam_read_reg(ARDUCHIP_FIFO_SIZE2);
    uint32_t len3 = arducam_read_reg(ARDUCHIP_FIFO_SIZE3) & 0x7F;
    uint32_t length = ((len3 << 16) | (len2 << 8) | len1) & 0x07FFFFF;
    printf("[4/5] Bytes: %d (Expected ~153600)\n", (int)length);

    printf("[5/5] Saving 'image.bin'...");
    fflush(stdout);
    
    FILE *fp = fopen("image.bin", "wb");
    if (!fp) return -1;

    SPI_REG(0x70) = 0xFFFFFFFE; 
    spi_transfer(BURST_FIFO_READ);
    spi_transfer(0x00); 

    for (uint32_t i = 0; i < length; i++) {
        fputc(spi_transfer(0x00), fp);
    }
    
    SPI_REG(0x70) = 0xFFFFFFFF; 
    fclose(fp);

    printf(" DONE\n");
    return 0;
}
