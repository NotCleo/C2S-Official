#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

// ==========================================
// 1. HARDWARE MAPPING
// ==========================================
#define UART_BASE       0x10111000UL 
#define SPI_PHYS_BASE   0x10115000UL 
#define I2C_PHYS_BASE   0x10116000UL
#define MAP_SIZE        4096UL
#define MAP_MASK        (MAP_SIZE - 1)

// Pointers
volatile uint32_t *uart_base = NULL;
volatile uint32_t *spi_base_virt = NULL;
volatile uint32_t *i2c_base_virt = NULL;

// Access Macros
#define UART_REG(off)   (*(volatile uint32_t *)((uint8_t *)uart_base + (off)))
#define SPI_REG(off)    (*(volatile uint32_t *)((uint8_t *)spi_base_virt + (off)))
#define I2C_REG(off)    (*(volatile uint32_t *)((uint8_t *)i2c_base_virt + (off)))

// UART Registers
#define UL_RX_FIFO      0x00
#define UL_STATUS       0x08
#define UL_CTRL         0x0C
#define STS_RX_VALID    0x01

// Camera Registers
#define ARDUCHIP_TEST1       0x00
#define ARDUCHIP_FIFO        0x04
#define ARDUCHIP_TRIG        0x41
#define CAP_DONE_MASK        0x08
#define ARDUCHIP_FIFO_SIZE1  0x42
#define ARDUCHIP_FIFO_SIZE2  0x43
#define ARDUCHIP_FIFO_SIZE3  0x44
#define BURST_FIFO_READ      0x3C
#define OV2640_ADDR          0x30

// Global GPS Data Storage
char gps_meta_string[256] = "GPS_DATA_NOT_FOUND";

// ==========================================
// 2. MEMORY DRIVER
// ==========================================
void *map_physical_memory(uint32_t phys_addr) {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) { perror("Error opening /dev/mem"); exit(EXIT_FAILURE); }
    void *mapped_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, phys_addr & ~MAP_MASK);
    if (mapped_base == MAP_FAILED) { perror("Error mapping memory"); close(mem_fd); exit(EXIT_FAILURE); }
    close(mem_fd);
    return (void *)((uint8_t *)mapped_base + (phys_addr & MAP_MASK));
}

// ==========================================
// 3. LOW-LEVEL DRIVERS
// ==========================================
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
    I2C_REG(0x100) = 0x01; 
}

void i2c_write_reg(uint8_t reg, uint8_t val) {
    I2C_REG(0x40) = 0x0A; usleep(200); 
    I2C_REG(0x100) = 0x01;
    I2C_REG(0x108) = 0x100 | (OV2640_ADDR << 1); 
    I2C_REG(0x108) = reg;                          
    I2C_REG(0x108) = 0x200 | val;                  
    usleep(5000); 
}

// ==========================================
// 4. OV2640 YUV422 CONFIGURATION
// ==========================================
// Replaced JPEG config with YUV422 to match Python script requirements
const uint8_t OV2640_YUV422[][2] = {
    {0xff, 0x00}, {0x2c, 0xff}, {0x2e, 0xdf}, {0xff, 0x01}, {0x3c, 0x32},
    {0x11, 0x00}, {0x09, 0x02}, {0x04, 0x28}, {0x13, 0xe5}, {0x14, 0x48},
    {0x2c, 0x0c}, {0x33, 0x78}, {0x3a, 0x33}, {0x3b, 0xfb}, {0x3e, 0x00},
    {0x43, 0x11}, {0x16, 0x10}, {0x39, 0x02}, {0x35, 0x88}, {0x22, 0x0a},
    {0x37, 0x40}, {0x23, 0x00}, {0x34, 0xa0}, {0x06, 0x02}, {0x07, 0xc0},
    {0x0d, 0xb7}, {0x0e, 0x01}, {0x4c, 0x00}, {0x4a, 0x81}, {0x21, 0x99},
    {0x24, 0x40}, {0x25, 0x38}, {0x26, 0x82}, {0x5c, 0x00}, {0x63, 0x00},
    {0x46, 0x22}, {0x0c, 0x3c}, {0x5d, 0x55}, {0x5e, 0x7d}, {0x5f, 0x7d},
    {0x60, 0x55}, {0x61, 0x70}, {0x62, 0x80}, {0x7c, 0x05}, {0x20, 0x80},
    {0x28, 0x30}, {0x6c, 0x00}, {0x6d, 0x80}, {0x6e, 0x00}, {0x70, 0x02},
    {0x71, 0x94}, {0x73, 0xc1}, {0x3d, 0x34}, {0x5a, 0x57}, {0xaa, 0x94}, 
    {0xc3, 0xed}, {0xff, 0xff}
};

const uint8_t OV2640_320x240[][2] = {
    {0xff, 0x01}, {0x12, 0x40}, {0x17, 0x11}, {0x18, 0x43}, {0x19, 0x00},
    {0x1a, 0x4b}, {0x32, 0x09}, {0x4f, 0x80}, {0x50, 0x80}, {0x5a, 0x23},
    {0x6d, 0x00}, {0x39, 0x12}, {0x35, 0xda}, {0x22, 0x1a}, {0x37, 0xc3},
    {0x23, 0x00}, {0x34, 0xc0}, {0x36, 0x1a}, {0x06, 0x88}, {0x07, 0xc0},
    {0x0d, 0x87}, {0x0e, 0x41}, {0x4c, 0x00}, {0xff, 0xff}
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

// ==========================================
// 5. GPS PARSER & LOGIC
// ==========================================
// Returns 1 if a fix is locked and data is saved, 0 otherwise
int parse_nmea(char *line) {
    if (strncmp(line, "$GPGGA", 6) != 0 && strncmp(line, "$GNGGA", 6) != 0) return 0;

    char time[16]="", lat[16]="", lat_d[2]="", lon[16]="", lon_d[2]="", fix[2]="0", sats[4]="", alt[10]="";
    char *token, *rest = line;
    int field = 0;

    while ((token = strsep(&rest, ",")) != NULL) {
        switch(field) {
            case 1: strncpy(time, token, 10); break;
            case 2: strncpy(lat, token, 15); break;
            case 3: strncpy(lat_d, token, 1); break;
            case 4: strncpy(lon, token, 15); break;
            case 5: strncpy(lon_d, token, 1); break;
            case 6: strncpy(fix, token, 1); break;
            case 7: strncpy(sats, token, 3); break;
            case 9: strncpy(alt, token, 9); break;
        }
        field++;
    }

    // Print Status
    printf("\r[GPS] Time: %.6s | Sats: %s | Fix: %s ", time, sats, (fix[0] > '0') ? "YES" : "NO ");
    fflush(stdout);

    if (fix[0] > '0' && strlen(lat) > 4) {
        // Save the coordinates to global string
        snprintf(gps_meta_string, 256, "FIXED,Time:%s,Lat:%s%s,Lon:%s%s,Alt:%sm,Sats:%s", 
                 time, lat, lat_d, lon, lon_d, alt, sats);
        printf("\n[GPS LOCKED] Coordinates found: %s\n", gps_meta_string);
        return 1; 
    }
    return 0;
}

void wait_for_gps_fix() {
    printf("[GPS] Initializing... Waiting for valid coordinates...\n");
    printf("[GPS] (Ensure antenna is outdoors)\n");
    
    // Reset FIFO
    UART_REG(UL_CTRL) = 0x03; 

    char nmea_buffer[128];
    int buf_idx = 0;
    
    while (1) {
        if (UART_REG(UL_STATUS) & STS_RX_VALID) {
            char c = (char)(UART_REG(UL_RX_FIFO) & 0xFF);
            if (c == '$') buf_idx = 0;
            if (buf_idx < 127) nmea_buffer[buf_idx++] = c;
            if (c == '\n') {
                nmea_buffer[buf_idx] = '\0';
                if (parse_nmea(nmea_buffer)) return; // Break loop on fix
                buf_idx = 0;
            }
        }
    }
}

// ==========================================
// 6. MAIN EXECUTION
// ==========================================
int main() {
    // --- 1. Map All Hardware ---
    printf("--- GEO-CAM SYSTEM START ---\n");
    uart_base     = (volatile uint32_t *)map_physical_memory(UART_BASE);
    spi_base_virt = (volatile uint32_t *)map_physical_memory(SPI_PHYS_BASE);
    i2c_base_virt = (volatile uint32_t *)map_physical_memory(I2C_PHYS_BASE);

    // --- 2. GET GPS COORDINATES ---
    wait_for_gps_fix();

    // --- 3. INIT CAMERA ---
    printf("\n[CAM] Initializing OV2640...\n");
    spi_init();
    i2c_init();

    arducam_write_reg(ARDUCHIP_TEST1, 0x55);
    if(arducam_read_reg(ARDUCHIP_TEST1) != 0x55) {
        printf("[ERROR] SPI Bus Fail.\n"); return -1;
    }

    i2c_write_reg(0xFF, 0x01); 
    i2c_write_reg(0x12, 0x80); 
    usleep(200000); 
    
    // Load YUV422 Config (Required for your Python script)
    sensor_write_array(OV2640_YUV422); 
    sensor_write_array(OV2640_320x240);
    printf("[CAM] Sensor Configured (YUV422).\n");

    // --- 4. CAPTURE IMAGE ---
    printf("[CAM] Capturing...\n");
    arducam_write_reg(ARDUCHIP_FIFO, 0x01); 
    arducam_write_reg(ARDUCHIP_FIFO, 0x02); 

    while (!(arducam_read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK)) {
        usleep(10000);
    }
    printf("[CAM] Capture Done.\n");

    // --- 5. SAVE IMAGE + GPS METADATA ---
    uint32_t len1 = arducam_read_reg(ARDUCHIP_FIFO_SIZE1);
    uint32_t len2 = arducam_read_reg(ARDUCHIP_FIFO_SIZE2);
    uint32_t len3 = arducam_read_reg(ARDUCHIP_FIFO_SIZE3) & 0x7F;
    uint32_t length = ((len3 << 16) | (len2 << 8) | len1) & 0x07FFFFF;

    // YUV422 320x240 should be exactly 153600 bytes
    printf("[FILE] Image Data Size: %d bytes\n", (int)length);

    FILE *fp = fopen("image.bin", "wb");
    if (!fp) { perror("File error"); return -1; }

    SPI_REG(0x70) = 0xFFFFFFFE; 
    spi_transfer(BURST_FIFO_READ);
    spi_transfer(0x00); 

    for (uint32_t i = 0; i < length; i++) {
        fputc(spi_transfer(0x00), fp);
    }
    SPI_REG(0x70) = 0xFFFFFFFF;

    // --- APPEND METADATA ---
    // We append the string at the exact end of the binary stream
    fprintf(fp, "%s", gps_meta_string);
    printf("[FILE] Appended GPS Metadata: %s\n", gps_meta_string);

    fclose(fp);
    printf("[SUCCESS] 'image.bin' created with Geotag.\n");

    return 0;
}
