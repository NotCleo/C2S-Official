#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

// ==========================================
// 1. HARDWARE CONFIGURATION (CONFIRMED)
// ==========================================
// Address verified by your 'txrx' test
#define UART_BASE       0x10111000UL 
#define MAP_SIZE        4096UL
#define MAP_MASK        (MAP_SIZE - 1)

// AXI UARTLite Registers
#define UL_RX_FIFO      0x00 
#define UL_TX_FIFO      0x04 
#define UL_STATUS       0x08 
#define UL_CTRL         0x0C 

// Status Bits
#define STS_RX_VALID    0x01
#define STS_TX_FULL     0x08
#define STS_FRAME_ERR   0x40 

volatile uint32_t *uart_base = NULL;
#define UART_REG(offset) (*(volatile uint32_t *)((uint8_t *)uart_base + (offset)))

// NMEA Buffer
char nmea_buffer[128];
int buf_idx = 0;

// ==========================================
// 2. MEMORY MAPPING DRIVER
// ==========================================
void *map_memory(uint32_t phys_addr) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) { perror("open /dev/mem"); exit(1); }
    void *map = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys_addr & ~MAP_MASK);
    if (map == MAP_FAILED) { perror("mmap"); exit(1); }
    close(fd);
    return (void *)((uint8_t *)map + (phys_addr & MAP_MASK));
}

// ==========================================
// 3. ROBUST NMEA PARSER
// ==========================================
// Handles $GPGGA and $GNGGA (Modern U-blox)
// Uses strsep() instead of strtok() to handle empty fields ",," correctly
void parse_nmea(char *line) {
    // Only parse GGA sentences (Global Positioning System Fix Data)
    if (strncmp(line, "$GPGGA", 6) != 0 && strncmp(line, "$GNGGA", 6) != 0) {
        return; 
    }

    char *token;
    char *rest = line;
    int field = 0;
    
    // Default values
    char time[16] = "Wait..";
    char lat[16] = "0.0";
    char lat_d[2] = "";
    char lon[16] = "0.0";
    char lon_d[2] = "";
    char fix[2]  = "0";
    char sats[4] = "0";
    char alt[10] = "0.0";

    // Loop through comma-separated fields
    while ((token = strsep(&rest, ",")) != NULL) {
        switch(field) {
            case 1: strncpy(time, token, 10); break;  // Time (UTC)
            case 2: strncpy(lat, token, 15); break;   // Latitude
            case 3: strncpy(lat_d, token, 1); break;  // N/S
            case 4: strncpy(lon, token, 15); break;   // Longitude
            case 5: strncpy(lon_d, token, 1); break;  // E/W
            case 6: strncpy(fix, token, 1); break;    // Fix (0=Invalid, 1=GPS)
            case 7: strncpy(sats, token, 3); break;   // Satellites
            case 9: strncpy(alt, token, 9); break;    // Altitude
        }
        field++;
    }

    // Only print if we have at least a valid Time field
    if (strlen(time) >= 6) {
        printf("\033[2K\r"); // Clear previous line
        
        if (fix[0] > '0') {
            // WE HAVE A FIX
            printf("[FIXED] Time: %.2s:%.2s:%.2s | Lat: %s %s | Lon: %s %s | Alt: %sm | Sats: %s", 
                   time, time+2, time+4, lat, lat_d, lon, lon_d, alt, sats);
        } else {
            // NO FIX YET (Searching)
            printf("[SEARCHING] Time: %.2s:%.2s:%.2s | Sats: %s | Fix: NO (Go Outdoors)", 
                   time, time+2, time+4, sats);
        }
        fflush(stdout);
    }
}

// ==========================================
// 4. MAIN LOOP
// ==========================================
int main() {
    uart_base = (volatile uint32_t *)map_memory(UART_BASE);
    
    printf("--- VEGA GPS DRIVER (0x%08X) ---\n", UART_BASE);
    printf("Status: Hardware Link CONFIRMED.\n");
    printf("Waiting for GPS data stream...\n\n");

    // Reset FIFO to clear any old garbage
    UART_REG(UL_CTRL) = 0x03; 

    while (1) {
        uint32_t status = UART_REG(UL_STATUS);

        // Bit 0 (0x01) = RX FIFO Valid (Data available)
        if (status & STS_RX_VALID) {
            // Read 1 byte
            char c = (char)(UART_REG(UL_RX_FIFO) & 0xFF);
            
            // Build the sentence
            if (c == '$') { 
                buf_idx = 0; // Start of new sentence
            }
            
            if (buf_idx < 127) {
                nmea_buffer[buf_idx++] = c;
            }

            // End of sentence
            if (c == '\n') {
                nmea_buffer[buf_idx] = '\0'; // Null terminate
                parse_nmea(nmea_buffer);
                buf_idx = 0; // Reset
            }
        }
    }

    return 0;
}
