#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <math.h>
#include <sys/time.h>
#include <signal.h>

#define I2C_PHYS_BASE   0x10116000UL
#define MAP_SIZE        4096UL
#define MAP_MASK        (MAP_SIZE - 1)

volatile uint32_t *i2c_base_virt = NULL;

#define I2C_REG(offset)  (*(volatile uint32_t *)((uint8_t *)i2c_base_virt + (offset)))

#define REG_CR          0x100
#define REG_SR          0x104
#define REG_TX_FIFO     0x108
#define REG_RX_FIFO     0x10C

#define MPU6050_ADDR    0x68
#define PWR_MGMT_1      0x6B
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_XOUT_H    0x3B

volatile int keep_running = 1;

void handle_sigint(int sig) {
    keep_running = 0;
}

long long get_time_us() {
    struct timeval te; 
    gettimeofday(&te, NULL);
    return te.tv_sec * 1000000LL + te.tv_usec;
}

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double rate;
    double P[2][2];
} Kalman;

void kalman_init(Kalman *k) {
    k->Q_angle = 0.001;
    k->Q_bias = 0.003;
    k->R_measure = 0.03;
    k->angle = 0.0;
    k->bias = 0.0;
    k->P[0][0] = 0.0; k->P[0][1] = 0.0;
    k->P[1][0] = 0.0; k->P[1][1] = 0.0;
}

double kalman_update(Kalman *k, double newAngle, double newRate, double dt) {
    k->rate = newRate - k->bias;
    k->angle += dt * k->rate;

    k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    double S = k->P[0][0] + k->R_measure;
    double K[2];
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;

    double y = newAngle - k->angle;
    k->angle += K[0] * y;
    k->bias += K[1] * y;

    double P00_temp = k->P[0][0];
    double P01_temp = k->P[0][1];

    k->P[0][0] -= K[0] * P00_temp;
    k->P[0][1] -= K[0] * P01_temp;
    k->P[1][0] -= K[1] * P00_temp;
    k->P[1][1] -= K[1] * P01_temp;

    return k->angle;
}

typedef struct {
    double kp, ki, kd;
    double integral;
    double prev_error;
} PID;

void pid_init(PID *pid, double p, double i, double d) {
    pid->kp = p; pid->ki = i; pid->kd = d;
    pid->integral = 0.0; pid->prev_error = 0.0;
}

double pid_compute(PID *pid, double setpoint, double measured, double dt) {
    if(dt <= 0.0) dt = 0.001; 
    double error = setpoint - measured;
    pid->integral += error * dt;
    double derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

void *map_physical_memory(uint32_t phys_addr) {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) { perror("Error opening /dev/mem"); exit(EXIT_FAILURE); }
    void *mapped_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, phys_addr & ~MAP_MASK);
    close(mem_fd);
    return (void *)((uint8_t *)mapped_base + (phys_addr & MAP_MASK));
}

void i2c_init() {
    I2C_REG(0x40) = 0x0A; usleep(1000);
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

        while ((I2C_REG(REG_SR) & 0x40) && (timeout < 1000000)) {
            timeout++;
            for(volatile int k=0; k<50; k++); 
        }

        if (timeout >= 1000000) {
            data[i] = 0; 
        } else {
            data[i] = (uint8_t)(I2C_REG(REG_RX_FIFO) & 0xFF);
        }
    }
}

void mpu_init() {
    i2c_write_reg(MPU6050_ADDR, PWR_MGMT_1, 0x00);
    i2c_write_reg(MPU6050_ADDR, ACCEL_CONFIG, 0x00); 
    i2c_write_reg(MPU6050_ADDR, GYRO_CONFIG, 0x00);  
}
int main() {
    signal(SIGINT, handle_sigint);
    i2c_base_virt = (volatile uint32_t *)map_physical_memory(I2C_PHYS_BASE);
    i2c_init();

    printf("1) mpu init\n");
    mpu_init();

    Kalman kalman_pitch, kalman_roll;
    kalman_init(&kalman_pitch);
    kalman_init(&kalman_roll);

    printf("2) recording for 15 seconds by writing to csv\n");
    FILE *fp = fopen("mpu_raw_data.csv", "w");
    if(fp) fprintf(fp, "time_s,ax,ay,az,gx,gy,gz,raw_pitch,kalman_pitch\n");

    long long start_us = get_time_us();
    long long prev_us = start_us;

    int16_t ax, ay, az, gx, gy, gz;
    uint8_t buf[14];
    int samples = 0;

    while(keep_running) {
        long long now_us = get_time_us();
        double elapsed = (now_us - start_us) / 1000000.0;
        double dt = (now_us - prev_us) / 1000000.0;
        prev_us = now_us;
        
        if (elapsed >= 15.0) break;

        i2c_read_bytes(MPU6050_ADDR, ACCEL_XOUT_H, buf, 14);
        ax = (int16_t)((buf[0] << 8) | buf[1]);
        ay = (int16_t)((buf[2] << 8) | buf[3]);
        az = (int16_t)((buf[4] << 8) | buf[5]);
        gx = (int16_t)((buf[8] << 8) | buf[9]);
        gy = (int16_t)((buf[10] << 8) | buf[11]);
        gz = (int16_t)((buf[12] << 8) | buf[13]);

        double pitch = -(atan2((double)ax, sqrt((double)ay*ay + (double)az*az)) * 180.0) / M_PI;
        double roll  = (atan2((double)ay, (double)az) * 180.0) / M_PI;

        double gyro_pitch_rate = gy / 131.0; 
        double gyro_roll_rate  = gx / 131.0;

        double filtered_pitch = kalman_update(&kalman_pitch, pitch, gyro_pitch_rate, dt);
        double filtered_roll  = kalman_update(&kalman_roll, roll, gyro_roll_rate, dt);

        if(fp) fprintf(fp, "%.4f,%d,%d,%d,%d,%d,%d,%.2f,%.2f\n", elapsed, ax, ay, az, gx, gy, gz, pitch, filtered_pitch);
        samples++;
    }
    if(fp) fclose(fp);

    printf("3) recording done running kalman\n");
    
    PID pid_pitch, pid_roll;
    pid_init(&pid_pitch, 1.50, 0.02, 0.50); 
    pid_init(&pid_roll, 1.50, 0.02, 0.50);

    printf("4) ran kalman filter activating live pid loop\n");
    printf("\n--- PID STABILIZATION TEST (Ctrl+C to Stop) ---\n");

    prev_us = get_time_us();

    while(keep_running) {
        long long now_us = get_time_us();
        double dt = (now_us - prev_us) / 1000000.0;
        prev_us = now_us;

        i2c_read_bytes(MPU6050_ADDR, ACCEL_XOUT_H, buf, 14);
        ax = (int16_t)((buf[0] << 8) | buf[1]);
        ay = (int16_t)((buf[2] << 8) | buf[3]);
        az = (int16_t)((buf[4] << 8) | buf[5]);
        gx = (int16_t)((buf[8] << 8) | buf[9]);
        gy = (int16_t)((buf[10] << 8) | buf[11]);

        double raw_pitch = -(atan2((double)ax, sqrt((double)ay*ay + (double)az*az)) * 180.0) / M_PI;
        double raw_roll  = (atan2((double)ay, (double)az) * 180.0) / M_PI;

        double gyro_pitch_rate = gy / 131.0; 
        double gyro_roll_rate  = gx / 131.0;

        double filtered_pitch = kalman_update(&kalman_pitch, raw_pitch, gyro_pitch_rate, dt);
        double filtered_roll  = kalman_update(&kalman_roll, raw_roll, gyro_roll_rate, dt);

        double pitch_correction = pid_compute(&pid_pitch, 0.0, filtered_pitch, dt);
        double roll_correction  = pid_compute(&pid_roll, 0.0, filtered_roll, dt);

        char pitch_cmd[30];
        if (pitch_correction > 15.0) strcpy(pitch_cmd, "TILT NOSE DOWN");
        else if (pitch_correction < -15.0) strcpy(pitch_cmd, "TILT NOSE UP  ");
        else strcpy(pitch_cmd, "PITCH LEVEL   ");

        char roll_cmd[30];
        if (roll_correction > 15.0) strcpy(roll_cmd, "ROLL LEFT ");
        else if (roll_correction < -15.0) strcpy(roll_cmd, "ROLL RIGHT");
        else strcpy(roll_cmd, "ROLL LEVEL");

        printf("\033[2K\r5) %s | %s | (P: %5.1f, R: %5.1f)", 
               pitch_cmd, roll_cmd, filtered_pitch, filtered_roll);
        fflush(stdout);

        usleep(50000); 
    }
}
