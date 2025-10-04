/* ---------------------------
   Includes & type shortcuts
   --------------------------- */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* ---------------------------
   HAL: Platform-specific functions
   ---------------------------
   You MUST implement these for your softcore/FPGA:
   - I2C/SPI access for MPU-9250
   - PWM outputs for ESCs
   - ADC reads for battery/current
   - Precise microsecond timer and delay
   - Pulse-capture for RC receiver channels (or implement PPM/PPM->channels)
*/
bool hal_i2c_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len);
/* read 'len' bytes into data from device register reg_addr */
bool hal_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

uint32_t hal_get_micros(void);        /* high-res microseconds */
void hal_delay_ms(uint32_t ms);      /* millisecond delay */

/* ADC reading (returns raw ADC units or scaled voltage depending on your ADC) */
int32_t hal_adc_read(uint8_t channel);

/* PWM: channel in 1..4, value is pulse width in microseconds (1000..2000 typical) */
void hal_pwm_write(uint8_t channel, uint16_t pulse_width_us);

/* Pulse capture for RC receiver:
   - pulsecapture_init(pin) : set up capture on input pin
   - pulsecapture_read_channel(ch_index) : returns last measured pulse width in microseconds,
     0 if not present. You will need to map receiver wiring to channels.
*/
void pulsecapture_init(int rx_pin);
uint16_t pulsecapture_read_channel(int ch_index);

/* Optionally implement an atomic critical section if needed */
void hal_enter_critical(void);
void hal_exit_critical(void);

/* ---------------------------
   MPU-9250 register defines (common ones)
   --------------------------- */
#define MPU9250_I2C_ADDR_LO  0x68  /* AD0 = 0 */
#define MPU9250_I2C_ADDR_HI  0x69  /* AD0 = 1 */

#define MPU9250_REG_PWR_MGMT_1  0x6B
#define MPU9250_REG_SMPLRT_DIV  0x19
#define MPU9250_REG_CONFIG      0x1A
#define MPU9250_REG_GYRO_CONFIG 0x1B
#define MPU9250_REG_ACCEL_CONFIG 0x1C
#define MPU9250_REG_INT_ENABLE  0x38

/* sensor data start registers */
#define MPU9250_REG_ACCEL_XOUT_H 0x3B
#define MPU9250_REG_GYRO_XOUT_H  0x43
#define MPU9250_REG_WHO_AM_I     0x75

/* Useful constants */
#define GYRO_SENS_FS_SEL_500DPS  65.5f   /* LSB per deg/s for FS_SEL=1 (±500 dps). See datasheet. */
#define DEFAULT_I2C_ADDR         MPU9250_I2C_ADDR_LO

/* ---------------------------
   Flight-control variables 
   --------------------------- */
float RatePitch = 0.0f, RateRoll = 0.0f, RateYaw = 0.0f;
float RateCalibrationPitch = 0.0f, RateCalibrationRoll = 0.0f, RateCalibrationYaw = 0.0f;
int RateCalibrationNumber = 0;

float ReceiverValue[8] = {0};
int ChannelNumber = 0;

float Voltage = 0.0f, Current = 0.0f, BatteryRemaining = 0.0f, BatteryAtStart = 0.0f;
float CurrentConsumed = 0.0f;
float BatteryDefault = 1300.0f;

uint32_t LoopTimer = 0;

float DesiredRateRoll = 0.0f, DesiredRatePitch = 0.0f, DesiredRateYaw = 0.0f;
float ErrorRateRoll=0, ErrorRatePitch=0, ErrorRateYaw=0;
float InputRoll=0, InputPitch=0, InputYaw=0;

float PrevErrorRateRoll=0, PrevErrorRatePitch=0, PrevErrorRateYaw=0;
float PrevItermRateRoll=0, PrevItermRatePitch=0, PrevItermRateYaw=0;

float PIDReturn[3] = {0};

float PRateRoll = 0.6f; float PRatePitch = 0.6f; float PRateYaw = 2.0f;
float IRateRoll = 3.5f; float IRatePitch = 3.5f; float IRateYaw = 12.0f;
float DRateRoll = 0.03f; float DRatePitch = 0.03f; float DRateYaw = 0.0f;

float MotorInput1=0, MotorInput2=0, MotorInput3=0, MotorInput4=0;

/* I2C dev address in use */
uint8_t mpu_addr = DEFAULT_I2C_ADDR;

/* ---------------------------
   Helper low-level routines
   --------------------------- */
static bool mpu9250_write_reg(uint8_t reg, uint8_t val) {
    return hal_i2c_write(mpu_addr, reg, &val, 1);
}
static bool mpu9250_read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    return hal_i2c_read(mpu_addr, reg, buf, len);
}

/* Read raw 16-bit signed from two regs (big-endian: H then L) */
static int16_t read_be16(const uint8_t *b) {
    return (int16_t)((b[0] << 8) | b[1]);
}

/* ---------------------------
   MPU9250 initialization & read
   --------------------------- */
bool mpu9250_init(void) {
    /* Reset device and wake up */
    if (!mpu9250_write_reg(MPU9250_REG_PWR_MGMT_1, 0x00)) return false;
    hal_delay_ms(50);

    /* Set sample rate divider (optional) and DLPF config – match Arduino code behavior */
    /* Example: set sample rate divider to 0 (max sample rate), DLPF_CFG=2 (approx 92 Hz) */
    mpu9250_write_reg(MPU9250_REG_SMPLRT_DIV, 0x00);
    mpu9250_write_reg(MPU9250_REG_CONFIG, 0x02);        /* DLPF_CFG = 2 */
    /* Set gyro full-scale to ±500 dps (FS_SEL = 1) -> matches using /65.5 scaling in original code */
    mpu9250_write_reg(MPU9250_REG_GYRO_CONFIG, (1 << 3)); /* FS_SEL=1 */
    /* Accelerometer: keep default ±2g or set as needed */
    mpu9250_write_reg(MPU9250_REG_ACCEL_CONFIG, 0x00); /* AFS=0 => ±2g */

    hal_delay_ms(10);
    return true;
}

/* Read gyros into RateRoll, RatePitch, RateYaw (deg/s) */
bool mpu9250_read_gyro_rates(void) {
    uint8_t buf[6];
    if (!mpu9250_read_regs(MPU9250_REG_GYRO_XOUT_H, buf, 6)) return false;
    int16_t gx = read_be16(buf + 0);
    int16_t gy = read_be16(buf + 2);
    int16_t gz = read_be16(buf + 4);

    /* Convert to degrees/sec using sensitivity for FS_SEL=1 (±500 dps) */
    RateRoll  = (float)gx / GYRO_SENS_FS_SEL_500DPS;   /* match original scaling */
    RatePitch = (float)gy / GYRO_SENS_FS_SEL_500DPS;
    RateYaw   = (float)gz / GYRO_SENS_FS_SEL_500DPS;
    return true;
}

/* ---------------------------
   PID routine 
   --------------------------- */
void pid_equation(float Error, float P , float I, float D,
                  float PrevError, float PrevIterm,
                  float *out_PIDOutput, float *out_PrevError, float *out_PrevIterm) {
    float dt = 0.004f; /* 4 ms loop as in original */
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * dt / 2.0f;
    if (Iterm > 400.0f) Iterm = 400.0f;
    else if (Iterm < -400.0f) Iterm = -400.0f;
    float Dterm = D * (Error - PrevError) / dt;
    float PIDOutput = Pterm + Iterm + Dterm;
    if (PIDOutput > 400.0f) PIDOutput = 400.0f;
    else if (PIDOutput < -400.0f) PIDOutput = -400.0f;
    *out_PIDOutput = PIDOutput;
    *out_PrevError  = Error;
    *out_PrevIterm  = Iterm;
}

/* ---------------------------
   Other helpers: battery, receiver
   --------------------------- */
void battery_voltage(void) {
    /* Replace ADC channel indices with your board mapping */
    int32_t adc_v = hal_adc_read(15); /* example channel for voltage sense */
    int32_t adc_i = hal_adc_read(21); /* example channel for current sense */
    /* Convert ADC to voltage/current using your ADC scaling */
    /* Here we assume hal_adc_read returns ADC such that dividing by 62 as in Arduino code works.
       Implement proper adc scaling on your board. */
    Voltage = (float)adc_v / 62.0f;
    Current = (float)adc_i * 0.089f;
}

void read_receiver(void) {
    /* We assume receiver channels 0..3 mapped to pulsecapture_read_channel indexes.
       Implement mapping / PPM decoding in your HW abstraction. */
    for (int i = 0; i < 4; ++i) {
        uint16_t w = pulsecapture_read_channel(i); /* microseconds */
        if (w == 0) {
            /* if missing, keep previous value or set to a safe default */
        } else {
            ReceiverValue[i] = (float)w;
        }
    }
}

/* ---------------------------
   Reset PID integrators & errors
   --------------------------- */
void reset_pid(void) {
    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0.0f;
    PrevItermRateRoll  = PrevItermRatePitch  = PrevItermRateYaw  = 0.0f;
}

/* ---------------------------
   Setup & main loop
   --------------------------- */
void system_setup(void) {
    /* Initialize HAL hardware pieces:
       - timers, I2C controller, ADC, PWM generator, pulse capture
       Implement these inside your platform code.
    */
    pulsecapture_init(14); /* example RX pin index — adapt to your board */
    hal_delay_ms(10);

    /* Initialize MPU-9250 */
    if (!mpu9250_init()) {
        /* handle error: sensor didn't respond */
        /* you may want to blink an LED or fail safe */
    }

    /* Calibrate gyros: average N samples while stationary */
    RateCalibrationRoll = RateCalibrationPitch = RateCalibrationYaw = 0.0f;
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; ++RateCalibrationNumber) {
        if (!mpu9250_read_gyro_rates()) {
            /* error reading sensor — small delay & retry or abort */
        }
        RateCalibrationRoll  += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw   += RateYaw;
        hal_delay_ms(1);
    }
    RateCalibrationRoll  /= 2000.0f;
    RateCalibrationPitch /= 2000.0f;
    RateCalibrationYaw   /= 2000.0f;

    /* battery init */
    battery_voltage();
    if (Voltage > 8.3f) {
        /* LED OFF behavior handled by your board */
        BatteryAtStart = BatteryDefault;
    } else if (Voltage < 7.5f) {
        BatteryAtStart = 0.30f * BatteryDefault;
    } else {
        BatteryAtStart = ((82.0f * Voltage - 580.0f) / 100.0f) * BatteryDefault;
    }

    /* Wait for receiver arming: throttle channel low ~1020..1050 in original */
    while (ReceiverValue[2] < 1020.0f || ReceiverValue[2] > 1050.0f) {
        read_receiver();
        hal_delay_ms(4);
    }

    LoopTimer = hal_get_micros();
    reset_pid();
}

/* Main loop: intended to run in a tight real-time thread */
void system_loop(void) {
    /* Read sensors */
    mpu9250_read_gyro_rates();

    /* Remove calibrated bias */
    RateRoll  -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw   -= RateCalibrationYaw;

    /* read rc */
    read_receiver();

    /* convert sticks to desired rates (same factor as original) */
    DesiredRateRoll  = 0.15f * (ReceiverValue[0] - 1500.0f);
    DesiredRatePitch = 0.15f * (ReceiverValue[1] - 1500.0f);
    float InputThrottle = ReceiverValue[2];
    DesiredRateYaw   = 0.15f * (ReceiverValue[3] - 1500.0f);

    /* compute errors */
    ErrorRateRoll  = DesiredRateRoll  - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw   = DesiredRateYaw   - RateYaw;

    /* PID roll */
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll,
                 PrevErrorRateRoll, PrevItermRateRoll,
                 &PIDReturn[0], &PIDReturn[1], &PIDReturn[2]);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    /* PID pitch */
    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch,
                 PrevErrorRatePitch, PrevItermRatePitch,
                 &PIDReturn[0], &PIDReturn[1], &PIDReturn[2]);
    InputPitch = PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];

    /* PID yaw */
    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw,
                 PrevErrorRateYaw, PrevItermRateYaw,
                 &PIDReturn[0], &PIDReturn[1], &PIDReturn[2]);
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[1];
    PrevItermRateYaw = PIDReturn[2];

    /* clamp throttle */
    if (InputThrottle > 1800.0f) InputThrottle = 1800.0f;

    /* Motor mixing (same formula as Arduino) */
    MotorInput1 = 1.024f * (InputThrottle - InputRoll - InputPitch - InputYaw);
    MotorInput2 = 1.024f * (InputThrottle - InputRoll + InputPitch + InputYaw);
    MotorInput3 = 1.024f * (InputThrottle + InputRoll + InputPitch - InputYaw);
    MotorInput4 = 1.024f * (InputThrottle + InputRoll - InputPitch + InputYaw);

    /* clamp */
    if (MotorInput1 > 2000.0f) MotorInput1 = 1999.0f;
    if (MotorInput2 > 2000.0f) MotorInput2 = 1999.0f;
    if (MotorInput3 > 2000.0f) MotorInput3 = 1999.0f;
    if (MotorInput4 > 2000.0f) MotorInput4 = 1999.0f;

    int ThrottleIdle = 1180;
    if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

    int ThrottleCutOff = 1000;
    if (ReceiverValue[2] < 1050.0f) {
        MotorInput1 = ThrottleCutOff;
        MotorInput2 = ThrottleCutOff;
        MotorInput3 = ThrottleCutOff;
        MotorInput4 = ThrottleCutOff;
        reset_pid();
    }

    /* Output to ESCs (pwm in microseconds) */
    hal_pwm_write(1, (uint16_t)MotorInput1);
    hal_pwm_write(2, (uint16_t)MotorInput2);
    hal_pwm_write(3, (uint16_t)MotorInput3);
    hal_pwm_write(4, (uint16_t)MotorInput4);

    /* battery & current integration */
    battery_voltage();
    CurrentConsumed = Current * 1000.0f * 0.004f / 3600.0f + CurrentConsumed;
    BatteryRemaining = (BatteryAtStart - CurrentConsumed) / BatteryDefault * 100.0f;
    /* toggle LED or alarm if battery low — implement via HAL (not shown) */

    /* Timing: enforce 4 ms loop like Arduino */
    while ((hal_get_micros() - LoopTimer) < 4000u) {
        /* busy wait; you could sleep/yield depending on your RTOS */
    }
    LoopTimer = hal_get_micros();
}

/* ---------------------------
   Example main: call setup & loop
   --------------------------- */
int main(void) {
    /* Call board/platform init (GPIO, clocks, peripherals) here */
    system_setup();
    for (;;) {
        system_loop();
    }
    return 0;
}
