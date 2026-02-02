#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ==========================================
// 1. PID CONFIGURATION (TUNING)
// ==========================================
// These would be tuned in real life. 
// Kp: Strength of reaction
// Ki: Fixes long-term drift
// Kd: Dampens vibration/overshoot
#define KP_PITCH  1.50
#define KI_PITCH  0.02
#define KD_PITCH  0.50

#define KP_ROLL   1.50
#define KI_ROLL   0.02
#define KD_ROLL   0.50

// Base throttle for hover (e.g., 1500us for PWM)
#define BASE_THROTTLE 1500 

// PID Structure to keep track of state
typedef struct {
    double kp;
    double ki;
    double kd;
    double previous_error;
    double integral;
} PID_Controller;

// ==========================================
// 2. PID LOGIC FUNCTION
// ==========================================
double calculate_pid(PID_Controller *pid, double setpoint, double current_value, double dt) {
    // 1. Calculate Error (Target - Actual)
    double error = setpoint - current_value;

    // 2. Proportional Term (The "Present")
    double P = pid->kp * error;

    // 3. Integral Term (The "Past")
    pid->integral += error * dt;
    double I = pid->ki * pid->integral;

    // 4. Derivative Term (The "Future")
    // (Error - Previous_Error) / dt = Rate of change of error
    double derivative = (error - pid->previous_error) / dt;
    double D = pid->kd * derivative;

    // 5. Update state for next loop
    pid->previous_error = error;

    // Total Correction
    return P + I + D;
}

// ==========================================
// 3. MAIN SIMULATION LOOP
// ==========================================
int main() {
    FILE *fp_in = fopen("mpu_flight.csv", "r");
    FILE *fp_out = fopen("pid_simulation_results.csv", "w");

    if (!fp_in || !fp_out) {
        perror("Error opening files");
        return -1;
    }

    // Initialize PID Controllers for Pitch and Roll
    PID_Controller pid_pitch = {KP_PITCH, KI_PITCH, KD_PITCH, 0, 0};
    PID_Controller pid_roll  = {KP_ROLL,  KI_ROLL,  KD_ROLL,  0, 0};

    char line[1024];
    double prev_time = 0.0;
    int first_line = 1;

    // Write Header for Output CSV
    fprintf(fp_out, "time_s,pitch_in,roll_in,motor_front,motor_back,motor_left,motor_right,correction_pitch,correction_roll\n");

    printf("Simulating PID Loop...\n");

    // Read CSV line by line
    while (fgets(line, 1024, fp_in)) {
        // Skip header line of input CSV
        if (strncmp(line, "time_s", 6) == 0) continue;

        double time_s, pitch, roll, yaw;
        int ax, ay, az;

        // Parse line: time_s,ax,ay,az,pitch,roll,yaw
        sscanf(line, "%lf,%d,%d,%d,%lf,%lf,%lf", &time_s, &ax, &ay, &az, &pitch, &roll, &yaw);

        // Calculate dt (delta time) since last sample
        double dt = time_s - prev_time;
        if (first_line) { dt = 0.01; first_line = 0; } // Avoid div by zero on first sample
        prev_time = time_s;

        // --- THE CONTROL LOOP ---
        
        // Target is 0.0 (Level Flight)
        double pitch_correction = calculate_pid(&pid_pitch, 0.0, pitch, dt);
        double roll_correction  = calculate_pid(&pid_roll,  0.0, roll,  dt);

        // --- MOTOR MIXING ---
        // Quadcopter '+' or 'x' configuration mixing logic
        // Front/Back motors handle Pitch
        // Left/Right motors handle Roll
        
        // If Pitch is positive (Nose up), we need to speed up Back motor, slow down Front.
        double motor_front = BASE_THROTTLE - pitch_correction;
        double motor_back  = BASE_THROTTLE + pitch_correction;

        // If Roll is positive (Right wing down), we need to speed up Right motor.
        double motor_left  = BASE_THROTTLE + roll_correction;
        double motor_right = BASE_THROTTLE - roll_correction;

        // Clamp values to realistic motor PWM range (e.g., 1000-2000)
        if (motor_front > 2000) motor_front = 2000; if (motor_front < 1000) motor_front = 1000;
        if (motor_back  > 2000) motor_back  = 2000; if (motor_back  < 1000) motor_back  = 1000;
        if (motor_left  > 2000) motor_left  = 2000; if (motor_left  < 1000) motor_left  = 1000;
        if (motor_right > 2000) motor_right = 2000; if (motor_right < 1000) motor_right = 1000;

        // Write to output file
        fprintf(fp_out, "%.4f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f,%.2f,%.2f\n", 
                time_s, pitch, roll, motor_front, motor_back, motor_left, motor_right, pitch_correction, roll_correction);
    }

    fclose(fp_in);
    fclose(fp_out);
    printf("Simulation Complete. Results saved to 'pid_simulation_results.csv'\n");
    return 0;
}
