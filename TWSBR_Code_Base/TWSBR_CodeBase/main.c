#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#include "TWSBR_MPU9250.h"
#include "TWSBR_Wheel.h"
#include "TWSBR_PIDController.h"

// Global variables - ALL PRESERVED
uint8_t accel_reg[6];
int16_t accelXYZ_int[3];
float accelXYZ[3];
bool busStatus;
float pitchAngle;

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t ui32SystemClock;
volatile uint32_t ui8Adjust1, ui8Adjust2, ui8Adjust3, ui8Adjust4;
float test1, test2, test3;

uint32_t encoderPosn[2];

// Add gyro variable
float gyroXYZ[3];  // Gyroscope data in dps
float filteredPitch;

float control_output;

// LED control variables
bool ledState = false;
uint32_t blinkInterval = 1000; // Start with 1s default
uint32_t ledCounter = 0;       // Simple counter for timing

// Function to initialize LED
void init_led(void)
{
    // Enable GPIO Port F (assuming red LED on PF1 on Tiva C Launchpad)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Configure LED pin as output
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    // Turn off LED initially
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
}

// Function to update LED blink speed based on angle (0-30 degree range)
void update_led_blink_speed(float angle)
{
    // Take absolute value since we care about magnitude, not direction
    float abs_angle = fabsf(angle);

    // Map 0-30 degrees to blink intervals (0° = 500ms, 30° = 2000ms)
    if (abs_angle < 2.0f) {
        // Very close to 0° - 500ms blink
        blinkInterval = 50;
    } else if (abs_angle < 5.0f) {
        // Close to 0° - 750ms blink
        blinkInterval = 80;
    } else if (abs_angle < 10.0f) {
        // Small angle - 1000ms blink
        blinkInterval = 100;
    } else if (abs_angle < 20.0f) {
        // Medium angle - 1500ms blink
        blinkInterval = 150;
    } else {
        // Large angle (20-30°) - 2000ms blink
        blinkInterval = 200;
    }
}
// Function to handle LED blinking (call this in main loop)
void handle_led_blinking(void)
{
    // Since main loop runs every 10ms, we count in 10ms increments
    // Calculate how many 10ms cycles needed for the desired interval
    uint32_t cyclesNeeded = blinkInterval / 10;

    ledCounter++;

    if (ledCounter >= cyclesNeeded) {
        // Toggle LED state
        ledState = !ledState;

        if (ledState) {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED ON
        } else {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); // LED OFF
        }

        ledCounter = 0; // Reset counter
    }
}

// Simple debug function to test LED
void test_led(void)
{
    // Quick test: blink LED 3 times fast
    uint8_t i;
    for(i = 0; i < 6; i++) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1); // LED ON
        delay_ms(100);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0); // LED OFF
        delay_ms(100);
    }
}

int main(void)
{
    // Initialize system clock
    SysCtlClockSet(SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_SYSDIV_1);

    // Initialize LED
    init_led();

    // Test LED to make sure it works
    test_led();
    delay_ms(500);

    mpu9250_init();
    delay_ms(100);
    wheels_init();
    PID_Controller balance_pid = {
        .Kp = 20.0f,
        .Ki = 0.0f,
        .Kd = 5.0f,
        .integral = 0.0f,
        .prev_error = 0.0f,
        .integral_limit = 50.0f
    };

    // Calibrate sensors (keep stationary during calibration)
    mpu9250_calculate_bias();      // Calibrate accelerometer
    mpu9250_calibrate_gyro_bias(); // Calibrate gyroscope


    while(true)
    {
        // Read accelerometer data
        mpu9250_readAcceleration(accel_reg, accelXYZ);

        // Read gyroscope data (with bias correction)
        mpu9250_get_gyro(gyroXYZ);

        // Calculate pitch angle using your existing function
        pitchAngle = mpu9250_calculate_pitch_degrees_robust(accelXYZ[0], accelXYZ[1], accelXYZ[2]);

        mpu9250_kalmanFilter(&filteredPitch, gyroXYZ, accelXYZ);

        // Update LED blink speed based on filtered pitch angle
        update_led_blink_speed(filteredPitch);

        // Handle LED blinking
        handle_led_blinking();

        // Calculate PID control output using the CORRECT function name
        control_output = calculate_pid_control(&balance_pid, filteredPitch, gyroXYZ[0]);

        // Test calculations (preserved from original)
        test1 = atan2(-0.5f, 1.0f) * (180.0f / M_PI); // Should be ~ -26.565°
        test2 = atan2(0.0f, 1.0f) * (180.0f / M_PI);  // Should be 0°
        test3 = atan2(1.0f, 1.0f) * (180.0f / M_PI);  // Should be 45°

        // Check I2C bus status
        check_i2c_bus_status();

        // Now you have the control_output to drive your wheels
        // Example wheel control (adjust based on your wheel_setSpeed function):
         wheel_setSpeed(-control_output, 0);

         wheels_getPosition(encoderPosn);

        // Delay before next read
        delay_ms(10);
    }

    return 0;
}

// ALL YOUR EXISTING FUNCTIONS REMAIN HERE (delay_ms, delay_us, check_i2c_bus_status,
// recover_i2c_bus, mpu9250_init, mpu9250_readBytes, mpu9250_calculate_bias,
// mpu9250_apply_bias_correction, mpu9250_readAcceleration,
// mpu9250_calculate_pitch_degrees_robust, and all gyro functions)
// [ALL YOUR EXISTING CODE AFTER main() REMAINS UNCHANGED]
