#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include <stdio.h>
 // Or the appropriate path for the sensor library
#include <stdbool.h> // For true/false definitions
#include <math.h> // Add this at the beginning of your code
// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



#define ISM330DLC_I2C_ADDRESS 0x6A << 1
#define WHO_AM_I_REG 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define OUTX_L_G 0x22
#define OUTX_L_XL 0x28
#define FREQ 100  // Sampling frequency in Hz


I2C_HandleTypeDef hi2c1;
extern USBD_HandleTypeDef hUsbDeviceFS;

// Motor control states
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_CW,
    MOTOR_CCW
} MotorState;

// Define pin mappings for motor control and button
#define PUL_PIN        GPIO_PIN_2     // PUL+ (PA2)
#define DIR_PIN        GPIO_PIN_3     // DIR+ (PA3)
#define BUTTON_PIN     GPIO_PIN_0     // B1 button (PA0)
#define MOTOR_PORT     GPIOA          // Motor GPIO Port

MotorState motorState = MOTOR_STOP;   // Initial state: motor stopped

// USB buffer
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t buffer[64] = {0};
char data[64] = "Motor Control Ready\n";

// Forward declaration of functions
void SystemClock_Config(void);
void GPIO_Init(void);
void ISM330DLC_Init(void);
void ISM330DLC_ReadAccelerometer(int16_t *accel_data);
void ISM330DLC_ReadGyroscope(int16_t *gyro_data);
void MX_USB_DEVICE_Init(void);
void Motor_Control(MotorState state);
void Process_USB_Command(uint8_t* Buf, uint16_t Len);
void I2C1_Init(void);
void CDC_Transmit_Data(char* data);
void calibrate(void);

int16_t gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
int16_t accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;
double temperature = 25.0;  // Default temperature in degrees Celsius
typedef struct {
    double q;  // Process noise covariance
    double r;  // Measurement noise covariance
    double x;  // Value
    double p;  // Estimation error covariance
    double k;  // Kalman gain
} KalmanFilter;

void KalmanFilter_Init(KalmanFilter* filter, double q, double r, double initial_value) {
    filter->q = q;
    filter->r = r;
    filter->x = initial_value;
    filter->p = 1.0;
    filter->k = 0.0;
}

double KalmanFilter_Update(KalmanFilter* filter, double measurement) {
    // Prediction update
    filter->p += filter->q;

    // Measurement update
    filter->k = filter->p / (filter->p + filter->r);
    filter->x += filter->k * (measurement - filter->x);
    filter->p *= (1 - filter->k);

    return filter->x;
}

// Global Kalman filters for accelerometer and gyroscope
KalmanFilter kalmanAccelX, kalmanAccelY, kalmanAccelZ;
KalmanFilter kalmanGyroX, kalmanGyroY, kalmanGyroZ;

// Sensitivity values for accelerometer and gyroscope based on full-scale range
double accelSensitivity = 0.488;  // mg/LSB for FS = ±2g
double gyroSensitivity = 4.375;    // mdps/LSB for FS = ±250 dps

double convertAccelToMg(int16_t rawValue) {
    return rawValue * accelSensitivity;
}

double convertGyroToDps(int16_t rawValue) {
    return rawValue * gyroSensitivity / 10.0;
}

void calibrate() {
    int x;
    long xSumGyro = 0, ySumGyro = 0, zSumGyro = 0;
    long xSumAccel = 0, ySumAccel = 0, zSumAccel = 0;
    int num = 500;

    int16_t accel_data[3] = {0};
    int16_t gyro_data[3] = {0};

    for (x = 0; x < num; x++) {
        // Read gyroscope data
        ISM330DLC_ReadGyroscope(gyro_data);
        xSumGyro += gyro_data[0];
        ySumGyro += gyro_data[1];
        zSumGyro += gyro_data[2];

        // Read accelerometer data
        ISM330DLC_ReadAccelerometer(accel_data);
        xSumAccel += accel_data[0];
        ySumAccel += accel_data[1];
        zSumAccel += accel_data[2];

        HAL_Delay(5);  // Small delay between readings
    }

    // Calculate offsets by averaging readings
    gyroXOffset = xSumGyro / num;
    gyroYOffset = ySumGyro / num;
    gyroZOffset = zSumGyro / num;

    accelXOffset = xSumAccel / num;
    accelYOffset = ySumAccel / num;
    accelZOffset = zSumAccel / num;

    // Optional: Print calibration results
    char calibMsg[128];
    snprintf(calibMsg, sizeof(calibMsg),
             "Calibration complete: Gyro Offsets (X: %d, Y: %d, Z: %d), Accel Offsets (X: %d, Y: %d, Z: %d)\n",
             gyroXOffset, gyroYOffset, gyroZOffset, accelXOffset, accelYOffset, accelZOffset);
    CDC_Transmit_Data(calibMsg);
}

int main(void) {
    // Step 1: Initialize HAL and System
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    I2C1_Init();
    MX_USB_DEVICE_Init(); // Initialize USB device first
    ISM330DLC_Init();

    // Perform sensor calibration
    calibrate();

    // Initialize Kalman filters for accelerometer and gyroscope
    KalmanFilter_Init(&kalmanAccelX, 0.01, 0.05, 0);
    KalmanFilter_Init(&kalmanAccelY, 0.01, 0.05, 0);
    KalmanFilter_Init(&kalmanAccelZ, 0.01, 0.05, 0);
    KalmanFilter_Init(&kalmanGyroX, 0.01, 0.05, 0);
    KalmanFilter_Init(&kalmanGyroY, 0.01, 0.05, 0);
    KalmanFilter_Init(&kalmanGyroZ, 0.01, 0.05, 0);

    // Send initial message to indicate USB is ready
    char* initMsg = "USB Initialized. Motor Control Ready\n";
    CDC_Transmit_Data(initMsg);

    int16_t accel_data[3] = {0};
    int16_t gyro_data[3] = {0};
    double ax = 0, ay = 0, gx = 0.0, gy = 0.0, gz = 0.0;
    uint16_t lastMotorUpdate = 0;   // Timestamp for motor updates
    uint16_t lastSensorUpdate = 0;  // Timestamp for sensor updates
    uint16_t motorInterval = 1;     // Motor update interval in milliseconds
    uint16_t sensorInterval = 500;  // Sensor update interval in milliseconds

    while (1) {
        // Check if a new command has been received over USB
        if (strlen((char*)buffer) > 0) {
            // Process the received command
            Process_USB_Command(buffer, strlen((char*)buffer));

            // Clear the buffer after processing
            memset(buffer, 0, sizeof(buffer));
        }

        // Run the motor based on the current state at a higher frequency
        if (HAL_GetTick() - lastMotorUpdate >= motorInterval) {
            Motor_Control(motorState);
            lastMotorUpdate = HAL_GetTick(); // Update timestamp
        }

        // Read sensors and print IMU data to USB at a slower frequency
        if (HAL_GetTick() - lastSensorUpdate >= sensorInterval) {
            ISM330DLC_ReadAccelerometer(accel_data);
            ISM330DLC_ReadGyroscope(gyro_data);

            // Apply calibration offsets to the gyroscope data
            gyro_data[0] -= gyroXOffset;
            gyro_data[1] -= gyroYOffset;
            gyro_data[2] -= gyroZOffset;

            // Apply calibration offsets to the accelerometer data
            accel_data[0] -= accelXOffset;
            accel_data[1] -= accelYOffset;
            accel_data[2] -= accelZOffset;

            // Convert raw data to meaningful units
            double accel_x_mg = convertAccelToMg(accel_data[0]);
            double accel_y_mg = convertAccelToMg(accel_data[1]);
            double accel_z_mg = convertAccelToMg(accel_data[2]);

            double gyro_x_dps = convertGyroToDps(gyro_data[0]);
            double gyro_y_dps = convertGyroToDps(gyro_data[1]);
            double gyro_z_dps = convertGyroToDps(gyro_data[2]);

            // Apply Kalman filter to accelerometer data
            double filtered_ax = KalmanFilter_Update(&kalmanAccelX, accel_x_mg);
            double filtered_ay = KalmanFilter_Update(&kalmanAccelY, accel_y_mg);
            double filtered_az = KalmanFilter_Update(&kalmanAccelZ, accel_z_mg);

            // Apply Kalman filter to gyroscope data
            double filtered_gx = KalmanFilter_Update(&kalmanGyroX, gyro_x_dps);
            double filtered_gy = KalmanFilter_Update(&kalmanGyroY, gyro_y_dps);
            double filtered_gz = KalmanFilter_Update(&kalmanGyroZ, gyro_z_dps);

            // Calculate angles based on accelerometer with proper type casting
            ay = atan2(filtered_ax, sqrt(pow(filtered_ay, 2) + pow(filtered_az, 2))) * 180 / M_PI;
            ax = atan2(filtered_ay, sqrt(pow(filtered_ax, 2) + pow(filtered_az, 2))) * 180 / M_PI;

            // Integrate gyro data to get angles (assuming FREQ is your sampling frequency)
            gx += (filtered_gx / FREQ);
            gy += (filtered_gy / FREQ);
            gz += (filtered_gz / FREQ);

            // Wrap gyro angles to maintain 0-360 range
            if (gx >= 360) gx -= 360;
            if (gx < 0) gx += 360;
            if (gy >= 360) gy -= 360;
            if (gy < 0) gy += 360;
            if (gz >= 360) gz -= 360;
            if (gz < 0) gz += 360;

            // Apply complementary filter
            gx = gx * 0.98 + ax * 0.02;
            gy = gy * 0.98 + ay * 0.02;

            // Apply temperature compensation for gyroscope (example implementation)
            double temperatureFactor = 1.0 + 0.015 * (temperature - 25.0);  // Compensate based on temperature drift
            filtered_gx *= temperatureFactor;
            filtered_gy *= temperatureFactor;
            filtered_gz *= temperatureFactor;

            // Print accelerometer and gyroscope values to USB
            char imuData[128];
            int ret = snprintf(imuData, sizeof(imuData),
                               "Gyro X: %.2f, Y: %.2f, Z: %.2f, Accel X: %.2f, Y: %.2f, Z: %.2f\n",
                               filtered_gx, filtered_gy, filtered_gz,
                               filtered_ax, filtered_ay, filtered_az);

            // Check if the output was truncated
            if (ret >= 0 && ret < sizeof(imuData)) {
                CDC_Transmit_Data(imuData);
            } else {
                CDC_Transmit_Data("Error: IMU data output truncated\n");
            }

            // Add delay to ensure data is properly transmitted before sending more
            HAL_Delay(10);  // 10 ms delay to ensure USB buffer readiness

            // Print calculated angle data if valid
            if (!isnan(ax) && !isnan(ay) && !isnan(gx) && !isnan(gy) && !isnan(gz)) {
                char angleData[128];
                int ret2 = snprintf(angleData, sizeof(angleData),
                                    "Angle ax: %.2f, ay: %.2f, gx: %.2f, gy: %.2f, gz: %.2f\n",
                                    ax, ay, gx, gy, gz);

                // Check if the output was truncated
                if (ret2 >= 0 && ret2 < sizeof(angleData)) {
                    CDC_Transmit_Data(angleData);
                } else {
                    CDC_Transmit_Data("Error: Angle data output truncated\n");
                }
            } else {
                CDC_Transmit_Data("Error: Calculated angles are NaN\n");
            }

            lastSensorUpdate = HAL_GetTick(); // Update timestamp
        }
    }
}

void I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();  // Enable I2C1 clock

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        // Initialization Error
        Error_Handler();
    }
}

void ISM330DLC_Init(void)
{
    uint8_t config_data[] = {0x40, 0x40};  // Configure accelerometer for ±2g and gyroscope for ±250 dps
    HAL_I2C_Mem_Write(&hi2c1, ISM330DLC_I2C_ADDRESS, CTRL1_XL, 1, &config_data[0], 1, 100);
    HAL_I2C_Mem_Write(&hi2c1, ISM330DLC_I2C_ADDRESS, CTRL2_G, 1, &config_data[1], 1, 100);
}

// Function to read accelerometer data
void ISM330DLC_ReadAccelerometer(int16_t *accel_data)
{
    uint8_t accel_buffer[6];
    if (HAL_I2C_Mem_Read(&hi2c1, ISM330DLC_I2C_ADDRESS, OUTX_L_XL, 1, accel_buffer, 6, 100) == HAL_OK) {
        accel_data[0] = (int16_t)(accel_buffer[1] << 8 | accel_buffer[0]);
        accel_data[1] = (int16_t)(accel_buffer[3] << 8 | accel_buffer[2]);
        accel_data[2] = (int16_t)(accel_buffer[5] << 8 | accel_buffer[4]);
    }
}

// Function to read gyroscope data
void ISM330DLC_ReadGyroscope(int16_t *gyro_data)
{
    uint8_t gyro_buffer[6];
    if (HAL_I2C_Mem_Read(&hi2c1, ISM330DLC_I2C_ADDRESS, OUTX_L_G, 1, gyro_buffer, 6, 100) == HAL_OK) {
        gyro_data[0] = (int16_t)(gyro_buffer[1] << 8 | gyro_buffer[0]);
        gyro_data[1] = (int16_t)(gyro_buffer[3] << 8 | gyro_buffer[2]);
        gyro_data[2] = (int16_t)(gyro_buffer[5] << 8 | gyro_buffer[4]);
    }
}
// Function to control motor based on its state
void Motor_Control(MotorState state) {
    switch (state) {
        case MOTOR_CW:
            // Set DIR+ high for clockwise direction
            HAL_GPIO_WritePin(MOTOR_PORT, DIR_PIN, GPIO_PIN_SET);

            // Generate a pulse on PUL+
            HAL_GPIO_WritePin(MOTOR_PORT, PUL_PIN, GPIO_PIN_SET);
            HAL_Delay(.1);  // Short pulse
            HAL_GPIO_WritePin(MOTOR_PORT, PUL_PIN, GPIO_PIN_RESET);
            break;

        case MOTOR_CCW:
            // Set DIR+ low for counterclockwise direction
            HAL_GPIO_WritePin(MOTOR_PORT, DIR_PIN, GPIO_PIN_RESET);

            // Generate a pulse on PUL+
            HAL_GPIO_WritePin(MOTOR_PORT, PUL_PIN, GPIO_PIN_SET);
            HAL_Delay(.1);  // Short pulse
            HAL_GPIO_WritePin(MOTOR_PORT, PUL_PIN, GPIO_PIN_RESET);
            break;

        case MOTOR_STOP:
        default:
            // Stop motor (no pulses generated)
            HAL_GPIO_WritePin(MOTOR_PORT, PUL_PIN, GPIO_PIN_RESET);
            break;
    }
}

// Function to process USB commands received from PuTTY
void Process_USB_Command(uint8_t* Buf, uint16_t Len)
{
    // Null-terminate the received buffer to avoid trailing characters
    Buf[Len] = '\0';

    if (strncmp((char*)Buf, "start_cw", strlen("start_cw")) == 0) {
        motorState = MOTOR_CW;
        CDC_Transmit_Data("Motor started clockwise\n");
    }
    else if (strncmp((char*)Buf, "start_ccw", strlen("start_ccw")) == 0) {
        motorState = MOTOR_CCW;
        CDC_Transmit_Data("Motor started counterclockwise\n");
    }
    else if (strncmp((char*)Buf, "stop", strlen("stop")) == 0) {
        motorState = MOTOR_STOP;
        CDC_Transmit_Data("Motor stopped\n");
    }
    else {
        CDC_Transmit_Data("Invalid command\n");
    }
}


// Function to transmit data via USB CDC to PuTTY
void CDC_Transmit_Data(char* data)
{
    CDC_Transmit_FS((uint8_t*)data, strlen(data));
    HAL_Delay(.1);  // Delay to ensure data transmission
}

// GPIO initialization function
void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable GPIOA clock

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure PUL+ and DIR+ as output
    GPIO_InitStruct.Pin = PUL_PIN | DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_PORT, &GPIO_InitStruct);

    // Configure B1 button as input (PA0)
    GPIO_InitStruct.Pin = BUTTON_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MOTOR_PORT, &GPIO_InitStruct);
}
int __io_putchar(int ch) {
    uint8_t data = ch;
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &data, 1);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
    return ch;
}
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initializes the RCC Oscillators
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        // Initialization Error
        while(1);
    }

    // Initializes the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        // Initialization Error
        while(1);
    }
}
void Error_Handler(void)
{
    // Error handling code (infinite loop or system reset)
    while(1)
    {
        // Optionally toggle an LED or other indicator
    }
}
