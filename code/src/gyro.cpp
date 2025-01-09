#include <mbed.h>                        // Include the mbed library for hardware abstraction
#include "gyro.h"                        // Include the custom gyroscope header file

// Initialize SPI communication for the gyroscope
SPI gyroscope(PF_9, PF_8, PF_7);           // MOSI on PF_9, MISO on PF_8, SCLK on PF_7
DigitalOut cs(PC_1);                        // Chip Select (CS) pin on PC_1 for SPI communication

// Calibration thresholds for each axis (initialized to zero)
int16_t x_threshold;                         // X-axis calibration threshold
int16_t y_threshold;                         // Y-axis calibration threshold
int16_t z_threshold;                         // Z-axis calibration threshold

// Zero-rate level samples for each axis (initialized to zero)
int16_t x_sample;                            // X-axis zero-rate level sample
int16_t y_sample;                            // Y-axis zero-rate level sample
int16_t z_sample;                            // Z-axis zero-rate level sample

float sensitivity = 0.0f;                    // Sensitivity factor based on full-scale selection

Gyroscope_RawData *gyro_raw;                  // Pointer to store raw gyroscope data

/*******************************************************************************
 * Function: WriteByte
 * -----------------------------------------------------------------------------
 * Sends a single byte to the gyroscope via SPI.
 *
 * Parameters:
 *  - address: Register address to write to.
 *  - data: Data byte to write to the specified register.
 *
 * Returns:
 *  - None
 ******************************************************************************/
void WriteByte(uint8_t address, uint8_t data)
{
    cs = 0;                                    // Activate the gyroscope by pulling CS low
    gyroscope.write(address);                  // Send the register address
    gyroscope.write(data);                     // Send the data byte to the register
    cs = 1;                                    // Deactivate the gyroscope by pulling CS high
}

/*******************************************************************************
 * Function: GetGyroValue
 * -----------------------------------------------------------------------------
 * Reads raw gyroscope data from the device via SPI.
 *
 * Parameters:
 *  - rawdata: Pointer to a Gyroscope_RawData structure to store the read values.
 *
 * Returns:
 *  - None
 ******************************************************************************/
void GetGyroValue(Gyroscope_RawData *rawdata)
{
    cs = 0;                                    // Activate the gyroscope by pulling CS low
    gyroscope.write(OUT_X_L | 0x80 | 0x40);     // Send the OUT_X_L register address with read and auto-increment bits set
    rawdata->x_raw = gyroscope.write(0xff) | (gyroscope.write(0xff) << 8); // Read X-axis low and high bytes
    rawdata->y_raw = gyroscope.write(0xff) | (gyroscope.write(0xff) << 8); // Read Y-axis low and high bytes
    rawdata->z_raw = gyroscope.write(0xff) | (gyroscope.write(0xff) << 8); // Read Z-axis low and high bytes
    cs = 1;                                    // Deactivate the gyroscope by pulling CS high
}

/*******************************************************************************
 * Function: CalibrateGyroscope
 * -----------------------------------------------------------------------------
 * Calibrates the gyroscope by determining the zero-rate level for each axis.
 * It also sets up thresholds to filter out minor vibrations.
 *
 * Parameters:
 *  - rawdata: Pointer to a Gyroscope_RawData structure to store calibration data.
 *
 * Returns:
 *  - None
 ******************************************************************************/
void CalibrateGyroscope(Gyroscope_RawData *rawdata)
{
    int16_t sumX = 0;                          // Accumulator for X-axis samples
    int16_t sumY = 0;                          // Accumulator for Y-axis samples
    int16_t sumZ = 0;                          // Accumulator for Z-axis samples

    printf("========[Calibrating...]========\r\n"); // Notify start of calibration

    // Collect 128 samples to calculate the average zero-rate level
    for (int i = 0; i < 128; i++)
    {
        GetGyroValue(rawdata);                  // Read raw gyroscope data
        sumX += rawdata->x_raw;                 // Accumulate X-axis data
        sumY += rawdata->y_raw;                 // Accumulate Y-axis data
        sumZ += rawdata->z_raw;                 // Accumulate Z-axis data

        // Update the maximum thresholds based on current samples
        x_threshold = max(x_threshold, rawdata->x_raw);
        y_threshold = max(y_threshold, rawdata->y_raw);
        z_threshold = max(z_threshold, rawdata->z_raw);

        wait_us(10000);                          // Wait for 10 milliseconds between samples
    }

    // Calculate the average (zero-rate level) for each axis
    x_sample = sumX >> 7;                       // Equivalent to sumX / 128
    y_sample = sumY >> 7;                       // Equivalent to sumY / 128
    z_sample = sumZ >> 7;                       // Equivalent to sumZ / 128

    printf("========[Calibration finish.]========\r\n"); // Notify end of calibration
}

/*******************************************************************************
 * Function: InitiateGyroscope
 * -----------------------------------------------------------------------------
 * Initializes the gyroscope by configuring control registers and performing calibration.
 *
 * Parameters:
 *  - init_parameters: Pointer to a Gyroscope_Init_Parameters structure containing
 *                     configuration settings.
 *  - init_raw_data: Pointer to a Gyroscope_RawData structure to store initial raw data.
 *
 * Returns:
 *  - None
 ******************************************************************************/
void InitiateGyroscope(Gyroscope_Init_Parameters *init_parameters, Gyroscope_RawData *init_raw_data)
{
    printf("\r\n========[Initializing gyroscope...]========\r\n"); // Notify start of initialization

    gyro_raw = init_raw_data;                      // Assign the raw data pointer for global access
    cs = 1;                                        // Ensure the gyroscope is inactive initially

    // Configure SPI settings for the gyroscope
    gyroscope.format(8, 3);                        // Set SPI to 8 bits per frame, mode 3 (polarity 1, phase 1)
    gyroscope.frequency(1000000);                  // Set SPI clock frequency to 1 MHz

    // Configure gyroscope control registers
    WriteByte(CTRL_REG_1, init_parameters->conf1 | POWERON); // Set Output Data Rate, bandwidth, and enable all 3 axes
    WriteByte(CTRL_REG_3, init_parameters->conf3);           // Enable Data Ready interrupt on INT2 pin
    WriteByte(CTRL_REG_4, init_parameters->conf4);           // Set full-scale range and other configurations

    // Set sensitivity based on full-scale selection
    switch (init_parameters->conf4)
    {
    case FULL_SCALE_245:
        sensitivity = SENSITIVITY_245;             // Sensitivity for ±245 dps
        break;

    case FULL_SCALE_500:
        sensitivity = SENSITIVITY_500;             // Sensitivity for ±500 dps
        break;

    case FULL_SCALE_2000:
        sensitivity = SENSITIVITY_2000;            // Sensitivity for ±2000 dps
        break;

    case FULL_SCALE_2000_ALT:
        sensitivity = SENSITIVITY_2000;            // Sensitivity for alternative ±2000 dps setting
        break;
    }

    CalibrateGyroscope(gyro_raw);                   // Perform calibration to determine zero-rate levels and thresholds
    printf("========[Initiation finish.]========\r\n"); // Notify end of initialization
}

/*******************************************************************************
 * Function: ConvertToDPS
 * -----------------------------------------------------------------------------
 * Converts raw gyroscope data to degrees per second (DPS) based on sensitivity.
 *
 * Parameters:
 *  - axis_data: Raw gyroscope data for a single axis.
 *
 * Returns:
 *  - Converted value in degrees per second as a float.
 ******************************************************************************/
float ConvertToDPS(int16_t axis_data)
{
    float dps = axis_data * sensitivity;             // Multiply raw data by sensitivity to get DPS
    return dps;                                      // Return the DPS value
}

/*******************************************************************************
 * Function: ConvertToVelocity
 * -----------------------------------------------------------------------------
 * Converts gyroscope data in degrees per second to linear velocity.
 *
 * Parameters:
 *  - axis_data: Raw gyroscope data for a single axis.
 *
 * Returns:
 *  - Converted velocity as a float.
 ******************************************************************************/
float ConvertToVelocity(int16_t axis_data)
{
    float velocity = axis_data * sensitivity * DEGREE_TO_RAD * MY_LEG; // Convert DPS to velocity
    return velocity;                                                // Return the velocity value
}

/*******************************************************************************
 * Function: GetDistance
 * -----------------------------------------------------------------------------
 * Calculates the cumulative distance based on an array of raw gyroscope data.
 *
 * Parameters:
 *  - arr: Array of raw gyroscope data samples.
 *
 * Returns:
 *  - Calculated distance as a float.
 ******************************************************************************/
float GetDistance(int16_t arr[])
{
    float distance = 0.00f;                                         // Initialize distance accumulator
    for (int i = 0; i < 400; i++)                                   // Iterate through 400 samples
    {
        float v = ConvertToVelocity(arr[i]);                        // Convert raw data to velocity
        distance += abs(v * 0.05f);                                 // Accumulate the absolute velocity scaled by time
    }
    return distance;                                                // Return the total calculated distance
}

/*******************************************************************************
 * Function: GetCalibratedRawData
 * -----------------------------------------------------------------------------
 * Retrieves calibrated raw data from the gyroscope by applying zero-rate offsets
 * and thresholding minor vibrations.
 *
 * Parameters:
 *  - None
 *
 * Returns:
 *  - None
 ******************************************************************************/
void GetCalibratedRawData()
{
    GetGyroValue(gyro_raw);                                         // Read raw gyroscope data

    // Apply zero-rate level offsets to calibrate data
    gyro_raw->x_raw -= x_sample;                                   // Subtract X-axis zero-rate level
    gyro_raw->y_raw -= y_sample;                                   // Subtract Y-axis zero-rate level
    gyro_raw->z_raw -= z_sample;                                   // Subtract Z-axis zero-rate level

    // Apply thresholding to eliminate minor vibrations
    if (abs(gyro_raw->x_raw) < abs(x_threshold))
        gyro_raw->x_raw = 0;                                       // Zero out X-axis data below threshold
    if (abs(gyro_raw->y_raw) < abs(y_threshold))
        gyro_raw->y_raw = 0;                                       // Zero out Y-axis data below threshold
    if (abs(gyro_raw->z_raw) < abs(z_threshold))
        gyro_raw->z_raw = 0;                                       // Zero out Z-axis data below threshold
}

/*******************************************************************************
 * Function: PowerOff
 * -----------------------------------------------------------------------------
 * Powers off the gyroscope by disabling it through the control register.
 *
 * Parameters:
 *  - None
 *
 * Returns:
 *  - None
 ******************************************************************************/
void PowerOff()
{
    WriteByte(CTRL_REG_1, 0x00);                                   // Disable the gyroscope by writing 0 to CTRL_REG_1
}
