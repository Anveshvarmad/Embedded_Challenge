/*
Project Team Members:
- Anvesh Varma Dantuluri (ad7647)
- Pavan Kishore Ramavath (pr2622)
- Bhanu Dileep Reddy Maryada (bm3689)
*/


#include <mbed.h>                                // Include the mbed OS header
#include <vector>                                // Include the vector container
#include <array>                                 // Include the array container
#include <limits>                                // Include limits for numeric limits
#include <cmath>                                 // Include cmath for mathematical functions
#include <math.h>                                // Include math.h for additional math functions
#include "gyro.h"                                // Include custom gyroscope header
#include "drivers/LCD_DISCO_F429ZI.h"           // Include LCD driver for DISCO_F429ZI board
#include "drivers/TS_DISCO_F429ZI.h"            // Include Touch Screen driver for DISCO_F429ZI board

// Define event flags using bitmask values
#define KEY_FLAG 1                                // Flag for key recording event
#define UNLOCK_FLAG 2                             // Flag for unlock event
#define ERASE_FLAG 4                              // Flag for erase event
#define DATA_READY_FLAG 8                         // Flag indicating gyroscope data is ready

// Define LCD font size
#define FONT_SIZE 16                              // Font size for LCD text

// Define the correlation threshold for unlocking
#define CORRELATION_THRESHOLD 0.0005f               // Threshold value for gesture correlation

// Initialize interrupt inputs with pull-down resistors
InterruptIn gyro_int2(PA_2, PullDown);            // Interrupt for gyroscope data ready on pin PA_2
InterruptIn user_button(PC_13, PullDown);         // Interrupt for user button on pin PC_13

// Initialize digital outputs for LEDs
DigitalOut green_led(LED1);                        // Green LED indicator
DigitalOut red_led(LED2);                          // Red LED indicator

// Create LCD and Touch Screen objects
LCD_DISCO_F429ZI lcd;                               // LCD display object for DISCO_F429ZI
TS_DISCO_F429ZI ts;                                 // Touch screen object for DISCO_F429ZI

// Initialize event flags and timer
EventFlags flags;                                    // Event flags object for inter-thread communication
Timer timer;                                        // Timer object for measuring elapsed time

/*******************************************************************************
 * Function Prototypes for LCD and Touch Screen Operations
 * ****************************************************************************/
void draw_button(int x, int y, int width, int height, const char *label); // Function to draw a button on the LCD
bool is_touch_inside_button(int touch_x, int touch_y, int button_x, int button_y, int button_width, int button_height); // Function to check if touch is inside a button
void remove_button(int x, int y, int width, int height); // Function to remove a button from the LCD

/*******************************************************************************
 * Function Prototypes for Data Processing
 * ****************************************************************************/
float euclidean_distance(const array<float, 3> &a, const array<float, 3> &b); // Calculate Euclidean distance between two 3D vectors
float dtw(const vector<array<float, 3>> &s, const vector<array<float, 3>> &t); // Calculate Dynamic Time Warping distance between two gesture sequences
void trim_gyro_data(vector<array<float, 3>> &data); // Trim insignificant gyroscope data from a gesture sequence
float correlation(const vector<float> &a, const vector<float> &b); // Calculate Pearson correlation between two vectors
array<float, 3> calculateCorrelationVectors(vector<array<float, 3>>& vec1, vector<array<float, 3>>& vec2); // Calculate correlation for each axis between two gesture sequences

/*******************************************************************************
 * Function Prototypes for Threads
 * ****************************************************************************/
void gyroscope_thread();                            // Thread function for handling gyroscope data and gesture recording
void touch_screen_thread();                         // Thread function for handling touch screen input

/*******************************************************************************
 * Function Prototypes for Flash Memory Operations
 * ****************************************************************************/
bool storeGyroDataToFlash(vector<array<float, 3>> &gesture_key, uint32_t flash_address); // Store gyroscope data to flash memory
vector<array<float, 3>> readGyroDataFromFlash(uint32_t flash_address, size_t data_size); // Read gyroscope data from flash memory

/*******************************************************************************
 * Function Prototypes for Filters
 * ****************************************************************************/
// moving average filter will be defined after main()
float movingAverageFilter(float input, float display_buffer[], size_t N, size_t &index, float &sum); // Apply a moving average filter to input data

/*******************************************************************************
 * ISR Callback Functions
 * ****************************************************************************/
/**
 * @brief Callback function for user button press interrupt
 */
void button_press() 
{
    flags.set(ERASE_FLAG);                          // Set the ERASE_FLAG when button is pressed
}

/**
 * @brief Callback function for gyroscope data ready interrupt
 */
void onGyroDataReady() 
{
    flags.set(DATA_READY_FLAG);                     // Set the DATA_READY_FLAG when gyroscope data is ready
}

/*******************************************************************************
 * @brief Global Variables
 * ****************************************************************************/
vector<array<float, 3>> gesture_key;                // Vector to store the recorded gesture key
vector<array<float, 3>> unlocking_record;           // Vector to store the unlocking gesture record

// Define button positions, sizes, and labels
const int button1_x = 60;                           // X-coordinate for the first button
const int button1_y = 80;                           // Y-coordinate for the first button
const int button1_width = 120;                      // Width of the first button
const int button1_height = 50;                      // Height of the first button
const char *button1_label = "RECORD";               // Label for the first button
const int button2_x = 60;                           // X-coordinate for the second button
const int button2_y = 180;                          // Y-coordinate for the second button
const int button2_width = 120;                      // Width of the second button
const int button2_height = 50;                      // Height of the second button
const char *button2_label = "UNLOCK";               // Label for the second button
const int message_x = 5;                            // X-coordinate for the welcome message
const int message_y = 30;                           // Y-coordinate for the welcome message
const char *message = "GESTURE UNLOCK";             // Welcome message text
const int text_x = 5;                               // X-coordinate for status messages
const int text_y = 270;                             // Y-coordinate for status messages
const char *text_0 = "NO KEY RECORDED";             // Message when no key is recorded
const char *text_1 = "LOCKED";                      // Message when the system is locked
const char *button3 = "RESET ";                     // Label for the reset button

int err = 0;                                        // Variable for error checking

/*******************************************************************************
 * @brief Main Function
 * ****************************************************************************/
int main()
{
    lcd.Clear(LCD_COLOR_ORANGE);                     // Clear the LCD with orange background color

    // Draw the first button labeled "RECORD"
    draw_button(button1_x, button1_y + 50, button1_width, button1_height, button1_label);

    // Optionally draw the second button labeled "UNLOCK" (currently commented out)
    //draw_button(button2_x, button2_y, button2_width, button2_height, button2_label);

    // Display the welcome message at specified coordinates in center mode
    lcd.DisplayStringAt(message_x, message_y, (uint8_t *)message, CENTER_MODE);

    // Initialize interrupt handlers for user button and gyroscope data ready
    user_button.rise(&button_press);                 // Attach button_press callback to rising edge of user_button
    gyro_int2.rise(&onGyroDataReady);                // Attach onGyroDataReady callback to rising edge of gyro_int2

    // Initialize LEDs based on whether a gesture key is already recorded
    if (gesture_key.empty())
    {
        red_led = 0;                                 // Turn off red LED
        green_led = 1;                               // Turn on green LED
        lcd.DisplayStringAt(text_x, text_y, (uint8_t *)text_0, CENTER_MODE); // Display "NO KEY RECORDED" message
    }
    else
    {
        red_led = 1;                                 // Turn on red LED
        green_led = 0;                               // Turn off green LED
        lcd.DisplayStringAt(text_x, text_y, (uint8_t *)text_1, CENTER_MODE); // Display "LOCKED" message
    }

    // Create and start the gyroscope handling thread
    Thread key_saving;                                // Define a thread object for gyroscope handling
    key_saving.start(callback(gyroscope_thread));     // Start the gyroscope_thread

    // Create and start the touch screen handling thread
    Thread touch_thread;                              // Define a thread object for touch screen handling
    touch_thread.start(callback(touch_screen_thread)); // Start the touch_screen_thread

    // Keep the main thread alive indefinitely
    while (1)
    {
        ThisThread::sleep_for(100ms);                 // Sleep for 100 milliseconds
    }
}

/*******************************************************************************
 *
 * @brief Gyroscope Gesture Key Saving Thread
 *
 * This thread handles recording gestures, saving gesture keys, and unlocking by comparing recorded gestures.
 *
 ******************************************************************************/
void gyroscope_thread()
{
    // Define and initialize gyroscope parameters
    Gyroscope_Init_Parameters init_parameters;        // Structure to hold gyroscope initialization parameters
    init_parameters.conf1 = ODR_200_CUTOFF_50;       // Set Output Data Rate to 200Hz and cutoff to 50Hz
    init_parameters.conf3 = INT2_DRDY;               // Configure interrupt 2 for data ready
    init_parameters.conf4 = FULL_SCALE_500;          // Set full scale to ±500 degrees per second

    // Define a structure to hold raw gyroscope data
    Gyroscope_RawData raw_data;                       // Structure to store raw gyroscope data

    // Define a buffer to hold status messages for display on the LCD
    char display_buffer[50];                          // Buffer to store display messages

    // Check if gyroscope data ready flag is not set and the interrupt pin is high
    if (!(flags.get() & DATA_READY_FLAG) && (gyro_int2.read() == 1))
    {
        flags.set(DATA_READY_FLAG);                   // Manually set DATA_READY_FLAG if data is already ready
    }

    // Infinite loop to handle events
    while (1)
    {
        vector<array<float, 3>> temp_key;             // Temporary vector to store recorded gyroscope data

        // Wait for any of the KEY_FLAG, UNLOCK_FLAG, or ERASE_FLAG to be set
        auto flag_check = flags.wait_any(KEY_FLAG | UNLOCK_FLAG | ERASE_FLAG);

        // Handle ERASE_FLAG event
        if (flag_check & ERASE_FLAG)
        {
            // Display "Erasing..." message on the LCD
            sprintf(display_buffer, "Erasing....");
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange (background color)
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
            lcd.SetTextColor(LCD_COLOR_BLACK);                       // Set text color to black
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display message

            gesture_key.clear();                                      // Clear the recorded gesture key

            // Display "Key Erasing finish." message
            sprintf(display_buffer, "Key Erasing finish.");
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
            lcd.SetTextColor(LCD_COLOR_BLACK);                       // Set text color to black
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display message

            unlocking_record.clear();                                 // Clear the unlocking record

            // Reset LEDs and display "All Erasing finish." message
            green_led = 1;                                           // Turn on green LED
            red_led = 0;                                             // Turn off red LED
            sprintf(display_buffer, "All Erasing finish.");
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
            lcd.SetTextColor(LCD_COLOR_BLACK);                       // Set text color to black
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display message
        }

        // Handle KEY_FLAG or UNLOCK_FLAG events
        if (flag_check & (KEY_FLAG | UNLOCK_FLAG))
        {
            // Display "Hold On" message
            sprintf(display_buffer, "Hold On");
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
            lcd.SetTextColor(LCD_COLOR_BLACK);                       // Set text color to black
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display message

            ThisThread::sleep_for(1s);                                // Wait for 1 second

            // Display "Calibrating..." message
            sprintf(display_buffer, "Calibrating...");
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
            lcd.SetTextColor(LCD_COLOR_BLACK);                       // Set text color to black
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display message

            // Initialize gyroscope with the defined parameters
            InitiateGyroscope(&init_parameters, &raw_data);           // Call function to initiate gyroscope

            // Display countdown messages before recording
            sprintf(display_buffer, "Recording in 3...");
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display countdown message
            ThisThread::sleep_for(1s);                                // Wait for 1 second

            sprintf(display_buffer, "Recording in 2...");
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display countdown message
            ThisThread::sleep_for(1s);                                // Wait for 1 second

            sprintf(display_buffer, "Recording in 1...");
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display countdown message
            ThisThread::sleep_for(1s);                                // Wait for 1 second

            // Display "Recording..." message
            sprintf(display_buffer, "Recording...");
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display recording message
            
            // Start recording gyroscope data for 5 seconds
            timer.start();                                            // Start the timer
            while (timer.elapsed_time() < 5s)                         // Loop for 5 seconds
            {
                flags.wait_all(DATA_READY_FLAG);                      // Wait until DATA_READY_FLAG is set
                GetCalibratedRawData();                               // Retrieve calibrated raw gyroscope data
                // Add the converted data to the temporary gesture key vector
                temp_key.push_back({ConvertToDPS(raw_data.x_raw), ConvertToDPS(raw_data.y_raw), ConvertToDPS(raw_data.z_raw)});
                ThisThread::sleep_for(50ms);                           // Wait for 50 milliseconds (20Hz sampling rate)
            }
            timer.stop();                                             // Stop the timer
            timer.reset();                                            // Reset the timer

            // Remove insignificant data from the recorded gesture
            trim_gyro_data(temp_key);                                 // Trim the temporary gesture key data

            // Display "Finished..." message
            sprintf(display_buffer, "Finished...");
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
            lcd.SetTextColor(LCD_COLOR_BLACK);                       // Set text color to black
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display finished message
        }

        // Check if the event was for recording a key or unlocking
        if (flag_check & KEY_FLAG)
        {
            if (gesture_key.empty())                                // If no key is currently recorded
            {
                // Display "Saving Key..." message
                sprintf(display_buffer, "Saving Key...");
                lcd.SetTextColor(LCD_COLOR_ORANGE);                  // Set text color to orange
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); // Clear the specific line on LCD
                lcd.SetTextColor(LCD_COLOR_BLACK);                   // Set text color to black
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display saving message

                gesture_key = temp_key;                              // Save the temporary gesture key as the main gesture key
                temp_key.clear();                                    // Clear the temporary gesture key vector

                // Toggle LEDs to indicate key is saved
                red_led = 1;                                         // Turn on red LED
                green_led = 0;                                       // Turn off green LED

                // Display "Key saved..." message
                sprintf(display_buffer, "Key saved...");
                lcd.SetTextColor(LCD_COLOR_ORANGE);                  // Set text color to orange
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); // Clear the specific line on LCD
                lcd.SetTextColor(LCD_COLOR_BLACK);                   // Set text color to black
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display saved message

                // Update buttons on the LCD
                draw_button(button1_x, button1_y, button1_width, button1_height, button3); // Draw "RESET" button
                remove_button(button1_x, button1_y + 50, button1_width, button1_height); // Remove the "RECORD" button
                draw_button(button2_x, button2_y, button2_width, button2_height, button2_label); // Draw "UNLOCK" button
            }
            else
            {
                // Display "Removing old key..." message
                sprintf(display_buffer, "Removing old key...");
                lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
                lcd.SetTextColor(LCD_COLOR_BLACK);                       // Set text color to black
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display removing message

                ThisThread::sleep_for(1s);                                // Wait for 1 second
                
                gesture_key.clear();                                      // Clear the existing gesture key
                gesture_key = temp_key;                                  // Save the new temporary gesture key as the main gesture key

                // Display "New key is saved." message
                sprintf(display_buffer, "New key is saved.");
                lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
                lcd.SetTextColor(LCD_COLOR_BLACK);                       // Set text color to black
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display saved message

                temp_key.clear();                                        // Clear the temporary gesture key vector

                // Toggle LEDs to indicate new key is saved
                red_led = 1;                                             // Turn on red LED
                green_led = 0;                                           // Turn off green LED
            }
        }
        else if (flag_check & UNLOCK_FLAG)                           // If the event was for unlocking
        {
            flags.clear(UNLOCK_FLAG);                                 // Clear the UNLOCK_FLAG
            // Display "Unlocking..." message
            sprintf(display_buffer, "Unlocking...");
            lcd.SetTextColor(LCD_COLOR_ORANGE);                      // Set text color to orange
            lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE);       // Clear the specific line on LCD
            lcd.SetTextColor(LCD_COLOR_BLACK);                       // Set text color to black
            lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display unlocking message

            unlocking_record = temp_key;                               // Save the temporary gesture key as the unlocking record
            temp_key.clear();                                         // Clear the temporary gesture key vector

            if (gesture_key.empty())                                  // If no gesture key is recorded
            {
                // Display "NO KEY SAVED." message
                sprintf(display_buffer, "NO KEY SAVED.");
                lcd.SetTextColor(LCD_COLOR_ORANGE);                  // Set text color to orange
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); // Clear the specific line on LCD
                lcd.SetTextColor(LCD_COLOR_BLACK);                   // Set text color to black
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display no key message

                unlocking_record.clear();                             // Clear the unlocking record

                // Toggle LEDs to indicate no key is saved
                green_led = 1;                                       // Turn on green LED
                red_led = 0;                                         // Turn off red LED
            }
            else // If a gesture key is recorded, compare it with the unlocking record
            {
                int unlock = 0;                                        // Counter for correlated axes

                // Calculate correlation for each axis between gesture_key and unlocking_record
                array<float, 3> correlationResult = calculateCorrelationVectors(gesture_key, unlocking_record); // Get correlation results

                if (err != 0)                                         // Check for correlation calculation errors
                {
                    printf("Error calculating correlation: vectors have different sizes\n"); // Print error message
                }
                else
                {
                    // Print correlation values for each axis
                    printf("Correlation values: x = %f, y = %f, z = %f\n", correlationResult[0], correlationResult[1], correlationResult[2]);
                    
                    // Iterate through correlation results to check against threshold
                    for (size_t i = 0; i < correlationResult.size(); i++)
                    {
                        if (correlationResult[i] > CORRELATION_THRESHOLD) // If correlation exceeds threshold
                        {
                            unlock++;                                   // Increment unlock counter
                        }
                    }
                }

                if (unlock == 3)                                      // If all three axes exceed threshold
                {
                    // Display "UNLOCK: SUCCESS" message
                    sprintf(display_buffer, "UNLOCK: SUCCESS");
                    lcd.SetTextColor(LCD_COLOR_GREEN);               // Set text color to green
                    lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); // Clear the specific line on LCD
                    lcd.SetTextColor(LCD_COLOR_BLACK);               // Set text color to black
                    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display success message
                    
                    // Toggle LEDs to indicate successful unlock
                    green_led = 1;                                   // Turn on green LED
                    red_led = 0;                                     // Turn off red LED

                    unlocking_record.clear();                        // Clear the unlocking record
                    unlock = 0;                                      // Reset unlock counter
                }
                else
                {
                    // Display "UNLOCK: FAILED" message
                    sprintf(display_buffer, "UNLOCK: FAILED");
                    lcd.SetTextColor(LCD_COLOR_RED);                 // Set text color to red
                    lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); // Clear the specific line on LCD
                    lcd.SetTextColor(LCD_COLOR_BLACK);               // Set text color to black
                    lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display failure message

                    // Toggle LEDs to indicate failed unlock
                    green_led = 0;                                   // Turn off green LED
                    red_led = 1;                                     // Turn on red LED

                    unlocking_record.clear();                        // Clear the unlocking record
                    unlock = 0;                                      // Reset unlock counter
                }
            }
        }
        ThisThread::sleep_for(100ms);                                 // Sleep for 100 milliseconds before next iteration
    }
}

/*******************************************************************************
 *
 * @brief Touch Screen Thread
 *
 * This thread handles touch screen input, determining which button was pressed and setting corresponding flags.
 *
 ******************************************************************************/
void touch_screen_thread()
{
    // Define a structure to hold touch screen state
    TS_StateTypeDef ts_state;                                      // Structure to store touch screen state

    // Initialize the touch screen with the LCD's dimensions
    if (ts.Init(lcd.GetXSize(), lcd.GetYSize()) != TS_OK)          // Initialize touch screen and check for success
    {
        printf("Failed to initialize the touch screen!\r\n");      // Print error message if initialization fails
        return;                                                    // Exit the thread if initialization fails
    }

    // Define a buffer to hold display messages
    char display_buffer[50];                                      // Buffer to store display messages

    // Infinite loop to handle touch inputs
    while (1)
    {
        ts.GetState(&ts_state);                                   // Get the current state of the touch screen
        if (ts_state.TouchDetected)                               // Check if a touch is detected
        {
            int touch_x = ts_state.X;                             // Get the X-coordinate of the touch
            int touch_y = ts_state.Y;                             // Get the Y-coordinate of the touch

            // Check if the touch is inside the "RECORD" button area (adjusted Y-coordinate)
            if (is_touch_inside_button(touch_x, touch_y + 50, button2_x, button2_y, button1_width, button1_height))
            {
                // Display "Recording Initiated..." message
                sprintf(display_buffer, "Recording Initiated...");
                lcd.SetTextColor(LCD_COLOR_LIGHTBLUE);            // Set text color to light blue
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); // Clear the specific line on LCD
                lcd.SetTextColor(LCD_COLOR_DARKRED);              // Set text color to dark red
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display initiation message
                ThisThread::sleep_for(1s);                         // Wait for 1 second
                flags.set(KEY_FLAG);                               // Set KEY_FLAG to initiate recording
            }

            // Check if the touch is inside the "RESET" button area
            if (is_touch_inside_button(touch_x, touch_y, button2_x, button2_y, button1_width, button1_height))
            {
                // Display "Resetting Key Initiated" message
                sprintf(display_buffer, "Resetting Key Initiated");
                lcd.SetTextColor(LCD_COLOR_LIGHTBLUE);            // Set text color to light blue
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); // Clear the specific line on LCD
                lcd.SetTextColor(LCD_COLOR_DARKRED);              // Set text color to dark red
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display reset message
                ThisThread::sleep_for(1s);                         // Wait for 1 second
                flags.set(KEY_FLAG);                               // Set KEY_FLAG to initiate key reset
            }

            // Check if the touch is inside the "UNLOCK" button area
            if (is_touch_inside_button(touch_x, touch_y, button1_x, button1_y, button2_width, button2_height))
            {
                // Display "Unlocking Initiated..." message
                sprintf(display_buffer, "Unlocking Initiated...");
                lcd.SetTextColor(LCD_COLOR_LIGHTBLUE);            // Set text color to light blue
                lcd.FillRect(0, text_y, lcd.GetXSize(), FONT_SIZE); // Clear the specific line on LCD
                lcd.SetTextColor(LCD_COLOR_DARKGREEN);             // Set text color to dark green
                lcd.DisplayStringAt(text_x, text_y, (uint8_t *)display_buffer, CENTER_MODE); // Display unlocking message
                ThisThread::sleep_for(1s);                         // Wait for 1 second
                flags.set(UNLOCK_FLAG);                            // Set UNLOCK_FLAG to initiate unlocking
            }
        }
        ThisThread::sleep_for(10ms);                                // Sleep for 10 milliseconds before next touch check
    }
}

/*******************************************************************************
 *
 * @brief Store Gyroscope Data to Flash Memory
 * @param gesture_key: The gesture data to store
 * @param flash_address: The starting address in flash memory to store the data
 * @return true if data is stored successfully, false otherwise
 *
 ******************************************************************************/
bool storeGyroDataToFlash(vector<array<float, 3>> &gesture_key, uint32_t flash_address)
{
    FlashIAP flash;                                               // Create a FlashIAP object for flash memory operations
    flash.init();                                                // Initialize the flash interface

    // Calculate the total size of the data to be stored in bytes
    uint32_t data_size = gesture_key.size() * sizeof(array<float, 3>); // Total size in bytes

    // Erase the flash sector where data will be stored
    flash.erase(flash_address, data_size);                      // Erase flash memory at specified address

    // Write the gesture data to flash memory
    int write_result = flash.program(gesture_key.data(), flash_address, data_size); // Program flash with gesture data

    flash.deinit();                                              // Deinitialize the flash interface

    return write_result == 0;                                    // Return true if programming was successful
}

/*******************************************************************************
 *
 * @brief Read Gyroscope Data from Flash Memory
 * @param flash_address: The starting address in flash memory to read from
 * @param data_size: The number of data elements to read
 * @return A vector containing the read gyroscope data
 *
 ******************************************************************************/
vector<array<float, 3>> readGyroDataFromFlash(uint32_t flash_address, size_t data_size)
{
    vector<array<float, 3>> gesture_key(data_size);               // Initialize a vector to hold the read data

    FlashIAP flash;                                               // Create a FlashIAP object for flash memory operations
    flash.init();                                                // Initialize the flash interface

    // Read the gesture data from flash memory
    flash.read(gesture_key.data(), flash_address, data_size * sizeof(array<float, 3>)); // Read data from flash

    flash.deinit();                                              // Deinitialize the flash interface

    return gesture_key;                                         // Return the read gesture data
}

/*******************************************************************************
 *
 * @brief Draw a Button on the LCD
 * @param x: X-coordinate of the button's top-left corner
 * @param y: Y-coordinate of the button's top-left corner
 * @param width: Width of the button
 * @param height: Height of the button
 * @param label: Text label to display on the button
 *
 ******************************************************************************/
void draw_button(int x, int y, int width, int height, const char *label)
{
    lcd.SetTextColor(LCD_COLOR_BLACK);                           // Set text color to black
    lcd.FillRect(x, y, width, height);                           // Draw a filled rectangle for the button
    // Display the button label centered within the button
    lcd.DisplayStringAt(x + width / 2 - strlen(label) * 19, y + height / 2 - 8, (uint8_t *)label, CENTER_MODE);
}

/*******************************************************************************
 *
 * @brief Remove a Button from the LCD
 * @param x: X-coordinate of the button's top-left corner
 * @param y: Y-coordinate of the button's top-left corner
 * @param width: Width of the button
 * @param height: Height of the button
 *
 ******************************************************************************/
void remove_button(int x, int y, int width, int height)
{
    // Set the color to the background color (e.g., orange)
    lcd.SetTextColor(LCD_COLOR_ORANGE);                           // Set text color to orange (background color)
    lcd.FillRect(x, y, width, height);                            // Fill the button area with background color to remove it
    lcd.SetTextColor(LCD_COLOR_BLACK);                            // Reset text color to black
}

/*******************************************************************************
 *
 * @brief Check if a Touch Point is Inside a Button Area
 * @param touch_x: X-coordinate of the touch point
 * @param touch_y: Y-coordinate of the touch point
 * @param button_x: X-coordinate of the button's top-left corner
 * @param button_y: Y-coordinate of the button's top-left corner
 * @param button_width: Width of the button
 * @param button_height: Height of the button
 * @return true if the touch is inside the button area, false otherwise
 *
 ******************************************************************************/
bool is_touch_inside_button(int touch_x, int touch_y, int button_x, int button_y, int button_width, int button_height)
{
    // Check if touch coordinates are within the button's boundaries
    return (touch_x >= button_x && touch_x <= button_x + button_width &&
            touch_y >= button_y && touch_y <= button_y + button_height);
}

/*******************************************************************************
 *
 * @brief Calculate the Euclidean Distance Between Two 3D Vectors
 * @param a: The first 3D vector
 * @param b: The second 3D vector
 * @return The Euclidean distance between vectors a and b
 *
 ******************************************************************************/
float euclidean_distance(const array<float, 3> &a, const array<float, 3> &b)
{
    float sum = 0;                                                // Initialize sum of squared differences
    for (size_t i = 0; i < 3; ++i)                               // Iterate over each axis
    {
        sum += (a[i] - b[i]) * (a[i] - b[i]);                    // Accumulate squared differences
    }
    return sqrt(sum);                                            // Return the square root of the sum (Euclidean distance)
}

/*******************************************************************************
 *
 * @brief Calculate the Dynamic Time Warping (DTW) Distance Between Two Gesture Sequences
 * @param s: The first gesture sequence
 * @param t: The second gesture sequence
 * @return The DTW distance between sequences s and t
 *
 ******************************************************************************/
float dtw(const vector<array<float, 3>> &s, const vector<array<float, 3>> &t)
{
    // Initialize DTW matrix with infinities
    vector<vector<float>> dtw_matrix(s.size() + 1, vector<float>(t.size() + 1, numeric_limits<float>::infinity()));

    dtw_matrix[0][0] = 0;                                        // Set the starting point to zero

    for (size_t i = 1; i <= s.size(); ++i)                      // Iterate over the first sequence
    {
        for (size_t j = 1; j <= t.size(); ++j)                  // Iterate over the second sequence
        {
            float cost = euclidean_distance(s[i - 1], t[j - 1]); // Calculate cost between current elements
            // Update DTW matrix with the minimum cost path
            dtw_matrix[i][j] = cost + min({dtw_matrix[i - 1][j], dtw_matrix[i][j - 1], dtw_matrix[i - 1][j - 1]});
        }
    }

    return dtw_matrix[s.size()][t.size()];                       // Return the final DTW distance
}

/*******************************************************************************
 *
 * @brief Trim Insignificant Gyroscope Data from a Gesture Sequence
 * @param data: The gesture data to trim
 *
 ******************************************************************************/
void trim_gyro_data(vector<array<float, 3>> &data)
{
    float threshold = 1e-8f;                                     // Define a small threshold to identify insignificant data
    auto ptr = data.begin();                                     // Iterator to traverse the data

    // Find the first element where any axis exceeds the threshold
    while (ptr != data.end() && 
           abs((*ptr)[0]) <= threshold && 
           abs((*ptr)[1]) <= threshold && 
           abs((*ptr)[2]) <= threshold)
    {
        ptr++;                                                    // Move to the next element
    }

    if (ptr == data.end())                                       // If all data points are below threshold
        return;                                                  // No trimming needed

    auto lptr = ptr;                                             // Left boundary pointer

    // Start searching from the end to find the last significant data point
    ptr = data.end() - 1;                                        // Set pointer to the last element
    while (abs((*ptr)[0]) <= threshold && 
           abs((*ptr)[1]) <= threshold && 
           abs((*ptr)[2]) <= threshold && 
           ptr != data.begin())
    {
        ptr--;                                                    // Move to the previous element
    }

    auto rptr = ptr;                                             // Right boundary pointer

    // Move significant data to the front of the vector
    auto replace_ptr = data.begin();                             // Pointer for replacement
    for (; replace_ptr != lptr && lptr <= rptr; replace_ptr++, lptr++)
    {
        *replace_ptr = *lptr;                                    // Replace data with significant data
    }

    // Trim the end of the vector based on the boundaries
    if (lptr > rptr)
    {
        data.erase(replace_ptr, data.end());                     // Erase from replacement pointer to the end
    }
    else
    {
        data.erase(rptr + 1, data.end());                        // Erase from the last significant point to the end
    }
}

/*******************************************************************************
 *
 * @brief Calculate the Pearson Correlation Between Two Vectors
 * @param a: The first vector
 * @param b: The second vector
 * @return The Pearson correlation coefficient between vectors a and b
 *
 ******************************************************************************/
float correlation(const vector<float> &a, const vector<float> &b)
{
    // Check if both vectors are of the same size
    if (a.size() != b.size())
    {
        err = -1;                                                // Set error flag if sizes differ
        return 0.0f;                                             // Return zero correlation
    }

    float sum_a = 0, sum_b = 0, sum_ab = 0, sq_sum_a = 0, sq_sum_b = 0; // Initialize sums

    for (size_t i = 0; i < a.size(); ++i)                      // Iterate over each element
    {
        sum_a += a[i];                                          // Sum of elements in vector a
        sum_b += b[i];                                          // Sum of elements in vector b
        sum_ab += a[i] * b[i];                                  // Sum of element-wise products
        sq_sum_a += a[i] * a[i];                                // Sum of squares of vector a
        sq_sum_b += b[i] * b[i];                                // Sum of squares of vector b
    }

    size_t n = a.size();                                        // Number of elements

    float numerator = n * sum_ab - sum_a * sum_b;               // Calculate covariance

    float denominator = sqrt((n * sq_sum_a - sum_a * sum_a) * 
                             (n * sq_sum_b - sum_b * sum_b));  // Calculate product of standard deviations

    return numerator / denominator;                             // Return Pearson correlation coefficient
}

/*******************************************************************************
 *
 * @brief Calculate Pearson Correlation for Each Axis Between Two Gesture Sequences
 * @param vec1: The first gesture sequence
 * @param vec2: The second gesture sequence
 * @return An array containing correlation coefficients for x, y, and z axes
 *
 ******************************************************************************/
array<float, 3> calculateCorrelationVectors(vector<array<float, 3>>& vec1, vector<array<float, 3>>& vec2) 
{
    array<float, 3> result;                                     // Array to store correlation results for each axis

    // Calculate correlation for each of the three axes
    for (int i = 0; i < 3; i++) 
    {
        vector<float> a;                                        // Vector to hold axis i data from vec1
        vector<float> b;                                        // Vector to hold axis i data from vec2

        // Populate 'a' with the i-th coordinate from vec1
        for (const auto& arr : vec1) 
        {
            a.push_back(arr[i]);                                 // Add i-th coordinate to vector a
        }
        // Populate 'b' with the i-th coordinate from vec2
        for (const auto& arr : vec2) 
        {
            b.push_back(arr[i]);                                 // Add i-th coordinate to vector b
        }

        // Resize vectors to match sizes by padding with zeros if necessary
        if (a.size() > b.size()) 
        {
            a.resize(b.size(), 0);                               // Resize a to match b's size, pad with zeros
        } 
        else if (b.size() > a.size()) 
        {
            b.resize(a.size(), 0);                               // Resize b to match a's size, pad with zeros
        }

        // Calculate Pearson correlation for the current axis and store in result
        result[i] = correlation(a, b);                          // Store correlation coefficient for axis i
    }

    return result;                                              // Return the array of correlation coefficients
}
