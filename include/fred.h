// Features for FRED
// OLED Display : https://github.com/avsa242/ssd130x-spin
// Relay Control (5 digital outputs)
// Temperature Sensor (3 analog inputs) : https://www.parallax.com/simple-analog-input/
// Potentiometers (2 analog inputs) : https://www.parallax.com/simple-analog-input/
// Width Measurement Sensors (I2C Bus) : https://github.com/drspangle/infidel-sensor
// 

#define OLED_SCK_PIN  1
#define OLED_SDA_PIN  2

#define STEPPER1_DIR_PIN  52
#define STEPPER1_STEP_PIN  54

#define STEPPER2_DIR_PIN  48
#define STEPPER2_STEP_PIN  50

#define STEPPER3_DIR_PIN  44
#define STEPPER3_STEP_PIN  46

#define STEPPER4_DIR_PIN  40
#define STEPPER4_STEP_PIN  42

#define RELAY1_PIN  41
#define RELAY2_PIN  43
#define RELAY3_PIN  45
#define RELAY4_PIN  47

#define TEMP1_PIN  8
#define TEMP2_PIN  9
#define TEMP3_PIN  10

#define POT1_PIN  6
#define POT2_PIN  12

#define WIDTH_SENSOR_SCL_PIN  13
#define WIDTH_SENSOR_SDA_PIN  14
#define WIDTH_SENSOR_Y_ADDR 0x40
#define WIDTH_SENSOR_X_ADDR 0x41
