/*
 * parameter_setting.h
 *
 *  Created on: Sep 17, 2023
 *      Author: tongx
 */

#ifndef PARAMETER_SETTING_H_
#define PARAMETER_SETTING_H_

#ifdef __cplusplus
extern "C" {
#endif

/* RF transmission */
#define PACKET_SIZE 12
#define NUM_SAMPLES 1
#define PAYLOAD_LENGTH 20

/* Sleep */
#define STANDBY_DURATION_SECOND 5

/* Bit to read */
#define READ_BIT                         0x8000
#define TEMP_BIT                         0x0004
#define XL_BIT                           0x0001
#define G_BIT                            0x0002
#define ACTIVITY_BIT                     0x0010

/* Configuration of XL, G and interrupt */
#define CTRL1_XL_VALUE_PowerDown         0x0000 //XL power down
#define CTRL1_XL_VALUE_125Hz_4g          0x0018 //ODR_XL = 12.5 Hz, FS_XL = ±4 g
#define CTRL1_XL_VALUE_26Hz_2g           0x0020 //ODR_XL = 26 Hz, FS_XL = ±2 g
#define CTRL1_XL_VALUE_26Hz_4g           0x0028 //ODR_XL = 52 Hz, FS_XL = ±4 g
#define CTRL1_XL_VALUE_52Hz_2g           0x0030 //ODR_XL = 52 Hz, FS_XL = ±2 g
#define CTRL1_XL_VALUE_52Hz_4g           0x0038 //ODR_XL = 52 Hz, FS_XL = ±4 g
#define CTRL1_XL_VALUE_104Hz_2g          0x0040 //ODR_XL = 104 Hz, FS_XL = ±2 g
#define CTRL1_XL_VALUE_104Hz_4g          0x0048 //ODR_XL = 104 Hz, FS_XL = ±4 g
#define CTRL1_XL_VALUE_208Hz_2g          0x0050 //ODR_XL = 208 Hz, FS_XL = ±2 g
#define CTRL1_XL_VALUE_208Hz_4g          0x0058 //ODR_XL = 208 Hz, FS_XL = ±4 g
#define CTRL1_XL_VALUE_3kHz_4g           0x0098 //ODR_XL = 208 Hz, FS_XL = ±4 g
#define CTRL2_G_VALUE_PowerDown          0x0000 //Gyro power down
#define CTRL2_G_VALUE_125Hz_1000         0x0018 //ODR_G = 26 Hz, FS_G = ±500 dps
#define CTRL2_G_VALUE_26Hz_500           0x0024 //ODR_G = 26 Hz, FS_G = ±500 dps
#define CTRL2_G_VALUE_26Hz_1000          0x0028 //ODR_G = 26 Hz, FS_G = ±1000 dps
#define CTRL2_G_VALUE_52Hz_500           0x0034 //ODR_G = 52 Hz, FS_G = ±500 dps
#define CTRL2_G_VALUE_52Hz_1000          0x0038 //ODR_G = 52 Hz, FS_G = ±1000 dps
#define CTRL2_G_VALUE_104Hz_500          0x0044 //ODR_G = 104 Hz, FS_G = ±500 dps
#define CTRL2_G_VALUE_104Hz_1000         0x0048 //ODR_G = 104 Hz, FS_G = ±1000 dps
#define CTRL2_G_VALUE_208Hz_500          0x0054 //ODR_G = 208 Hz, FS_G = ±500 dps
#define CTRL2_G_VALUE_208Hz_1000         0x0058 //ODR_G = 208 Hz, FS_G = ±1000 dps
#define CTRL2_G_VALUE_3kHz_1000          0x0098 //ODR_G = 3.33k Hz, FS_G = ±1000 dps
#define WAKE_UP_DUR                      0x0062 //last 4 bits: duration for inactivity detection - LSB*512/ODR_XL
#define WAKE_UP_THS                      0x0002 // Threshold for wakeup (last 5 bits)
#define TAP_CFG0_VALUE                   0x0000 // Select sleep-change notification; 0x0020 reporting
#define TAP_CFG2_VALUE                   0x00E0 // Enable interrupt; set XL ODR to 12.5 Hz in LP, gyro to power down
#define MD1_CFG_VALUE                    0x0080 // Routing activity/inactivity event on INT1 enabled
#define MD2_CFG_VALUE                    0x0080
#define INT1_CTRL_VALUE                  0x0000

/* XL and G Value */
const float XL_SCALE_RANGE_2_G = 0.061;
const float XL_SCALE_RANGE_4_G = 0.122;
const float XL_SCALE_RANGE_8_G = 0.244;
const float XL_SCALE_RANGE_16_G = 0.488;

const float G_SCALE_RANGE_250_DPS = 8.75;
const float G_SCALE_RANGE_500_DPS = 17.5;
const float G_SCALE_RANGE_1000_DPS = 35.0;
const float G_SCALE_RANGE_2000_DPS = 70.0;

uint8_t buffer_XL[6];
uint8_t buffer_G[6];

uint8_t dummy_address;

uint8_t test_buffer_SPI[2] = {0x00, 0x01};
uint8_t test_buffer[2] = {0x09, 0x25};
uint8_t test_startup_buffer[1] = {0x28};
uint8_t test_buffer_configure[2] = {0x00, 0x11};

uint8_t GPIO_INT_ACTIVE = 0x00;

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_SETTING_H_ */
