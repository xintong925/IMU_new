/*!
 *  @file Adafruit_LSM6DS.h
 *
 *  I2C Driver base for Adafruit LSM6DSxx 6-DoF Accelerometer and Gyroscope
 *      library
 *
 *  Adafruit invests time and resources providing this open source code,
 *      please support Adafruit and open-source hardware by purchasing products
 *from Adafruit!
 *
 *  BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LSM6DSOX_H
#define _ADAFRUIT_LSM6DSOX_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#define LSM6DSOX_I2CADDR_DEFAULT           0x6A           ///< LSM6DOX default i3c address
#define LSM6DSOX_CHIP_ID                   0x6C           ///< LSM6DSOX default device id from WHOAMI
#define LSM6DSOX_FUNC_CFG_ACCESS           0x0100         ///< Enable embedded functions register
#define LSM6DSOX_PIN_CTRL                  0x0200         ///< SDO, OCS_AUX, SDO_AUX pins pull-up enable/disable register
#define LSM6DSOX_MLC_INT1                  0x0D00        ///< Interrupt control for INT 1 -  each bit enables a signal to be carried through INT1
#define LSM6DSOX_INT1_CTRL                 0x0D00        ///< Enables Gyro & Accel data-ready interrupt on INT1 pin
#define LSM6DSOX_INT2_CTRL                 0x0E00        ///< Gyroscope and Accelerometer data-ready
#define LSM6DSOX_EMB_FUNC_INT2             0x0E00        ///< Interrupt control for INT 2
#define LSM6DSOX_WHOAMI                    0x0F00        ///< Chip ID register Address
#define LSM6DSOX_CTRL1_XL                  0x1000        ///< Main accelerometer config register
#define LSM6DSOX_CTRL2_G                   0x1100        ///< Main gyro config register
#define LSM6DSOX_CTRL3_C                   0x1200        ///< Main configuration register
#define LSM6DSOX_CTRL4_C                   0x1300        ///< Gyroscope sleep mode; I2C disable
#define LSM6DSOX_CTRL5_C                   0x1400        ///< Accelerometer ultra-low mode enable; self test
#define LSM6DSOX_CTRL6_C                   0x1500        ///< High-performance operating mode of accelerometer; gyro low-pass filter bw selection
#define LSM6DSOX_CTRL7_G                   0x1600        ///< High-performance operating mode of gyro; gyro digital HP filter cutoff selection
#define LSM6DSOX_CTRL8_XL                  0x1700        ///< High and low pass for accel
#define LSM6DSOX_CTRL9_XL                  0x1800        ///< DEN
#define LSM6DSOX_CTRL10_C                  0x1900        ///< Main configuration register
#define LSM6DSOX_ALL_INT_SRC               0x1A00        ///< Interrupts status
#define LSM6DSOX_WAKE_UP_SRC               0x1B00        ///< Why we woke up
#define LSM6DSOX_TAP_SRC                   0x1C00        ///< Tap source register
#define LSM6DSOX_STATUS_REG                0X1E00        ///< Status register
#define LSM6DSOX_OUT_TEMP_L                0x2000        ///< First data register (temperature low)
#define LSM6DSOX_OUT_TEMP_H                0x2100        ///< Second data register (temperature high)
#define LSM6DSOX_OUTX_L_G                  0x2200        ///< First angular rate pitch axis data register
#define LSM6DSOX_OUTX_H_G                  0x2300        ///< Second angular rate pitch axis data register
#define LSM6DSOX_OUTY_L_G                  0x2400        ///< First angular rate roll axis data register
#define LSM6DSOX_OUTY_H_G                  0x2500        ///< Second angular rate roll axis data register
#define LSM6DSOX_OUTZ_L_G                  0x2600        ///< First angular rate yaw axis data register
#define LSM6DSOX_OUTZ_H_G                  0x2700        ///< Second angular rate yaw axis data register
#define LSM6DSOX_OUTX_L_A                  0x2800        ///< First linear acceleration x-axis data register
#define LSM6DSOX_OUTX_H_A                  0x2900        ///< Second linear acceleration x-axis data register
#define LSM6DSOX_OUTY_L_A                  0x2A00        ///< First linear acceleration y-axis data register
#define LSM6DSOX_OUTY_H_A                  0x2B00        ///< Second linear acceleration y-axis data register
#define LSM6DSOX_OUTZ_L_A                  0x2C00        ///< First linear acceleration z-axis data register
#define LSM6DSOX_OUTZ_H_A                  0x2D00        ///< Second linear acceleration z-axis data register
#define LSM6DSOX_EMB_FUNC_STATUS_MAINPAGE  0x3500        ///< Interrupt status bit for different movement detection
#define LSM6DSOX_STEPCOUNTER               0x4B00        ///< 16-bit step counter
#define LSM6DSOX_TAP_CFG0                  0x5600        ///< Activity/inactivity functions, filtering, tap recognition
#define LSM6DSOX_TAP_CFG1                  0X5700        ///< X-axis threshold and axes priority
#define LSM6DSOX_TAP_CFG2                  0x5800        ///< Enable basic interrupts, activity/inactivity (sleep) function. Tap/pedometer configuration
#define LSM6DSOX_TAP_THS_6D                0x5900        ///< Portrait/landscape position and tap function threshold register
#define LSM6DSOX_INT_DUR2                  0x5A00        ///< Expected quiet time and duration of overthreshold event
#define LSM6DSOX_WAKE_UP_THS               0x5B00        ///< Single and double-tap function threshold register
#define LSM6DSOX_WAKE_UP_DUR               0x5C00        ///< Free-fall, wakeup, timestamp and sleep mode duration
#define LSM6DSOX_MD1_CFG                   0x5E00        ///< Functions routing on INT1 register
#define LSM6DSOX_MD2_CFG                   0x5F00        ///< Functions routing on INT2 register
#define LSM6DSOX_UI_CTRL1_OIS              0x7000        ///< OSI configuration registor - enable mode 4


/** The acce ODR */
typedef enum data_rate_accel {
  LSM6DSOX_ACCEL_RATE_SHUTDOWN = 0b0000,
  LSM6DSOX_ACCEL_RATE_1_6_HZ = 0b1011,
  LSM6DSOX_ACCEL_RATE_12_5_HZ = 0b0001,
  LSM6DSOX_ACCEL_RATE_26_HZ = 0b0010,
  LSM6DSOX_ACCEL_RATE_52_HZ = 0b0011,
  LSM6DSOX_ACCEL_RATE_104_HZ = 0b0100,
  LSM6DSOX_ACCEL_RATE_208_HZ = 0b0101,
  LSM6DSOX_ACCEL_RATE_416_HZ = 0b0110,
  LSM6DSOX_ACCEL_RATE_833_HZ = 0b0111,
  LSM6DSOX_ACCEL_RATE_1_66K_HZ = 0b1000,
  LSM6DSOX_ACCEL_RATE_3_33K_HZ = 0b1001,
  LSM6DSOX_ACCEL_RATE_6_66K_HZ = 0b1010,
} lsm6dsox_data_rate_accel_t;

/** The accelerometer data range */
typedef enum accel_range {
  LSM6DSOX_ACCEL_RANGE_2_G = 0b00,
  LSM6DSOX_ACCEL_RANGE_16_G = 0b01,
  LSM6DSOX_ACCEL_RANGE_4_G = 0b10,
  LSM6DSOX_ACCEL_RANGE_8_G = 0b11
} lsm6dsox_accel_range_t;

/** The gyro ODR */
typedef enum data_rate_gyro {
  LSM6DSOX_GYRO_RATE_SHUTDOWN = 0b0000,
  LSM6DSOX_GYRO_RATE_12_5_HZ = 0b0001,
  LSM6DSOX_GYRO_RATE_26_HZ = 0b0010,
  LSM6DSOX_GYRO_RATE_52_HZ = 0b0011,
  LSM6DSOX_GYRO_RATE_104_HZ = 0b0100,
  LSM6DSOX_GYRO_RATE_208_HZ = 0b0101,
  LSM6DSOX_GYRO_RATE_416_HZ = 0b0110,
  LSM6DSOX_GYRO_RATE_833_HZ = 0b0111,
  LSM6DSOX_GYRO_RATE_1_66K_HZ = 0b1000,
  LSM6DSOX_GYRO_RATE_3_33K_HZ = 0b1001,
  LSM6DSOX_GYRO_RATE_6_66K_HZ = 0b1010,
} lsm6dsox_data_rate_gyro_t;

/** The gyro data range */
typedef enum gyro_range {
  LSM6DSOX_GYRO_RANGE_125_DPS = 0b0010,
  LSM6DSOX_GYRO_RANGE_250_DPS = 0b0000,
  LSM6DSOX_GYRO_RANGE_500_DPS = 0b0100,
  LSM6DSOX_GYRO_RANGE_1000_DPS = 0b1000,
  LSM6DSOX_GYRO_RANGE_2000_DPS = 0b1100,
  LSM6DSOX_GYRO_RANGE_4000_DPS = 0b0001
} lsm6dsox_gyro_range_t;

/** The high pass filter bandwidth */
typedef enum hpf_range {
  LSM6DSOX_HPF_ODR_DIV_50 = 0,
  LSM6DSOX_HPF_ODR_DIV_100 = 1,
  LSM6DSOX_HPF_ODR_DIV_9 = 2,
  LSM6DSOX_HPF_ODR_DIV_400 = 3,
} lsm6dsox_hp_filter_t;

typedef struct {
    // Placeholder structure for Adafruit_SPIDevice
    int8_t _cs;
    int8_t _sck;
    int8_t _miso;
    int8_t _mosi;
    uint32_t _freq;
    uint8_t _dataOrder;
    uint8_t _dataMode;
} Adafruit_SPIDevice;

typedef enum {
    ADDRBIT8_HIGH_TOREAD = 0,
    AD8_HIGH_TOREAD_AD7_HIGH_TOINC = 1,
    ADDRBIT8_HIGH_TOWRITE = 2,
    ADDRESSED_OPCODE_BIT0_LOW_TO_WRITE = 3
} Adafruit_BusIO_SPIRegType;

//typedef enum _BitOrder {
 // SPI_BITORDER_MSBFIRST = SPI_MSBFIRST,
 // SPI_BITORDER_LSBFIRST = SPI_LSBFIRST,
//} BusIOBitOrder;

typedef struct {
    // Placeholder structure for Adafruit_BusIO_Register
    Adafruit_SPIDevice *_spidevice;
    Adafruit_BusIO_SPIRegType _type;
    uint16_t _reg_addr;
} Adafruit_BusIO_Register;

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t); // a pointer to the function;  Function pointers are used to store the address of a function, allowing it to be invoked indirectly
typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
typedef void (*stmdev_mdelay_ptr)(uint32_t millisec);

//typedef struct
//{
  /** Component mandatory fields **/
  //stmdev_write_ptr  write_reg; //  store the address of a function responsible for writing data.

  /** Component optional fields **/
  //stmdev_mdelay_ptr   mdelay; // used to store the address of a function responsible for delaying execution for a specified duration.
  /** Customizable optional pointer **/
  //void *handle;
//} stmdev_ctx_t;

//int32_t lsm6dsox_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);
//int32_t lsm6dsox_read_reg(stmdev_ctx_t *ctx, uint8_t reg,
 //                         uint8_t *data,
  //                        uint16_t len);


#ifdef __cplusplus
}
#endif

#endif /*LSM6DSOX_DRIVER_H */
