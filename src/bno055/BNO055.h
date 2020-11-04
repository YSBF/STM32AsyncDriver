//
// Created by ysbf on 6/29/20.
//

#ifndef BNO055DRIVER_SRC_BNO055_H
#define BNO055DRIVER_SRC_BNO055_H

#include <cstdint>
#include <cstring>
#include "functional"
#include "sensor.h"


/** BNO055 Address A **/
#define BNO055_ADDRESS_A (0x28)
/** BNO055 Address B **/
#define BNO055_ADDRESS_B (0x29)
/** BNO055 ID **/
#define BNO055_ID (0xA0)

/** Offsets registers **/
#define NUM_BNO055_OFFSET_REGISTERS (22)
#define begin_transmission
#define end_transmission

/** A structure to represent offsets **/
typedef struct {
  int16_t accel_offset_x; /**< x acceleration offset */
  int16_t accel_offset_y; /**< y acceleration offset */
  int16_t accel_offset_z; /**< z acceleration offset */

  int16_t mag_offset_x; /**< x magnetometer offset */
  int16_t mag_offset_y; /**< y magnetometer offset */
  int16_t mag_offset_z; /**< z magnetometer offset */

  int16_t gyro_offset_x; /**< x gyroscrope offset */
  int16_t gyro_offset_y; /**< y gyroscrope offset */
  int16_t gyro_offset_z; /**< z gyroscrope offset */

  int16_t accel_radius; /**< acceleration radius */

  int16_t mag_radius; /**< magnetometer radius */
} bno055_offsets_t;

class BNO055 {
public:
  /** BNO055 Registers **/
  typedef enum {
    /* Page id register definition */
    BNO055_PAGE_ID_ADDR = 0X07,

    /* PAGE0 REGISTER DEFINITION START*/
    BNO055_CHIP_ID_ADDR = 0x00,
    BNO055_ACCEL_REV_ID_ADDR = 0x01,
    BNO055_MAG_REV_ID_ADDR = 0x02,
    BNO055_GYRO_REV_ID_ADDR = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR = 0x05,
    BNO055_BL_REV_ID_ADDR = 0X06,

    /* Accel data register */
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08,
    BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09,
    BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A,
    BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B,
    BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C,
    BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D,

    /* Mag data register */
    BNO055_MAG_DATA_X_LSB_ADDR = 0X0E,
    BNO055_MAG_DATA_X_MSB_ADDR = 0X0F,
    BNO055_MAG_DATA_Y_LSB_ADDR = 0X10,
    BNO055_MAG_DATA_Y_MSB_ADDR = 0X11,
    BNO055_MAG_DATA_Z_LSB_ADDR = 0X12,
    BNO055_MAG_DATA_Z_MSB_ADDR = 0X13,

    /* Gyro data registers */
    BNO055_GYRO_DATA_X_LSB_ADDR = 0X14,
    BNO055_GYRO_DATA_X_MSB_ADDR = 0X15,
    BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16,
    BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17,
    BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18,
    BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19,

    /* Euler data registers */
    BNO055_EULER_H_LSB_ADDR = 0X1A,
    BNO055_EULER_H_MSB_ADDR = 0X1B,
    BNO055_EULER_R_LSB_ADDR = 0X1C,
    BNO055_EULER_R_MSB_ADDR = 0X1D,
    BNO055_EULER_P_LSB_ADDR = 0X1E,
    BNO055_EULER_P_MSB_ADDR = 0X1F,

    /* Quaternion data registers */
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20,
    BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21,
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22,
    BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23,
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24,
    BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25,
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26,
    BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27,

    /* Linear acceleration data registers */
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29,
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A,
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B,
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C,
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D,

    /* Gravity data registers */
    BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E,
    BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F,
    BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30,
    BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31,
    BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32,
    BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33,

    /* Temperature data register */
    BNO055_TEMP_ADDR = 0X34,

    /* Status registers */
    BNO055_CALIB_STAT_ADDR = 0X35,
    BNO055_SELFTEST_RESULT_ADDR = 0X36,
    BNO055_INTR_STAT_ADDR = 0X37,

    BNO055_SYS_CLK_STAT_ADDR = 0X38,
    BNO055_SYS_STAT_ADDR = 0X39,
    BNO055_SYS_ERR_ADDR = 0X3A,

    /* Unit selection register */
    BNO055_UNIT_SEL_ADDR = 0X3B,
    BNO055_DATA_SELECT_ADDR = 0X3C,

    /* Mode registers */
    BNO055_OPR_MODE_ADDR = 0X3D,
    BNO055_PWR_MODE_ADDR = 0X3E,

    BNO055_SYS_TRIGGER_ADDR = 0X3F,
    BNO055_TEMP_SOURCE_ADDR = 0X40,

    /* Axis remap registers */
    BNO055_AXIS_MAP_CONFIG_ADDR = 0X41,
    BNO055_AXIS_MAP_SIGN_ADDR = 0X42,

    /* SIC registers */
    BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43,
    BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44,
    BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45,
    BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46,
    BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47,
    BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48,
    BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49,
    BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50,
    BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51,
    BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52,
    BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53,
    BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54,

    /* Accelerometer Offset registers */
    ACCEL_OFFSET_X_LSB_ADDR = 0X55,
    ACCEL_OFFSET_X_MSB_ADDR = 0X56,
    ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
    ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
    ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
    ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

    /* Magnetometer Offset registers */
    MAG_OFFSET_X_LSB_ADDR = 0X5B,
    MAG_OFFSET_X_MSB_ADDR = 0X5C,
    MAG_OFFSET_Y_LSB_ADDR = 0X5D,
    MAG_OFFSET_Y_MSB_ADDR = 0X5E,
    MAG_OFFSET_Z_LSB_ADDR = 0X5F,
    MAG_OFFSET_Z_MSB_ADDR = 0X60,

    /* Gyroscope Offset register s*/
    GYRO_OFFSET_X_LSB_ADDR = 0X61,
    GYRO_OFFSET_X_MSB_ADDR = 0X62,
    GYRO_OFFSET_Y_LSB_ADDR = 0X63,
    GYRO_OFFSET_Y_MSB_ADDR = 0X64,
    GYRO_OFFSET_Z_LSB_ADDR = 0X65,
    GYRO_OFFSET_Z_MSB_ADDR = 0X66,

    /* Radius registers */
    ACCEL_RADIUS_LSB_ADDR = 0X67,
    ACCEL_RADIUS_MSB_ADDR = 0X68,
    MAG_RADIUS_LSB_ADDR = 0X69,
    MAG_RADIUS_MSB_ADDR = 0X6A
  } bno055_reg_t;

  /** BNO055 power settings */
  typedef enum {
    POWER_MODE_NORMAL = 0X00,
    POWER_MODE_LOWPOWER = 0X01,
    POWER_MODE_SUSPEND = 0X02
  } bno055_powermode_t;

  /** Operation mode settings **/
  typedef enum {
    OPERATION_MODE_CONFIG = 0X00,
    OPERATION_MODE_ACCONLY = 0X01,
    OPERATION_MODE_MAGONLY = 0X02,
    OPERATION_MODE_GYRONLY = 0X03,
    OPERATION_MODE_ACCMAG = 0X04,
    OPERATION_MODE_ACCGYRO = 0X05,
    OPERATION_MODE_MAGGYRO = 0X06,
    OPERATION_MODE_AMG = 0X07,
    OPERATION_MODE_IMUPLUS = 0X08,
    OPERATION_MODE_COMPASS = 0X09,
    OPERATION_MODE_M4G = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    OPERATION_MODE_NDOF = 0X0C
  } bno055_opmode_t;

  /** Remap settings **/
  typedef enum {
    REMAP_CONFIG_P0 = 0x21,
    REMAP_CONFIG_P1 = 0x24, // default
    REMAP_CONFIG_P2 = 0x24,
    REMAP_CONFIG_P3 = 0x21,
    REMAP_CONFIG_P4 = 0x24,
    REMAP_CONFIG_P5 = 0x21,
    REMAP_CONFIG_P6 = 0x21,
    REMAP_CONFIG_P7 = 0x24
  } bno055_axis_remap_config_t;

  /** Remap Signs **/
  typedef enum {
    REMAP_SIGN_P0 = 0x04,
    REMAP_SIGN_P1 = 0x00, // default
    REMAP_SIGN_P2 = 0x06,
    REMAP_SIGN_P3 = 0x02,
    REMAP_SIGN_P4 = 0x03,
    REMAP_SIGN_P5 = 0x01,
    REMAP_SIGN_P6 = 0x07,
    REMAP_SIGN_P7 = 0x05
  } bno055_axis_remap_sign_t;

  /** A structure to represent revisions **/
  typedef struct {
    uint8_t accel_rev; /**< acceleration rev */
    uint8_t mag_rev;   /**< magnetometer rev */
    uint8_t gyro_rev;  /**< gyroscrope rev */
    uint16_t sw_rev;   /**< SW rev */
    uint8_t bl_rev;    /**< bootloader rev */
  } bno055_rev_info_t;

  /** Vector Mappings **/
  typedef enum {
    VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_MAGNETOMETER = BNO055_MAG_DATA_X_LSB_ADDR,
    VECTOR_GYROSCOPE = BNO055_GYRO_DATA_X_LSB_ADDR,
    VECTOR_EULER = BNO055_EULER_H_LSB_ADDR,
    VECTOR_LINEARACCEL = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_GRAVITY = BNO055_GRAVITY_DATA_X_LSB_ADDR
  } vector_type_t;

  typedef enum {
    WRITE_SUCCESS = 0x01,
    READ_FAIL = 0x02,
    WRITE_FAIL = 0x03,
    REGMAP_INVALID_ADDRESS = 0x04,
    REGMAP_WRITE_DISABLED = 0x05,
    WRONG_START_BYTE = 0x06,
    BUS_OVER_RUN_ERROR = 0x07,
    MAX_LENGTH_ERROR = 0x08,
    MIN_LENGTH_ERROR = 0x09,
    RECEIVE_CHARACTER_TIMEOUT = 0x0a
  } uart_ack_type_t;
  BNO055(
      std::function<uint8_t( uint8_t *buffer, const uint8_t len)> &read_bytes,
      std::function<uint8_t(const uint8_t *buffer, const uint8_t len)> &write_bytes,
      std::function<void(uint32_t usec)> &delay );

//    BNO055(
//        uint8_t read_bytes( uint8_t *buffer, const uint8_t len) ,
//        uint8_t write_bytes(const uint8_t *buffer, const uint8_t len) ,
//        void *delay(uint32_t usec)  );


  /*!
   *  @brief  Sets up the HW
   *  @param  mode
   *          mode values
   *           [OPERATION_MODE_CONFIG,
   *            OPERATION_MODE_ACCONLY,
   *            OPERATION_MODE_MAGONLY,
   *            OPERATION_MODE_GYRONLY,
   *            OPERATION_MODE_ACCMAG,
   *            OPERATION_MODE_ACCGYRO,
   *            OPERATION_MODE_MAGGYRO,
   *            OPERATION_MODE_AMG,
   *            OPERATION_MODE_IMUPLUS,
   *            OPERATION_MODE_COMPASS,
   *            OPERATION_MODE_M4G,
   *            OPERATION_MODE_NDOF_FMC_OFF,
   *            OPERATION_MODE_NDOF]
   *  @return true if process is successful
   */
  bool begin(bno055_opmode_t mode);

  /*!
   *  @brief  Puts the chip in the specified operating mode
   *  @param  mode
   *          mode values
   *           [OPERATION_MODE_CONFIG,
   *            OPERATION_MODE_ACCONLY,
   *            OPERATION_MODE_MAGONLY,
   *            OPERATION_MODE_GYRONLY,
   *            OPERATION_MODE_ACCMAG,
   *            OPERATION_MODE_ACCGYRO,
   *            OPERATION_MODE_MAGGYRO,
   *            OPERATION_MODE_AMG,
   *            OPERATION_MODE_IMUPLUS,
   *            OPERATION_MODE_COMPASS,
   *            OPERATION_MODE_M4G,
   *            OPERATION_MODE_NDOF_FMC_OFF,
   *            OPERATION_MODE_NDOF]
   */
  uint8_t setMode(bno055_opmode_t mode);

  /*!
   *  @brief  Changes the chip's axis remap
   *  @param  remapcode
   *          remap code possible values
   *          [REMAP_CONFIG_P0
   *           REMAP_CONFIG_P1 (default)
   *           REMAP_CONFIG_P2
   *           REMAP_CONFIG_P3
   *           REMAP_CONFIG_P4
   *           REMAP_CONFIG_P5
   *           REMAP_CONFIG_P6
   *           REMAP_CONFIG_P7]
   */
  void setAxisRemap(bno055_axis_remap_config_t remapcode);

  /*!
   *  @brief  Changes the chip's axis signs
   *  @param  remapsign
   *          remap sign possible values
   *          [REMAP_SIGN_P0
   *           REMAP_SIGN_P1 (default)
   *           REMAP_SIGN_P2
   *           REMAP_SIGN_P3
   *           REMAP_SIGN_P4
   *           REMAP_SIGN_P5
   *           REMAP_SIGN_P6
   *           REMAP_SIGN_P7]
   */
  void setAxisSign(bno055_axis_remap_sign_t remapsign);

  /*!
   *  @brief  Use the external 32.768KHz crystal
   *  @param  usextal
   *          use external crystal boolean
   */
  void setExtCrystalUse(bool usextal);

  /*!
   *   @brief  Gets the latest system status info
   *   @param  system_status
   *           system status info
   *   @param  self_test_result
   *           self test result
   *   @param  system_error
   *           system error info
   */
  void getSystemStatus(uint8_t *system_status,
                       uint8_t *self_test_result,
                       uint8_t *system_error);

  /*!
   *  @brief  Gets the chip revision numbers
   *  @param  info
   *          revision info
   */
  void getRevInfo(bno055_rev_info_t *info);

  /*!
   *  @brief  Gets current calibration state.  Each value should be a uint8_t
   *          pointer and it will be set to 0 if not calibrated and 3 if
   *          fully calibrated.
   *          See section 34.3.54
   *  @param  sys
   *          Current system calibration status, depends on status of all sensors,
   * read_byte-only
   *  @param  gyro
   *          Current calibration status of Gyroscope, read_byte-only
   *  @param  accel
   *          Current calibration status of Accelerometer, read_byte-only
   *  @param  mag
   *          Current calibration status of Magnetometer, read_byte-only
   */
  void getCalibration(uint8_t *sys, uint8_t *gyro,
                      uint8_t *accel, uint8_t *mag);

  /*!
   *  @brief  Gets the temperature in degrees celsius
   *  @return temperature in degrees celsius
   */
  int8_t getTemp();

  /*!
   *  @brief   Gets a vector reading from the specified source
   *  @param   vector_type
   *           possible vector type values
   *           [VECTOR_ACCELEROMETER
   *            VECTOR_MAGNETOMETER
   *            VECTOR_GYROSCOPE
   *            VECTOR_EULER
   *            VECTOR_LINEARACCEL
   *            VECTOR_GRAVITY]
   *  @return  vector from specified source
   */
  float *getVector(vector_type_t vector_type);

  /*!
   *  @brief  Gets a quaternion reading from the specified source
   *  @return quaternion reading
   */
  float *getQuat();

  /*!
   *  @brief  Provides the sensor_t data for this sensor
   *  @param  sensor
   *          Sensor description
   */
  void getSensor(sensor_t *sensor) const;

  /*!
   *  @brief  Reads the sensor and returns the data as a sensors_event_t
   *  @param  event
   *          Event description
   *  @return always returns true
   */
  bool getEvent(sensors_event_t *event);

  /*!
   *  @brief  Reads the sensor and returns the data as a sensors_event_t
   *  @param  event
   *          Event description
   *  @param  vec_type
   *          specify the type of reading
   *  @return always returns true
   */
  bool getEvent(sensors_event_t *event, vector_type_t vec_type);

  /*!
   *  @brief  Reads the sensor's offset registers into a byte array
   *  @param  calibData
   *          Calibration offset (buffer size should be 22)
   *  @return true if read_byte is successful
   */
  bool getSensorOffsets(uint8_t *calibData);

  /*!
   *  @brief  Reads the sensor's offset registers into an offset struct
   *  @param  offsets_type
   *          type of offsets
   *  @return true if read_byte is successful
   */
  bool getSensorOffsets(bno055_offsets_t &offsets_type);

  /*!
   *  @brief  Writes an array of calibration values to the sensor's offset
   *  @param  calibData
   *          calibration data
   */
  void setSensorOffsets(const uint8_t *calibData);

  /*!
   *  @brief  Writes to the sensor's offset registers from an offset struct
   *  @param  offsets_type
   *          accel_offset_x = acceleration offset x
   *          accel_offset_y = acceleration offset y
   *          accel_offset_z = acceleration offset z
   *
   *          mag_offset_x   = magnetometer offset x
   *          mag_offset_y   = magnetometer offset y
   *          mag_offset_z   = magnetometer offset z
   *
   *          gyro_offset_x  = gyroscrope offset x
   *          gyro_offset_y  = gyroscrope offset y
   *          gyro_offset_z  = gyroscrope offset z
   */
  void setSensorOffsets(const bno055_offsets_t &offsets_type);

  /*!
   *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
   *  @return status of calibration
   */
  bool isFullyCalibrated();

  /*!
   *  @brief  Enter Suspend mode (i.e., sleep)
   */
  void enterSuspendMode();

  /*!
   *  @brief  Enter Normal mode (i.e., wake)
   */
  void enterNormalMode();

private:
  uint8_t _address{};
  int32_t _sensorID{};
  bno055_opmode_t _mode;
  uint8_t bno_reset();

  uint8_t write_reg(uint8_t reg, uint8_t value);
  uint8_t write_regs(uint8_t reg, const uint8_t *buffer,uint8_t len);;

  uint8_t read_reg(uint8_t reg, uint8_t *value);

  uint8_t read_regs(uint8_t reg,  uint8_t *buffer, uint8_t len);

  std::function<uint8_t( uint8_t *buffer, const uint8_t len)> &read_bytes;
  std::function<uint8_t(const uint8_t *buffer, const uint8_t len)> &write_bytes;
  std::function<void(uint32_t usec)> &delay;

//  typedef   uint8_t *read_bytes_type( uint8_t *buffer, const uint8_t len) ;
//  typedef  uint8_t *write_bytes_type(const uint8_t *buffer, const uint8_t len) ;
//  typedef  void *delay_type(uint32_t usec) ;

//  read_bytes_type read_bytes;
//   write_bytes_type write_bytes;
//  delay_type  delay;



};

#endif //BNO055DRIVER_SRC_BNO055_H
