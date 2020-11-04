

#include "BNO055.h"

bool BNO055::begin(BNO055::bno055_opmode_t mode) {
  volatile uint8_t result;
  /* Enable I2C */

  // BNO055 clock stretches for 500us or more!

  /* Make sure we have the right device */
  uint8_t id = 0;

  result= read_reg(BNO055_CHIP_ID_ADDR, &id);

  while (id != BNO055_ID) {
    delay(1000); // hold on for boot
    read_reg(BNO055_CHIP_ID_ADDR, &id);
    //    if (id != BNO055_ID) {
    //      return false; // still not? ok bail
    //    }
  }

  delay(30);
  /* Switch to config mode (just in case since this is the default) */
  result= setMode(OPERATION_MODE_CONFIG);

  delay(30);
  /* Reset */
  //  write_reg(BNO055_SYS_TRIGGER_ADDR, 0x20);  //return 0xee  no response type
  result= bno_reset();

  /* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
  delay(800);
  result = read_reg(BNO055_CHIP_ID_ADDR, &id);
  while (id != BNO055_ID) {
    delay(10);
  }
  delay(50);

  /* Set to normal power mode */
  result= write_reg(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  delay(10);

  result= write_reg(BNO055_PAGE_ID_ADDR, 0);

  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write_reg(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  write_reg(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  delay(10);
  write_reg(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  delay(10);
  */
  delay(10);
  result=write_reg(BNO055_SYS_TRIGGER_ADDR, 0x0);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  result=setMode(mode);
  delay(20);

  return true;
}
uint8_t BNO055::setMode(BNO055::bno055_opmode_t mode) {
  _mode = mode;
 uint8_t  result=write_reg(BNO055_OPR_MODE_ADDR, _mode);
  delay(30);
  return result;
}
void BNO055::setAxisRemap(BNO055::bno055_axis_remap_config_t remapcode) {
  bno055_opmode_t modeback = _mode;

  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write_reg(BNO055_AXIS_MAP_CONFIG_ADDR, remapcode);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}
void BNO055::setAxisSign(BNO055::bno055_axis_remap_sign_t remapsign) {
  bno055_opmode_t modeback = _mode;

  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write_reg(BNO055_AXIS_MAP_SIGN_ADDR, remapsign);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}
void BNO055::setExtCrystalUse(bool usextal) {
  bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write_reg(BNO055_PAGE_ID_ADDR, 0);
  if (usextal) {
    write_reg(BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else {
    write_reg(BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}
void BNO055::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error) {
  write_reg(BNO055_PAGE_ID_ADDR, 0);

  /* System Status (see section 4.3.58)
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms
   */

  if (system_status != 0)
    read_reg(BNO055_SYS_STAT_ADDR, system_status);

  /* Self Test Results
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good!
   */

  if (self_test_result != 0)
    read_reg(BNO055_SELFTEST_RESULT_ADDR, self_test_result);

  /* System Error (see section 4.3.59)
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error
   */

  if (system_error != 0)
    read_reg(BNO055_SYS_ERR_ADDR, system_status);

  delay(200);
}
void BNO055::getRevInfo(BNO055::bno055_rev_info_t *info) {
  uint8_t a, b;

  memset(info, 0, sizeof(bno055_rev_info_t));

  /* Check the accelerometer revision */
  read_reg(BNO055_ACCEL_REV_ID_ADDR, &info->accel_rev);

  /* Check the magnetometer revision */
  read_reg(BNO055_MAG_REV_ID_ADDR, &info->mag_rev);

  /* Check the gyroscope revision */
  read_reg(BNO055_GYRO_REV_ID_ADDR, &info->gyro_rev);

  /* Check the SW revision */
  read_reg(BNO055_BL_REV_ID_ADDR, &info->bl_rev);

  read_reg(BNO055_SW_REV_ID_LSB_ADDR, &a);
  read_reg(BNO055_SW_REV_ID_MSB_ADDR, &b);
  info->sw_rev = (((uint16_t) b) << 8) | ((uint16_t) a);
}
void BNO055::getCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
  uint8_t calData;
  read_reg(BNO055_CALIB_STAT_ADDR, &calData);
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
}
int8_t BNO055::getTemp() {
  int8_t temp;
  read_reg(BNO055_TEMP_ADDR, (uint8_t *) &temp);
  return temp;
}
float *BNO055::getVector(BNO055::vector_type_t vector_type) {
  static float xyz[3];
  uint8_t buffer[6];
  memset(buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  read_regs((bno055_reg_t) vector_type, buffer, 6);

  x = ((int16_t) buffer[0]) | (((int16_t) buffer[1]) << 8);
  y = ((int16_t) buffer[2]) | (((int16_t) buffer[3]) << 8);
  z = ((int16_t) buffer[4]) | (((int16_t) buffer[5]) << 8);

  /*!
   * Convert the value to an appropriate range (section 3.6.4)
   * and assign the value to the Vector type
   */
  switch (vector_type) {
  case VECTOR_MAGNETOMETER:
    /* 1uT = 16 LSB */
    xyz[0] = ((double) x) / 16.0;
    xyz[1] = ((double) y) / 16.0;
    xyz[2] = ((double) z) / 16.0;
    break;
  case VECTOR_GYROSCOPE:
    /* 1dps = 16 LSB */
    xyz[0] = ((double) x) / 16.0;
    xyz[1] = ((double) y) / 16.0;
    xyz[2] = ((double) z) / 16.0;
    break;
  case VECTOR_EULER:
    /* 1 degree = 16 LSB */
    xyz[0] = ((double) x) / 16.0;
    xyz[1] = ((double) y) / 16.0;
    xyz[2] = ((double) z) / 16.0;
    break;
  case VECTOR_ACCELEROMETER:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double) x) / 100.0;
    xyz[1] = ((double) y) / 100.0;
    xyz[2] = ((double) z) / 100.0;
    break;
  case VECTOR_LINEARACCEL:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double) x) / 100.0;
    xyz[1] = ((double) y) / 100.0;
    xyz[2] = ((double) z) / 100.0;
    break;
  case VECTOR_GRAVITY:
    /* 1m/s^2 = 100 LSB */
    xyz[0] = ((double) x) / 100.0;
    xyz[1] = ((double) y) / 100.0;
    xyz[2] = ((double) z) / 100.0;
    break;
  }

  return xyz;
}
float *BNO055::getQuat() {
  uint8_t buffer[8];
  memset(buffer, 0, 8);

  int16_t x, y, z, w;
  x = y = z = w = 0;

  /* Read quat data (8 bytes) */
  if (read_regs(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8)) {

  };
  w = (((uint16_t) buffer[1]) << 8) | ((uint16_t) buffer[0]);
  x = (((uint16_t) buffer[3]) << 8) | ((uint16_t) buffer[2]);
  y = (((uint16_t) buffer[5]) << 8) | ((uint16_t) buffer[4]);
  z = (((uint16_t) buffer[7]) << 8) | ((uint16_t) buffer[6]);

  /*!
   * Assign to Quaternion
   * See
   * http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
   * 3.6.5.5 Orientation (Quaternion)
   */
  const double scale = (1.0 / (1 << 14));
  static float quat[4];
  quat[0] = scale * w;
  quat[1] = scale * x;
  quat[2] = scale * y;
  quat[3] = scale * z;
  return quat;
}
void BNO055::getSensor(sensor_t *sensor) const {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "BNO055", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ORIENTATION;
  sensor->min_delay = 0;
  sensor->max_value = 0.0F;
  sensor->min_value = 0.0F;
  sensor->resolution = 0.01F;
}
bool BNO055::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_ORIENTATION;
  //  event->timestamp = millis();

  /* Get a Euler angle sample for orientation */
  float *euler = getVector(VECTOR_EULER);
  event->orientation.x = euler[0];
  event->orientation.y = euler[1];
  event->orientation.z = euler[2];

  return true;
}
bool BNO055::getEvent(sensors_event_t *event, BNO055::vector_type_t vec_type) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  //  event->timestamp = millis();

  // read_reg the data according to vec_type

  if (vec_type == VECTOR_LINEARACCEL) {
    event->type = SENSOR_TYPE_LINEAR_ACCELERATION;
    float *vec = getVector(VECTOR_LINEARACCEL);

    event->acceleration.x = vec[0];
    event->acceleration.y = vec[1];
    event->acceleration.z = vec[2];
  } else if (vec_type == VECTOR_ACCELEROMETER) {
    event->type = SENSOR_TYPE_ACCELEROMETER;
    float *vec = getVector(VECTOR_ACCELEROMETER);

    event->acceleration.x = vec[0];
    event->acceleration.y = vec[1];
    event->acceleration.z = vec[2];
  } else if (vec_type == VECTOR_GRAVITY) {
    event->type = SENSOR_TYPE_ACCELEROMETER;
    float *vec = getVector(VECTOR_GRAVITY);

    event->acceleration.x = vec[0];
    event->acceleration.y = vec[1];
    event->acceleration.z = vec[2];
  } else if (vec_type == VECTOR_EULER) {
    event->type = SENSOR_TYPE_ORIENTATION;
    float *vec = getVector(VECTOR_EULER);

    event->orientation.x = vec[0];
    event->orientation.y = vec[1];
    event->orientation.z = vec[2];
  } else if (vec_type == VECTOR_GYROSCOPE) {
    event->type = SENSOR_TYPE_ROTATION_VECTOR;
    float *vec = getVector(VECTOR_GYROSCOPE);

    event->gyro.x = vec[0];
    event->gyro.y = vec[1];
    event->gyro.z = vec[2];
  } else if (vec_type == VECTOR_MAGNETOMETER) {
    event->type = SENSOR_TYPE_MAGNETIC_FIELD;
    float *vec = getVector(VECTOR_MAGNETOMETER);

    event->magnetic.x = vec[0];
    event->magnetic.y = vec[1];
    event->magnetic.z = vec[2];
  }

  return true;
}
bool BNO055::getSensorOffsets(uint8_t *calibData) {
  if (isFullyCalibrated()) {
    bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);

    read_regs(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS);

    setMode(lastMode);
    return true;
  }
  return false;
}
bool BNO055::getSensorOffsets(bno055_offsets_t &offsets_type) {
  if (isFullyCalibrated()) {
    bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    delay(25);

    /* Accel offset range depends on the G-range:
       +/-2g  = +/- 2000 mg
       +/-4g  = +/- 4000 mg
       +/-8g  = +/- 8000 mg
       +/-1§g = +/- 16000 mg */

    uint8_t data_lsb = 0, data_msb = 0;
    read_reg(ACCEL_OFFSET_X_MSB_ADDR, &data_msb);
    read_reg(ACCEL_OFFSET_X_LSB_ADDR, &data_lsb);
    offsets_type.accel_offset_x = data_msb << 8 | data_lsb;

    read_reg(ACCEL_OFFSET_Y_MSB_ADDR, &data_msb);
    read_reg(ACCEL_OFFSET_Y_LSB_ADDR, &data_lsb);
    offsets_type.accel_offset_y = data_msb << 8 | data_lsb;

    read_reg(ACCEL_OFFSET_Z_MSB_ADDR, &data_msb);
    read_reg(ACCEL_OFFSET_Z_LSB_ADDR, &data_lsb);
    offsets_type.accel_offset_z = data_msb << 8 | data_lsb;



    /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */

    read_reg(MAG_OFFSET_X_MSB_ADDR, &data_msb);
    read_reg(MAG_OFFSET_X_LSB_ADDR, &data_lsb);
    offsets_type.mag_offset_x = data_msb << 8 | data_lsb;

    read_reg(MAG_OFFSET_Y_MSB_ADDR, &data_msb);
    read_reg(MAG_OFFSET_Y_LSB_ADDR, &data_lsb);
    offsets_type.mag_offset_y = data_msb << 8 | data_lsb;

    read_reg(MAG_OFFSET_Z_MSB_ADDR, &data_msb);
    read_reg(MAG_OFFSET_Z_LSB_ADDR, &data_lsb);
    offsets_type.mag_offset_z = data_msb << 8 | data_lsb;



    /* Gyro offset range depends on the DPS range:
      2000 dps = +/- 32000 LSB
      1000 dps = +/- 16000 LSB
       500 dps = +/- 8000 LSB
       250 dps = +/- 4000 LSB
       125 dps = +/- 2000 LSB
       ... where 1 DPS = 16 LSB */

    read_reg(GYRO_OFFSET_X_MSB_ADDR, &data_msb);
    read_reg(GYRO_OFFSET_X_LSB_ADDR, &data_lsb);
    offsets_type.gyro_offset_x = data_msb << 8 | data_lsb;

    read_reg(GYRO_OFFSET_Y_MSB_ADDR, &data_msb);
    read_reg(GYRO_OFFSET_Y_LSB_ADDR, &data_lsb);
    offsets_type.gyro_offset_y = data_msb << 8 | data_lsb;

    read_reg(GYRO_OFFSET_Z_MSB_ADDR, &data_msb);
    read_reg(GYRO_OFFSET_Z_LSB_ADDR, &data_lsb);
    offsets_type.gyro_offset_z = data_msb << 8 | data_lsb;


    /* Accelerometer radius = +/- 1000 LSB */

    read_reg(ACCEL_RADIUS_MSB_ADDR, &data_msb);
    read_reg(ACCEL_RADIUS_LSB_ADDR, &data_lsb);
    offsets_type.accel_radius = data_msb << 8 | data_lsb;

    /* Magnetometer radius = +/- 960 LSB */


    read_reg(MAG_RADIUS_MSB_ADDR, &data_msb);
    read_reg(MAG_RADIUS_LSB_ADDR, &data_lsb);
    offsets_type.mag_radius = data_msb << 8 | data_lsb;

    setMode(lastMode);
    return true;
  }
  return false;
}
void BNO055::setSensorOffsets(const uint8_t *calibData) {
  bno055_opmode_t lastMode = _mode;
  setMode(OPERATION_MODE_CONFIG);
  delay(25);

  /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */

  /* A writeLen() would make this much cleaner */
  write_reg(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
  write_reg(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
  write_reg(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
  write_reg(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
  write_reg(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
  write_reg(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

  write_reg(MAG_OFFSET_X_LSB_ADDR, calibData[6]);
  write_reg(MAG_OFFSET_X_MSB_ADDR, calibData[7]);
  write_reg(MAG_OFFSET_Y_LSB_ADDR, calibData[8]);
  write_reg(MAG_OFFSET_Y_MSB_ADDR, calibData[9]);
  write_reg(MAG_OFFSET_Z_LSB_ADDR, calibData[10]);
  write_reg(MAG_OFFSET_Z_MSB_ADDR, calibData[11]);

  write_reg(GYRO_OFFSET_X_LSB_ADDR, calibData[12]);
  write_reg(GYRO_OFFSET_X_MSB_ADDR, calibData[13]);
  write_reg(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);
  write_reg(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);
  write_reg(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);
  write_reg(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);

  write_reg(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
  write_reg(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

  write_reg(MAG_RADIUS_LSB_ADDR, calibData[20]);
  write_reg(MAG_RADIUS_MSB_ADDR, calibData[21]);

  setMode(lastMode);
}
void BNO055::setSensorOffsets(const bno055_offsets_t &offsets_type) {
  bno055_opmode_t lastMode = _mode;
  setMode(OPERATION_MODE_CONFIG);
  delay(25);

  /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */

  write_reg(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
  write_reg(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
  write_reg(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
  write_reg(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
  write_reg(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
  write_reg(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

  write_reg(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
  write_reg(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
  write_reg(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
  write_reg(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
  write_reg(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
  write_reg(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

  write_reg(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
  write_reg(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
  write_reg(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
  write_reg(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
  write_reg(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
  write_reg(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

  write_reg(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
  write_reg(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

  write_reg(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
  write_reg(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

  setMode(lastMode);
}
bool BNO055::isFullyCalibrated() {
  uint8_t system, gyro, accel, mag;
  getCalibration(&system, &gyro, &accel, &mag);

  switch (_mode) {
  case OPERATION_MODE_ACCONLY:return (accel == 3);
  case OPERATION_MODE_MAGONLY:return (mag == 3);
  case OPERATION_MODE_GYRONLY:
  case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
    return (gyro == 3);
  case OPERATION_MODE_ACCMAG:
  case OPERATION_MODE_COMPASS:return (accel == 3 && mag == 3);
  case OPERATION_MODE_ACCGYRO:
  case OPERATION_MODE_IMUPLUS:return (accel == 3 && gyro == 3);
  case OPERATION_MODE_MAGGYRO:return (mag == 3 && gyro == 3);
  default:return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
  }
}
void BNO055::enterSuspendMode() {
  bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write_reg(BNO055_PWR_MODE_ADDR, 0x02);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}
void BNO055::enterNormalMode() {
  bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(OPERATION_MODE_CONFIG);
  delay(25);
  write_reg(BNO055_PWR_MODE_ADDR, 0x00);
  /* Set the requested operating mode (see section 3.3) */
  setMode(modeback);
  delay(20);
}
BNO055::BNO055(std::function<uint8_t(uint8_t *, const uint8_t)> &read_bytes, std::function<uint8_t(const uint8_t *, const uint8_t)> &write_bytes, std::function<void(uint32_t)> &delay)
    : read_bytes(read_bytes), write_bytes(write_bytes), delay(delay) {

}

//
//BNO055::BNO055( uint8_t _read_bytes( uint8_t *buffer, const uint8_t len) ,
//                uint8_t write_bytes(const uint8_t *buffer, const uint8_t len) ,
//                void delay(uint32_t usec) ){
//}

uint8_t BNO055::write_regs(const uint8_t reg, const uint8_t *buffer, const uint8_t len) {
  begin_transmission
  uint8_t tx_data[len + 4];
  tx_data[0] = 0xaa;
  tx_data[1] = 0x00;
  tx_data[2] = reg;
  tx_data[3] = len;
  for (int i = 0; i < len; i++) {
    tx_data[i + 4] = buffer[i];
  }
  write_bytes(tx_data, len + 4);

  uint8_t data[2]{};
  read_bytes(data, 2);
  end_transmission
  if (data[0] == 0xee) {
    if (data[1] == 0x01) {
      return 0;
    } else {
      return data[1];
    }
  } else {
    return -1;
  }
}

uint8_t BNO055::write_reg(const uint8_t reg, uint8_t value) {
  return write_regs(reg, &value, 1);
}
uint8_t BNO055::read_reg(const uint8_t reg, uint8_t *value) {
  return read_regs(reg, value, 1);

}
uint8_t BNO055::read_regs(const uint8_t reg, uint8_t *buffer, const uint8_t len) {
  begin_transmission

  uint8_t tx_data[4] = {
      0xaa, 0x01, reg, len
  };

  write_bytes(tx_data, 4);

  //  write_byte(0xaa);  //start
  //  write_byte(0x01);  //read
  //  write_byte(reg);  //reg addr
  //  write_byte(len); //length

  uint8_t rx_data = 0;
  read_bytes(&rx_data, 1);
  if (rx_data == 0xbb) {
    read_bytes(&rx_data, 1);
    if (rx_data == len) {
      for (uint8_t i = 0; i < len; i++) {
        read_bytes(&rx_data, 1);
        buffer[i] = rx_data;
      }
      end_transmission
      return 0;
    } else {
      //        uint8_t len=data;
      //        while (len--){
      //          read_byte(&data);
      //        }
      end_transmission
      return -1;
    }
  } else if (rx_data == 0xee) {
    read_bytes(&rx_data, 1);
    end_transmission
    return rx_data;
  } else {
    end_transmission
    return -1;

  }
}
uint8_t BNO055::bno_reset() {
  begin_transmission

  uint8_t tx_data[5];
  tx_data[0] = 0xaa;
  tx_data[1] = 0x00;
  tx_data[2] = BNO055_SYS_TRIGGER_ADDR;
  tx_data[3] = 0x01;
  tx_data[4] = 0x20;
  write_bytes(tx_data, 5);

  uint8_t data = 0;
  read_bytes(&data, 1);
  end_transmission
  if (data == 0xee) {
    return 0;
  } else {
    return -1;
  }

}
