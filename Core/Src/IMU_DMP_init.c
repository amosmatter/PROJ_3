/*
 * IMU_DMP_init.c
 *
 *  Created on: Nov 22, 2023
 *      Author: amosm
 */

#include "ICM_20948/src/util/ICM_20948_C.h"

#include "ICM_20948/src/util/ICM_20948_DMP.h"
#include "ICM_20948/src/util/AK09916_REGISTERS.h"
#include "spi_common.h"
#include <stdio.h>
/**
 * Taken from ICM library
 */

static ICM_20948_Status_e ICM_20948_int_enable_raw_data_ready(ICM_20948_Device_t *pdev, bool enable)
{
  ICM_20948_Status_e status;
  ICM_20948_INT_enable_t en;
  // storage
  status = ICM_20948_int_enable(pdev, NULL, &en); // read phase
  if (status != ICM_20948_Stat_Ok)
  {
    return status;
  }
  en.RAW_DATA_0_RDY_EN = enable;                 // change the setting
  status = ICM_20948_int_enable(pdev, &en, &en); // write phase w/ readback
  if (status != ICM_20948_Stat_Ok)
  {
    return status;
  }
  if (en.RAW_DATA_0_RDY_EN != enable)
  {
    status = ICM_20948_Stat_Err;
    return status;
  }
  return status;
}

void debugPrintStatus(ICM_20948_Status_e stat)
{
  switch (stat)
  {
  case ICM_20948_Stat_Ok:
    printf("All is well.");
    break;
  case ICM_20948_Stat_Err:
    printf("General Error");
    break;
  case ICM_20948_Stat_NotImpl:
    printf("Not Implemented");
    break;
  case ICM_20948_Stat_ParamErr:
    printf("Parameter Error");
    break;
  case ICM_20948_Stat_WrongID:
    printf("Wrong ID");
    break;
  case ICM_20948_Stat_InvalSensor:
    printf("Invalid Sensor");
    break;
  case ICM_20948_Stat_NoData:
    printf("Data Underflow");
    break;
  case ICM_20948_Stat_SensorNotSupported:
    printf("Sensor Not Supported");
    break;
  case ICM_20948_Stat_DMPNotSupported:
    printf("DMP Firmware Not Supported. Is #define ICM_20948_USE_DMP commented in util/ICM_20948_C.h?");
    break;
  case ICM_20948_Stat_DMPVerifyFail:
    printf("DMP Firmware Verification Failed");
    break;
  case ICM_20948_Stat_FIFONoDataAvail:
    printf("No FIFO Data Available");
    break;
  case ICM_20948_Stat_FIFOIncompleteData:
    printf("DMP data in FIFO was incomplete");
    break;
  case ICM_20948_Stat_FIFOMoreDataAvail:
    printf("More FIFO Data Available");
    break;
  case ICM_20948_Stat_UnrecognisedDMPHeader:
    printf("Unrecognised DMP Header");
    break;
  case ICM_20948_Stat_UnrecognisedDMPHeader2:
    printf("Unrecognised DMP Header2");
    break;
  case ICM_20948_Stat_InvalDMPRegister:
    printf("Invalid DMP Register");
    break;
  default:
    printf("Unknown Status");
    break;
  }
}

ICM_20948_Status_e ICM_20948_mag_who_i_am(ICM_20948_Device_t *pdev)
{
  ICM_20948_Status_e status = ICM_20948_Stat_Ok;

  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  uint8_t whoiam1, whoiam2;
  status = ICM_20948_i2c_master_single_r(pdev, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA1, &whoiam1);
  // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
  // i2cMasterSingleR updates status so it is OK to set retval to status here
  retval = status;
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::ICM_20948_mag_who_i_am: whoiam1: %d", (int)whoiam1);
    printf(" (should be 72) readMag set status to: ");
    debugPrintStatus(status);
    printf("\r\n");
    return retval;
  }
  status = ICM_20948_i2c_master_single_r(pdev, MAG_AK09916_I2C_ADDR, AK09916_REG_WIA2, &whoiam2);
  // readMag calls i2cMasterSingleR which calls ICM_20948_i2c_master_single_r
  // i2cMasterSingleR updates status so it is OK to set retval to status here
  retval = status;
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::ICM_20948_mag_who_i_am: whoiam1: %d", (int)whoiam1);
    printf(" (should be 72) whoiam2: %d", (int)whoiam2);
    printf(" (should be 9) readMag set status to: ");
    debugPrintStatus(status);
    printf("\r\n");
    return retval;
  }

  if ((whoiam1 == (MAG_AK09916_WHO_AM_I >> 8)) && (whoiam2 == (MAG_AK09916_WHO_AM_I & 0xFF)))
  {
    retval = ICM_20948_Stat_Ok;
    status = retval;
    return status;
  }

  printf("ICM_20948::ICM_20948_mag_who_i_am: whoiam1: %d", (int)whoiam1);
  printf(" (should be 72) whoiam2: %d", (int)whoiam2);
  printf("\r\n");

  retval = ICM_20948_Stat_WrongID;
  status = retval;
  return status;
}

#define MAX_MAGNETOMETER_STARTS 10
ICM_20948_Status_e ICM_20948_startup_magnetometer(ICM_20948_Device_t *pdev)
{
  ICM_20948_Status_e status = ICM_20948_Stat_Ok;

  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  ICM_20948_i2c_master_passthrough(pdev, false); // Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
  ICM_20948_i2c_master_enable(pdev, true);
  uint8_t SRST = 1;
  ICM_20948_i2c_master_single_w(pdev, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL3, &SRST);

  // After a ICM reset the Mag sensor may stop responding over the I2C master
  // Reset the Master I2C until it responds
  uint8_t tries = 0;
  while (tries < MAX_MAGNETOMETER_STARTS)
  {
    tries++;

    // See if we can read the WhoIAm register correctly
    retval = ICM_20948_mag_who_i_am(pdev);
    if (retval == ICM_20948_Stat_Ok)
      break; // WIA matched!

    ICM_20948_i2c_master_reset(pdev); // Otherwise, reset the master I2C and try again

    delay_ms(10);
  }

  if (tries == MAX_MAGNETOMETER_STARTS)
  {
    printf("ICM_20948::ICM_20948_startup_magnetometer: reached MAX_MAGNETOMETER_STARTS (%d) \r\n", MAX_MAGNETOMETER_STARTS);

    status = ICM_20948_Stat_WrongID;
  }
  else
  {
    if (tries > 1)
    {
      printf("ICM_20948::ICM_20948_startup_magnetometer: successful ICM_20948_mag_who_i_am after  %d tries\r\n", tries);
    }
  }
  return status;

}

ICM_20948_Status_e ICM_20948_startup_default(ICM_20948_Device_t *pdev, const ICM_20948_Serif_t *pserif, uint32_t minimal)
{
  ICM_20948_Status_e status = ICM_20948_Stat_Ok;

  ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

  // Initialize myICM
  retval = ICM_20948_init_struct(pdev);

  // Link the serif
  retval = ICM_20948_link_serif(pdev, pserif);

  pdev->_dmp_firmware_available = true; // Initialize _dmp_firmware_available

  pdev->_firmware_loaded = false; // Initialize _firmware_loaded
  pdev->_last_bank = 255;         // Initialize _last_bank. Make it invalid. It will be set by the first call of ICM_20948_set_bank.
  pdev->_last_mems_bank = 255;    // Initialize _last_mems_bank. Make it invalid. It will be set by the first call of inv_icm20948_write_mems.
  pdev->_gyroSF = 0;              // Use this to record the GyroSF, calculated by inv_icm20948_set_gyro_sf
  pdev->_gyroSFpll = 0;
  pdev->_enabled_Android_0 = 0;      // Keep track of which Android sensors are enabled: 0-31
  pdev->_enabled_Android_1 = 0;      // Keep track of which Android sensors are enabled: 32-
  pdev->_enabled_Android_intr_0 = 0; // Keep track of which Android sensor interrupts are enabled: 0-31
  pdev->_enabled_Android_intr_1 = 0; // Keep track of which Android sensor interrupts are enabled: 32-

  retval = ICM_20948_check_id(pdev);
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::startupDefault: checkID returned: ");
    debugPrintStatus(retval);
    printf("\r\n");
    status = retval;
    return status;
  }

  retval = ICM_20948_sw_reset(pdev);
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::startupDefault: swReset returned: ");
    debugPrintStatus(retval);
    printf("\r\n");
    status = retval;
    return status;
  }
  delay_ms(50);

  retval = ICM_20948_sleep(pdev, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::startupDefault: sleep returned: ");
    debugPrintStatus(retval);
    printf("\r\n");
    status = retval;
    return status;
  }

  retval = ICM_20948_low_power(pdev, false);
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::startupDefault: lowPower returned: ");
    debugPrintStatus(retval);
    printf("\r\n");
    status = retval;
    return status;
  }

  retval = ICM_20948_startup_magnetometer(pdev); // Pass the minimal startup flag to ICM_20948_startup_magnetometer
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::startupDefault: ICM_20948_startup_magnetometer returned: ");
    debugPrintStatus(retval);
    printf("\r\n");
    status = retval;
    return status;
  }

  if (minimal) // Return now if minimal is true
  {
    printf("\r\n");
    return status;
  }

  retval = ICM_20948_set_sample_mode(pdev, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous); // options: ICM_20948_Sample_Mode_Continuous or ICM_20948_Sample_Mode_Cycled
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::startupDefault: setSampleMode returned: ");
    debugPrintStatus(retval);
    printf("\r\n");
    status = retval;
    return status;
  } // sensors: 	ICM_20948_Internal_Acc, ICM_20948_Internal_Gyr, ICM_20948_Internal_Mst

  ICM_20948_fss_t FSS;
  FSS.a = gpm4;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  FSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  retval = ICM_20948_set_full_scale(pdev, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), FSS);
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::startupDefault: setFullScale returned: ");
    debugPrintStatus(retval);
    printf("\r\n");
    status = retval;
    return status;
  }

  ICM_20948_dlpcfg_t dlpcfg;
  dlpcfg.a = acc_d473bw_n499bw;
  dlpcfg.g = gyr_d196bw6_n229bw8;
  retval = ICM_20948_set_dlpf_cfg(pdev, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::startupDefault: setDLPFcfg returned: ");
    debugPrintStatus(retval);
    printf("\r\n");
    status = retval;
    return status;
  }

  retval = ICM_20948_enable_dlpf(pdev, ICM_20948_Internal_Acc, true);
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::startupDefault: enableDLPF (Acc) returned: ");
    debugPrintStatus(retval);
    printf("\r\n");
    status = retval;
    return status;
  }

  retval = ICM_20948_enable_dlpf(pdev, ICM_20948_Internal_Gyr, true);
  if (retval != ICM_20948_Stat_Ok)
  {
    printf("ICM_20948::startupDefault: enableDLPF (Gyr) returned: ");
    debugPrintStatus(retval);
    printf("\r\n");
    status = retval;
    return status;
  }

  return status;
}

// DMP Startup Code from ICM_20948.cpp
ICM_20948_Status_e ICM_20948_initialize_DMP(ICM_20948_Device_t *pdev)
{

  ICM_20948_Status_e worstResult = ICM_20948_Stat_Ok;

  // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
  // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

  ICM_20948_Status_e result = ICM_20948_Stat_Ok; // Use result and worstResult to show if the configuration was successful

  // Normally, when the DMP is not enabled, ICM_20948_startup_magnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
  // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
  // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
  // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
  //
  // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
  // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
  // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
  // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
  // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
  //
  // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
  // 0: use I2C_SLV0
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
  // 10: we read 10 bytes each cycle
  // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)

 // ADDITION
  result = ICM_20948_i2c_controller_configure_peripheral(pdev, 0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false, 0);
  if (result > worstResult)
    worstResult = result;

  result = ICM_20948_i2c_controller_configure_peripheral(pdev, 0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true, 0);
  if (result > worstResult)
    worstResult = result;
  //
  // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
  // 1: use I2C_SLV1
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_CNTL2: we start writing here (0x31)
  // 1: not sure why, but the write does not happen if this is set to zero
  // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
  // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
  result = ICM_20948_i2c_controller_configure_peripheral(pdev, 1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single);
  if (result > worstResult)
    worstResult = result;






  // Set the I2C Master ODR configuration
  // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
  // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
  //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
  //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
  //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
  // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
  // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
  result = ICM_20948_set_bank(pdev, 3);
  if (result > worstResult)
    worstResult = result;
  uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
  result = ICM_20948_execute_w(pdev, AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1);
  if (result > worstResult)
    worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  result = ICM_20948_set_clock_source(pdev, ICM_20948_Clock_Auto);
  if (result > worstResult)
    worstResult = result; // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  result = ICM_20948_set_bank(pdev, 0);
  if (result > worstResult)
    worstResult = result;  // Select Bank 0
  uint8_t pwrMgmt2 = 0x40; // Set the reserved bit 6 (pressure sensor disable?)
  result = ICM_20948_execute_w(pdev, AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1);

  if (result > worstResult)
    worstResult = result; // Write one byte to the PWR_MGMT_2 register

  // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
  // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
  result = ICM_20948_set_sample_mode(pdev, ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled);
  if (result > worstResult)
    worstResult = result;

  // Disable the FIFO
  result = ICM_20948_enable_FIFO(pdev, false);
  if (result > worstResult)
    worstResult = result;

  // Disable the DMP
  result = ICM_20948_enable_DMP(pdev, false);
  if (result > worstResult)
    worstResult = result;

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                         // gpm2
                         // gpm4
                         // gpm8
                         // gpm16
  myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                         // dps250
                         // dps500
                         // dps1000
                         // dps2000
  result = ICM_20948_set_full_scale(pdev, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (result > worstResult)
    worstResult = result;

  // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
  // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
  // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
  result = ICM_20948_enable_dlpf(pdev, ICM_20948_Internal_Gyr, true);
  if (result > worstResult)
    worstResult = result;

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  // result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  result = ICM_20948_set_bank(pdev, 0);
  if (result > worstResult)
    worstResult = result; // Select Bank 0
  uint8_t zero = 0;
  result = ICM_20948_execute_w(pdev, AGB0_REG_FIFO_EN_1, &zero, 1);
  if (result > worstResult)
    worstResult = result;
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  result = ICM_20948_execute_w(pdev, AGB0_REG_FIFO_EN_2, &zero, 1);
  if (result > worstResult)
    worstResult = result;

  // Turn off data ready interrupt through INT_ENABLE_1
  result = ICM_20948_int_enable_raw_data_ready(pdev, false);
  if (result > worstResult)
    worstResult = result;

  // Reset FIFO through FIFO_RST
  result = ICM_20948_reset_FIFO(pdev);
  if (result > worstResult)
    worstResult = result;

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  // mySmplrt.g = 4; // 225Hz
  // mySmplrt.a = 4; // 225Hz
  // mySmplrt.g = 8; // 112Hz
  // mySmplrt.a = 8; // 112Hz
  result = ICM_20948_set_sample_rate(pdev, (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);
  if (result > worstResult)
    worstResult = result;

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = ICM_20948_set_dmp_start_address(pdev, DMP_START_ADDRESS);
  if (result > worstResult)
    worstResult = result; // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  result = ICM_20948_firmware_load(pdev);
  if (result > worstResult)
    worstResult = result;

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = ICM_20948_set_dmp_start_address(pdev, DMP_START_ADDRESS);
  if (result > worstResult)
    worstResult = result; // Defaults to DMP_START_ADDRESS

  // Set the Hardware Fix Disable register to 0x48
  result = ICM_20948_set_bank(pdev, 0);
  if (result > worstResult)
    worstResult = result; // Select Bank 0
  uint8_t fix = 0x48;
  result = ICM_20948_execute_w(pdev, AGB0_REG_HW_FIX_DISABLE, &fix, 1);
  if (result > worstResult)
    worstResult = result;

  // Set the Single FIFO Priority Select register to 0xE4
  result = ICM_20948_set_bank(pdev, 0);
  if (result > worstResult)
    worstResult = result; // Select Bank 0
  uint8_t fifoPrio = 0xE4;
  result = ICM_20948_execute_w(pdev, AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1);
  if (result > worstResult)
    worstResult = result;

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  result = inv_icm20948_write_mems(pdev, ACC_SCALE, 4, &accScale[0]);
  if (result > worstResult)
    worstResult = result; // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  result = inv_icm20948_write_mems(pdev, ACC_SCALE2, 4, &accScale2[0]);
  if (result > worstResult)
    worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  result = inv_icm20948_write_mems(pdev, CPASS_MTX_00, 4, &mountMultiplierPlus[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, CPASS_MTX_01, 4, &mountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, CPASS_MTX_02, 4, &mountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, CPASS_MTX_10, 4, &mountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, CPASS_MTX_11, 4, &mountMultiplierMinus[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, CPASS_MTX_12, 4, &mountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, CPASS_MTX_20, 4, &mountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, CPASS_MTX_21, 4, &mountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, CPASS_MTX_22, 4, &mountMultiplierMinus[0]);
  if (result > worstResult)
    worstResult = result;

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = inv_icm20948_write_mems(pdev, B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, B2S_MTX_01, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, B2S_MTX_02, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, B2S_MTX_10, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, B2S_MTX_12, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, B2S_MTX_20, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, B2S_MTX_21, 4, &b2sMountMultiplierZero[0]);
  if (result > worstResult)
    worstResult = result;
  result = inv_icm20948_write_mems(pdev, B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]);
  if (result > worstResult)
    worstResult = result;

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  result = inv_icm20948_set_gyro_sf(pdev, 19, 3);
  if (result > worstResult)
    worstResult = result; // 19 = 55Hz (see above), 3 = 2000dps (see above)

  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  result = inv_icm20948_write_mems(pdev, GYRO_FULLSCALE, 4, &gyroFullScale[0]);
  if (result > worstResult)
    worstResult = result;

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  // const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  // const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  result = inv_icm20948_write_mems(pdev, ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]);
  if (result > worstResult)
    worstResult = result;

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  // const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  // const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  result = inv_icm20948_write_mems(pdev, ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]);
  if (result > worstResult)
    worstResult = result;

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  // const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  // const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  result = inv_icm20948_write_mems(pdev, ACCEL_A_VAR, 4, &accelAVar[0]);
  if (result > worstResult)
    worstResult = result;

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = inv_icm20948_write_mems(pdev, ACCEL_CAL_RATE, 2, &accelCalRate[0]);
  if (result > worstResult)
    worstResult = result;

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  result = inv_icm20948_write_mems(pdev, CPASS_TIME_BUFFER, 2, &compassRate[0]);
  if (result > worstResult)
    worstResult = result;

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  // result = intEnableDMP(true); if (result > worstResult) worstResult = result;//TODO: activate

  return worstResult;
}
