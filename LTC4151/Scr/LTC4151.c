/*
 * LTC4151.c
 *
 */

#include "LTC4151.h"

/**
  * @brief  Initializes LTC4151 sensor.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @param  A0 Value of Address Pin 0 (L (0) = Tie to GND; H (1) = Tie High; NC (2) = Open;).
  * @param  A1 Value of Address Pin 1 (L (0) = Tie to GND; H (1) = Tie High; NC (2) = Open;).
  * @retval None.
  */
void init(struct LTC4151 *ltc4151, uint8_t A0, uint8_t A1)
{
	ltc4151->L = (uint8_t)0;
	ltc4151->H = (uint8_t)1;
	ltc4151->NC = (uint8_t)2;

	// Set the I2C address depending on ADR0 and ADR1 pins
	if (A0 == ltc4151->L && A1 == ltc4151->H) ltc4151->I2C_ADDRESS = 0b1100111;
	else if (A0 == ltc4151->H  && A1 == ltc4151->NC) ltc4151->I2C_ADDRESS = 0b1101000;
	else if (A0 == ltc4151->H  && A1 == ltc4151->H ) ltc4151->I2C_ADDRESS = 0b1101001;
	else if (A0 == ltc4151->NC && A1 == ltc4151->NC) ltc4151->I2C_ADDRESS = 0b1101010;
	else if (A0 == ltc4151->L  && A1 == ltc4151->NC) ltc4151->I2C_ADDRESS = 0b1101011;
	else if (A0 == ltc4151->H  && A1 == ltc4151->L ) ltc4151->I2C_ADDRESS = 0b1101100;
	else if (A0 == ltc4151->NC && A1 == ltc4151->H ) ltc4151->I2C_ADDRESS = 0b1101101;
	else if (A0 == ltc4151->NC && A1 == ltc4151->L ) ltc4151->I2C_ADDRESS = 0b1101110;
	else if (A0 == ltc4151->L  && A1 == ltc4151->L ) ltc4151->I2C_ADDRESS = 0b1101111;
	ltc4151->I2C_ADDRESS = ltc4151->I2C_ADDRESS << 1;
}

/**
  * @brief  Reads a register.
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @param  reg Register to read.
  * @param  numOfBytes Number of bytes to read.
  * @retval Contents of numOfBytes consecutive registers starting at register reg.
  */
long readADC(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, uint16_t reg, uint16_t numOfBytes)
{
	uint8_t buf[2];
	unsigned int h, l; // No son necesarios
	long result;

	HAL_I2C_Mem_Read(hi2c, ltc4151->I2C_ADDRESS, reg, 1, buf, numOfBytes, HAL_MAX_DELAY);

	if (numOfBytes == 1)
	{
		result = buf[0];
	} else if (numOfBytes == 2)
	{
		h = buf[0];
		l = buf[1];
		result = h << 4 | l >> 4;
	}

	return result;
}

/**
  * @brief  Reads a register one time when previous conversion is complete.
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @param  reg Register to read.
  * @retval Contents of 2 consecutive registers starting at register reg.
  */
long readADCSnapshot(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, uint16_t reg)
{
	uint8_t buf[2];
	long result;
	uint8_t ready = 0;

	while (ready == 0)
	{
		HAL_I2C_Mem_Read(hi2c, ltc4151->I2C_ADDRESS, reg, 1, buf, 2, HAL_MAX_DELAY);

		ready = ((buf[1] & 0x8) == 0);
	}

	result = buf[0] << 4 | buf[1] >> 4;

	return result;
}

/**
  * @brief  Writes to CONTROL Register G (06h).
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @param  ctrlReg Value to write to control register.
  * @retval None.
  */
void setControlRegister(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, uint8_t ctrlReg)
{
	uint8_t buf[1];
	buf[0] = ctrlReg;
	HAL_I2C_Mem_Write(hi2c, ltc4151->I2C_ADDRESS, REG_CTRL, 1, buf, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Gets the content of the control register.
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval Value of CONTROL Register G (06h).
  */
uint8_t getControlRegister(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151)
{
	return (uint8_t)(readADC(hi2c, ltc4151, REG_CTRL, 1));
}

/**
  * @brief  Measures voltage between S+ and S- terminals
  * 		and calculates current passing through the sense resistor.
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @param	r Sense resistor value in ohms (Rs).
  * @retval Value of current passing through the load resistor (Rs).
  */
double getLoadCurrent(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, double r)
{
	return readADC(hi2c, ltc4151, REG_SENSE_H, 2) * 81.92 / 4096.0 / r;
}

/**
  * @brief  Measures voltage in VIN terminal.
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval Value of voltage in VIN.
  */
double getInputVoltage(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151)
{
	return readADC(hi2c, ltc4151, REG_VIN_H, 2) * 102.4 / 4096.0;
}

/**
  * @brief  Measures voltage in VADIN terminal.
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval Value of voltage in VADIN.
  */
double getADCInVoltage(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151)
{
	return readADC(hi2c, ltc4151, REG_ADIN_H, 2) * 2.048 / 4096.0;
}

/**
  * @brief  Disables Snapshot Mode and sets ADC channel to default.
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval None.
  */
void disableSnapshotMode(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, uint8_t ctrlReg)
{
	if ((ctrlReg & (1 << CTRL_BIT_SNAPSHOT_ENABLE)) > 0)
	{
		ctrlReg = ctrlReg & ~(1 << CTRL_BIT_SNAPSHOT_ENABLE);

		ctrlReg = ctrlReg & ~(1 << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE);
		ctrlReg = ctrlReg & ~(1 << (CTRL_BIT_ADC_CHN_SNAPSHOT_MODE + 1));

		setControlRegister(hi2c, ltc4151, ctrlReg);
	}
}

/**
  * @brief  Takes a snapshot of voltage between S+ and S- terminals
  * 		and calculates current passing through the sense resistor.
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @param	r Sense resistor value in ohms (Rs).
  * @retval Value of current passing through the load resistor (Rs).
  */
double getSnapshotLoadCurrent(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, double r)
{
	uint8_t ctrlReg = getControlRegister(hi2c, ltc4151);
	disableSnapshotMode(hi2c, ltc4151, ctrlReg);

	ctrlReg |= SNAPSHOT_CHANNEL_SENSE << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE;
	setControlRegister(hi2c, ltc4151, ctrlReg);

	return readADCSnapshot(hi2c, ltc4151, REG_SENSE_H) * 81.92 / 4096.0 / r;
}

/**
  * @brief  Takes a snapshot of voltage in VIN terminal.
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval Value of voltage in VIN.
  */
double getSnapshotInputVoltage(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151)
{
	uint8_t ctrlReg = getControlRegister(hi2c, ltc4151);
	disableSnapshotMode(hi2c, ltc4151, ctrlReg);

	ctrlReg |= SNAPSHOT_CHANNEL_VIN << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE;
	setControlRegister(hi2c, ltc4151, ctrlReg);

	return readADCSnapshot(hi2c, ltc4151, REG_VIN_H) * 102.4 / 4096.0;
}

/**
  * @brief  Takes a snapshot of voltage in VADIN terminal.
  * @param	hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for the specified I2C.
  * @param	ltc4151 Pointer to a LTC4151 structure that contains the
  *         configuration of the LTC4151 IC.
  * @retval Value of voltage in VADIN.
  */
double getSnapshotADCInVoltage(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151)
{
	uint8_t ctrlReg = getControlRegister(hi2c, ltc4151);
	disableSnapshotMode(hi2c, ltc4151, ctrlReg);

	ctrlReg |= SNAPSHOT_CHANNEL_ADIN << CTRL_BIT_ADC_CHN_SNAPSHOT_MODE;
	setControlRegister(hi2c, ltc4151, ctrlReg);

	return readADCSnapshot(hi2c, ltc4151, REG_ADIN_H) * 2.048 / 4096.0;
}

