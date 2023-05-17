/*
 * LTC4151.h
 *
 */

#ifndef LTC4151_H_
#define LTC4151_H_

#include <stdint.h>

#include "stm32g4xx_hal.h"

// Register directions
#define REG_SENSE_H 0x00 // (A) SENSE (8 MSBs)
#define REG_SENSE_L 0x01 // (B) SENSE (4 LSBs)
#define REG_VIN_H 0x02   // (C) VIN (8 MSBs)
#define REG_VIN_L 0x03   // (D) VIN (4 LSBs)
#define REG_ADIN_H 0x04  // (E) ADIN (8 MSBs)
#define REG_ADIN_L 0x05  // (F) ADIN (4 MSBs)
#define REG_CTRL 0x06    // (G) CONTROL

#define CTRL_BIT_SNAPSHOT_ENABLE 0x7

#define CTRL_BIT_ADC_CHN_SNAPSHOT_MODE 0x5

#define CTRL_BIT_TEST_MODE_ENABLE 0x4

#define CTRL_BIT_PAGE_RW_ENABLE 0x3

#define CTRL_BIT_STUCK_BUS_TIMER_ENABLE 0x2

#define SNAPSHOT_CHANNEL_SENSE 0x0
#define SNAPSHOT_CHANNEL_VIN 0x1
#define SNAPSHOT_CHANNEL_ADIN 0x2

// Structure that contains the configuration of the LTC4151 IC.
struct LTC4151
{
	volatile uint8_t L; // Low
	volatile uint8_t H; // High
	volatile uint8_t NC; // Float

	volatile uint16_t I2C_ADDRESS;
};


void init(struct LTC4151 *ltc4151, uint8_t A0, uint8_t A1);

long readADC(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, uint16_t reg, uint16_t numOfBytes);
long readADCSnapshot(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, uint16_t reg);

void setControlRegister(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, uint8_t ctrlReg);
uint8_t getControlRegister(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151);

double getLoadCurrent(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, double r);
double getInputVoltage(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151);
double getADCInVoltage(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151);

void disableSnapshotMode(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, uint8_t ctrlReg);

double getSnapshotLoadCurrent(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151, double r);
double getSnapshotInputVoltage(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151);
double getSnapshotADCInVoltage(I2C_HandleTypeDef* hi2c, struct LTC4151 *ltc4151);

#endif /* LTC4151_H_ */
