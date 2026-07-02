#ifndef SAVO_PERCEPTION_THIRD_PARTY_VL53L1X_ULD_PLATFORM_VL53L1_PLATFORM_H_
#define SAVO_PERCEPTION_THIRD_PARTY_VL53L1X_ULD_PLATFORM_VL53L1_PLATFORM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void VL53L1_SetI2CBus(int bus);
void VL53L1_CloseI2CBus(void);

void VL53L1X_SetI2CBus(int bus);
void VL53L1X_CloseI2CBus(void);

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t * pdata, uint32_t count);
int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t * pdata, uint32_t count);

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data);
int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data);
int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data);

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t * data);
int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t * data);
int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t * data);

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms);

int8_t VL53L1X_WriteMulti(uint16_t dev, uint16_t index, uint8_t * pdata, uint32_t count);
int8_t VL53L1X_ReadMulti(uint16_t dev, uint16_t index, uint8_t * pdata, uint32_t count);

int8_t VL53L1X_WrByte(uint16_t dev, uint16_t index, uint8_t data);
int8_t VL53L1X_WrWord(uint16_t dev, uint16_t index, uint16_t data);
int8_t VL53L1X_WrDWord(uint16_t dev, uint16_t index, uint32_t data);

int8_t VL53L1X_RdByte(uint16_t dev, uint16_t index, uint8_t * data);
int8_t VL53L1X_RdWord(uint16_t dev, uint16_t index, uint16_t * data);
int8_t VL53L1X_RdDWord(uint16_t dev, uint16_t index, uint32_t * data);

int8_t VL53L1X_WaitMs(uint16_t dev, int32_t wait_ms);

#ifdef __cplusplus
}
#endif

#endif  // SAVO_PERCEPTION_THIRD_PARTY_VL53L1X_ULD_PLATFORM_VL53L1_PLATFORM_H_
