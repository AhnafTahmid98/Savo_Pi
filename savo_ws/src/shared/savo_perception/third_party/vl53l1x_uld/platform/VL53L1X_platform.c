#include "vl53l1_platform.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

static int g_i2c_bus = 1;

static uint8_t to_linux_i2c_addr(uint16_t dev)
{
  /*
   * ST VL53L1X ULD normally uses the 8-bit I2C address.
   * Linux i2c-dev needs the 7-bit address.
   *
   * VL53L1X default:
   *   Linux 7-bit address: 0x29
   *   ST API 8-bit address: 0x52
   */
  if (dev == 0x52U) {
    return 0x29U;
  }

  if (dev > 0x7FU) {
    return (uint8_t)(dev >> 1);
  }

  return (uint8_t)dev;
}

static int open_i2c_device(uint16_t dev)
{
  char path[32];
  const int written = snprintf(path, sizeof(path), "/dev/i2c-%d", g_i2c_bus);
  if (written <= 0 || written >= (int)sizeof(path)) {
    return -1;
  }

  const int fd = open(path, O_RDWR);
  if (fd < 0) {
    return -1;
  }

  const uint8_t addr = to_linux_i2c_addr(dev);
  if (ioctl(fd, I2C_SLAVE, addr) < 0) {
    close(fd);
    return -1;
  }

  return fd;
}

static ssize_t retry_write(int fd, const uint8_t * data, size_t count)
{
  ssize_t result;

  do {
    result = write(fd, data, count);
  } while (result < 0 && errno == EINTR);

  return result;
}

static ssize_t retry_read(int fd, uint8_t * data, size_t count)
{
  ssize_t result;

  do {
    result = read(fd, data, count);
  } while (result < 0 && errno == EINTR);

  return result;
}

void VL53L1_SetI2CBus(int bus)
{
  if (bus >= 0) {
    g_i2c_bus = bus;
  }
}

void VL53L1_CloseI2CBus(void)
{
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t * pdata, uint32_t count)
{
  if (pdata == NULL && count > 0U) {
    return -1;
  }

  const uint32_t total_count = count + 2U;
  uint8_t * buffer = (uint8_t *)malloc(total_count);
  if (buffer == NULL) {
    return -1;
  }

  buffer[0] = (uint8_t)((index >> 8) & 0xFFU);
  buffer[1] = (uint8_t)(index & 0xFFU);

  if (count > 0U) {
    memcpy(&buffer[2], pdata, count);
  }

  const int fd = open_i2c_device(dev);
  if (fd < 0) {
    free(buffer);
    return -1;
  }

  const ssize_t written = retry_write(fd, buffer, total_count);
  close(fd);
  free(buffer);

  return written == (ssize_t)total_count ? 0 : -1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t * pdata, uint32_t count)
{
  if (pdata == NULL || count == 0U) {
    return -1;
  }

  uint8_t reg[2];
  reg[0] = (uint8_t)((index >> 8) & 0xFFU);
  reg[1] = (uint8_t)(index & 0xFFU);

  const int fd = open_i2c_device(dev);
  if (fd < 0) {
    return -1;
  }

  const ssize_t written = retry_write(fd, reg, sizeof(reg));
  if (written != (ssize_t)sizeof(reg)) {
    close(fd);
    return -1;
  }

  const ssize_t read_count = retry_read(fd, pdata, count);
  close(fd);

  return read_count == (ssize_t)count ? 0 : -1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
  return VL53L1_WriteMulti(dev, index, &data, 1U);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
  uint8_t buffer[2];
  buffer[0] = (uint8_t)((data >> 8) & 0xFFU);
  buffer[1] = (uint8_t)(data & 0xFFU);

  return VL53L1_WriteMulti(dev, index, buffer, 2U);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
  uint8_t buffer[4];
  buffer[0] = (uint8_t)((data >> 24) & 0xFFU);
  buffer[1] = (uint8_t)((data >> 16) & 0xFFU);
  buffer[2] = (uint8_t)((data >> 8) & 0xFFU);
  buffer[3] = (uint8_t)(data & 0xFFU);

  return VL53L1_WriteMulti(dev, index, buffer, 4U);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t * data)
{
  return VL53L1_ReadMulti(dev, index, data, 1U);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t * data)
{
  if (data == NULL) {
    return -1;
  }

  uint8_t buffer[2];
  const int8_t status = VL53L1_ReadMulti(dev, index, buffer, 2U);
  if (status != 0) {
    return status;
  }

  *data = (uint16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
  return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t * data)
{
  if (data == NULL) {
    return -1;
  }

  uint8_t buffer[4];
  const int8_t status = VL53L1_ReadMulti(dev, index, buffer, 4U);
  if (status != 0) {
    return status;
  }

  *data =
    ((uint32_t)buffer[0] << 24) |
    ((uint32_t)buffer[1] << 16) |
    ((uint32_t)buffer[2] << 8) |
    (uint32_t)buffer[3];

  return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
  (void)dev;

  if (wait_ms <= 0) {
    return 0;
  }

  struct timespec req;
  req.tv_sec = wait_ms / 1000;
  req.tv_nsec = (long)(wait_ms % 1000) * 1000000L;

  while (nanosleep(&req, &req) != 0) {
    if (errno != EINTR) {
      return -1;
    }
  }

  return 0;
}

void VL53L1X_SetI2CBus(int bus)
{
  VL53L1_SetI2CBus(bus);
}

void VL53L1X_CloseI2CBus(void)
{
  VL53L1_CloseI2CBus();
}

int8_t VL53L1X_WriteMulti(uint16_t dev, uint16_t index, uint8_t * pdata, uint32_t count)
{
  return VL53L1_WriteMulti(dev, index, pdata, count);
}

int8_t VL53L1X_ReadMulti(uint16_t dev, uint16_t index, uint8_t * pdata, uint32_t count)
{
  return VL53L1_ReadMulti(dev, index, pdata, count);
}

int8_t VL53L1X_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
  return VL53L1_WrByte(dev, index, data);
}

int8_t VL53L1X_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
  return VL53L1_WrWord(dev, index, data);
}

int8_t VL53L1X_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
  return VL53L1_WrDWord(dev, index, data);
}

int8_t VL53L1X_RdByte(uint16_t dev, uint16_t index, uint8_t * data)
{
  return VL53L1_RdByte(dev, index, data);
}

int8_t VL53L1X_RdWord(uint16_t dev, uint16_t index, uint16_t * data)
{
  return VL53L1_RdWord(dev, index, data);
}

int8_t VL53L1X_RdDWord(uint16_t dev, uint16_t index, uint32_t * data)
{
  return VL53L1_RdDWord(dev, index, data);
}

int8_t VL53L1X_WaitMs(uint16_t dev, int32_t wait_ms)
{
  return VL53L1_WaitMs(dev, wait_ms);
}
