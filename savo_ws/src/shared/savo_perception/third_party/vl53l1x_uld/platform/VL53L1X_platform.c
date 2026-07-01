#include "VL53L1X_platform.h"

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

static int vl53l1x_i2c_bus = 1;

void VL53L1X_SetI2CBus(int bus)
{
  if (bus >= 0) {
    vl53l1x_i2c_bus = bus;
  }
}

static int open_device(uint16_t dev)
{
  char path[32];
  const int length = snprintf(path, sizeof(path), "/dev/i2c-%d", vl53l1x_i2c_bus);
  if (length <= 0 || (size_t)length >= sizeof(path)) {
    return -1;
  }

  const int fd = open(path, O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    return -1;
  }

  const unsigned long address_7bit = (unsigned long)(dev >> 1U);
  if (ioctl(fd, I2C_SLAVE, address_7bit) < 0) {
    close(fd);
    return -1;
  }

  return fd;
}

int8_t VL53L1X_WriteMulti(
  uint16_t dev,
  uint16_t index,
  uint8_t * pdata,
  uint32_t count)
{
  if (pdata == NULL || count == 0U || count > (uint32_t)(SIZE_MAX - 2U)) {
    return -1;
  }

  uint8_t * buffer = (uint8_t *)malloc((size_t)count + 2U);
  if (buffer == NULL) {
    return -1;
  }

  buffer[0] = (uint8_t)(index >> 8U);
  buffer[1] = (uint8_t)(index & 0xFFU);
  memcpy(buffer + 2, pdata, count);

  const int fd = open_device(dev);
  if (fd < 0) {
    free(buffer);
    return -1;
  }

  const size_t bytes_to_write = (size_t)count + 2U;
  const ssize_t written = write(fd, buffer, bytes_to_write);
  const int saved_errno = errno;
  close(fd);
  free(buffer);
  errno = saved_errno;

  return written == (ssize_t)bytes_to_write ? 0 : -1;
}

int8_t VL53L1X_ReadMulti(
  uint16_t dev,
  uint16_t index,
  uint8_t * pdata,
  uint32_t count)
{
  if (pdata == NULL || count == 0U) {
    return -1;
  }

  const int fd = open_device(dev);
  if (fd < 0) {
    return -1;
  }

  const uint8_t register_address[2] = {
    (uint8_t)(index >> 8U),
    (uint8_t)(index & 0xFFU),
  };

  if (write(fd, register_address, sizeof(register_address)) !=
    (ssize_t)sizeof(register_address))
  {
    close(fd);
    return -1;
  }

  const ssize_t received = read(fd, pdata, (size_t)count);
  close(fd);
  return received == (ssize_t)count ? 0 : -1;
}

int8_t VL53L1X_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
  return VL53L1X_WriteMulti(dev, index, &data, 1U);
}

int8_t VL53L1X_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
  uint8_t bytes[2] = {
    (uint8_t)(data >> 8U),
    (uint8_t)(data & 0xFFU),
  };
  return VL53L1X_WriteMulti(dev, index, bytes, 2U);
}

int8_t VL53L1X_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
  uint8_t bytes[4] = {
    (uint8_t)(data >> 24U),
    (uint8_t)(data >> 16U),
    (uint8_t)(data >> 8U),
    (uint8_t)(data & 0xFFU),
  };
  return VL53L1X_WriteMulti(dev, index, bytes, 4U);
}

int8_t VL53L1X_RdByte(uint16_t dev, uint16_t index, uint8_t * data)
{
  return VL53L1X_ReadMulti(dev, index, data, 1U);
}

int8_t VL53L1X_RdWord(uint16_t dev, uint16_t index, uint16_t * data)
{
  uint8_t bytes[2];
  if (data == NULL || VL53L1X_ReadMulti(dev, index, bytes, 2U) != 0) {
    return -1;
  }
  *data = ((uint16_t)bytes[0] << 8U) | (uint16_t)bytes[1];
  return 0;
}

int8_t VL53L1X_RdDWord(uint16_t dev, uint16_t index, uint32_t * data)
{
  uint8_t bytes[4];
  if (data == NULL || VL53L1X_ReadMulti(dev, index, bytes, 4U) != 0) {
    return -1;
  }
  *data = ((uint32_t)bytes[0] << 24U) |
    ((uint32_t)bytes[1] << 16U) |
    ((uint32_t)bytes[2] << 8U) |
    (uint32_t)bytes[3];
  return 0;
}

int8_t VL53L1X_WaitMs(uint16_t dev, int32_t wait_ms)
{
  (void)dev;
  if (wait_ms < 0) {
    return -1;
  }

  struct timespec delay;
  delay.tv_sec = wait_ms / 1000;
  delay.tv_nsec = (long)(wait_ms % 1000) * 1000000L;

  while (nanosleep(&delay, &delay) != 0) {
    if (errno != EINTR) {
      return -1;
    }
  }
  return 0;
}
