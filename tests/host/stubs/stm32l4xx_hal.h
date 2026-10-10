#ifndef HOST_STM32L4XX_HAL_H
#define HOST_STM32L4XX_HAL_H

/* Host-only declarations; these never enter the CubeIDE firmware build. */
#include <stddef.h>
#include <stdint.h>

typedef struct I2C_HandleTypeDef I2C_HandleTypeDef;

uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);

#endif
