
#ifndef __drv_lsm6dsl_H
#define __drv_lsm6dsl_H

#include "drv_lsm6dsl.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "lsm6dsl.h"

#define LSM6DSL_ACC_SCALE 8
#define LSM6DSL_ACC_SAMPLERATE 3300.0f
#define LSM6DSL_LPF_A LSM6DSL_XL_LOW_NOISE_LP_ODR_DIV_100

#define LSM6DSL_GYO_SCALE 1000
#define LSM6DSL_GYO_SAMPLERATE 1600.0f
#define LSM6DSL_LPF_G LSM6DSL_HP_DISABLE_LP1_LIGHT



extern int32_t lsm6dsl_IMU_init(void);
extern LSM6DSL_Object_t obj_lsm6dsl;







#endif /*__ pinoutConfig_H */

