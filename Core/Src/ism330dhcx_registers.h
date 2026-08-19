/*
 * ism330dhcx_registers.h
 *
 *  Created on: Aug 19, 2026
 *      Author: alimi
 */

#ifndef SRC_ISM330DHCX_REGISTERS_H_
#define SRC_ISM330DHCX_REGISTERS_H_


/* Register addresses */
#define ISM330DHCX_WHO_AM_I_REG             	0x0FU
#define ISM330DHCX_CTRL1_XL_REG             	0x10U
#define ISM330DHCX_CTRL2_G_REG              	0x11U
#define ISM330DHCX_CTRL3_C_REG              	0x12U

/* CTRL1_XL fields */
#define ISM330DHCX_ODR_MASK        				0xF0U
#define ISM330DHCX_ACCEL_RANGE_MASK      		0x0CU

#define ISM330DHCX_ACCEL_CONFIG_MASK \
    (ISM330DHCX_ODR_MASK | \
     ISM330DHCX_ACCEL_RANGE_MASK)

/* CTRL2_G fields */
#define ISM330DHCX_GYRO_RANGE_MASK       		0x0FU

#define ISM330DHCX_GYRO_CONFIG_MASK \
    (ISM330DHCX_ODR_MASK | \
     ISM330DHCX_GYRO_RANGE_MASK)

/* CTRL3_C fields */
#define ISM330DHCX_CTRL3_C_BDU_MASK         	0x40U
#define ISM330DHCX_CTRL3_C_IF_INC_MASK      	0x04U
#define ISM330DHCX_CTRL3_C_RESET_MASK       	0x01U


//ACCEL power mode CTRL6_C fields
#define ISM330DHCX_CTRL6_C_REG 					0x15U
#define ISM330DHCX_CTRL6_C_XL_HM_MODE_MASK		0x10U

//Gyro Power model CTRL7_C fields
#define ISM330DHCX_CTRL7_C_REG 					0x16U
#define ISM330DHCX_CTRL7_C_G_HM_MODE_MASK 		0x80U

//I2C Data Register
#define ISM330DHCX_REG_OUTX_L_G          		0x22U
#define ISM330DHCX_SAMPLE_BYTE_COUNT     		12U

#endif /* SRC_ISM330DHCX_REGISTERS_H_ */
