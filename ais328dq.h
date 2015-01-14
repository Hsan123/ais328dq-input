/* Copyright (C) 2014 STMicroelectronics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * File Name		: ais328dq.h
 * Authors		: VMA - Volume Mems & Analog Division
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
 * Version		: V 1.0.1
 * Date			: 2012/10/07
 * Description		: AIS328DQ 3D accelerometer sensor LDD
 *
 */

#ifndef __AIS328DQ_H__
#define __AIS328DQ_H__

#define AIS328DQ_ACC_I2C_SAD_L		(0x18)
#define AIS328DQ_ACC_I2C_SAD_H		(0x19)
#define	AIS328DQ_ACC_DEV_NAME	"ais328dq_acc"



/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define AIS328DQ_ACC_FS_MASK		0x30
#define AIS328DQ_ACC_G_2G 		0x00
#define AIS328DQ_ACC_G_4G 		0x10
#define AIS328DQ_ACC_G_8G 		0x30

/* Accelerometer Sensor Operating Mode */
#define AIS328DQ_ACC_ENABLE		0x01
#define AIS328DQ_ACC_DISABLE		0x00
#define AIS328DQ_ACC_PM_NORMAL		0x20
#define AIS328DQ_ACC_PM_OFF		AIS328DQ_ACC_DISABLE




#ifdef __KERNEL__
struct ais328dq_acc_platform_data {

	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
};


#endif /* __KERNEL__ */

#endif  /* __AIS328DQ_H__ */
