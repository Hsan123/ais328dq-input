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
 *
 * File Name		: ais328dq_acc.c
 * Authors		: VMA - Volume Mems & Analog Division
 *			: Matteo Dameno (matteo.dameno@st.com)
 *			: Author is willing to be considered the contact
 *			: and update point for the driver.
 * Version		: V 1.0.1
 * Date			: 2012/10/07
 * Description		: AIS328DQ 3D accelerometer sensor LDD
 *
 ******************************************************************************
 * Revision 1.0.0 2011/03/02
 *	first release
 * Revision 1.0.1 2012/10/07
 *	corrects default permissions on sysfs files
 *****************************************************************************/

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

#include <linux/input/ais328dq.h>
//#include "ais328dq.h"

//#define	DEBUG	1

#define G_MAX		8000

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/

#define AXISDATA_REG		0x28
#define WHOAMI_AIS328DQ_ACC	0x32	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS ADDRESSES	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define CTRL_REG1		0x20	/*				*/
#define CTRL_REG2		0x21	/*				*/
#define CTRL_REG3		0x22	/*				*/
#define CTRL_REG4		0x23	/*				*/
#define	CTRL_REG5		0x24	/*				*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/

#define	INT_CFG2		0x34	/*	interrupt 2 config	*/
#define	INT_SRC2		0x35	/*	interrupt 2 source	*/
#define	INT_THS2		0x36	/*	interrupt 2 threshold	*/
#define	INT_DUR2		0x37	/*	interrupt 2 duration	*/
/*	end CONTROL REGISTRES ADDRESSES	*/

#define AIS328DQ_ACC_ENABLE_ALL_AXES	0x07
#define AIS328DQ_SELFTEST_EN		0x02
#define AIS328DQ_SELFTEST_DIS		0x00
#define AIS328DQ_SELFTEST_POS		0x00
#define AIS328DQ_SELFTEST_NEG		0x08

#define AIS328DQ_ACC_BDU_EN		0x80

/* Accelerometer output data rate  */
#define AIS328DQ_ACC_ODRHALF		0x40	/* 0.5Hz output data rate */
#define AIS328DQ_ACC_ODR1		0x60	/* 1Hz output data rate */
#define AIS328DQ_ACC_ODR2		0x80	/* 2Hz output data rate */
#define AIS328DQ_ACC_ODR5		0xA0	/* 5Hz output data rate */
#define AIS328DQ_ACC_ODR10		0xC0	/* 10Hz output data rate */
#define AIS328DQ_ACC_ODR50		0x00	/* 50Hz output data rate */
#define AIS328DQ_ACC_ODR100		0x08	/* 100Hz output data rate */
#define AIS328DQ_ACC_ODR400		0x10	/* 400Hz output data rate */
#define AIS328DQ_ACC_ODR1000		0x18	/* 1000Hz output data rate */

#define FUZZ			0
#define FLAT			0
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5
#define I2C_AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_REFERENCE		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8
#define	RES_INT_CFG2		9
#define	RES_INT_THS2		10
#define	RES_INT_DUR2		11

#define	RESUME_ENTRIES		12
/* end RESUME STATE INDICES */


static struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} ais328dq_acc_odr_table[] = {
	{1, AIS328DQ_ACC_PM_NORMAL | AIS328DQ_ACC_ODR1000},
	{3, AIS328DQ_ACC_PM_NORMAL | AIS328DQ_ACC_ODR400},
	{10, AIS328DQ_ACC_PM_NORMAL | AIS328DQ_ACC_ODR100},
	{20, AIS328DQ_ACC_PM_NORMAL | AIS328DQ_ACC_ODR50},
	/* low power settings, max low pass filter cut-off freq */
	{100, AIS328DQ_ACC_ODR10 | AIS328DQ_ACC_ODR1000},
	{200, AIS328DQ_ACC_ODR5 | AIS328DQ_ACC_ODR1000},
	{5000, AIS328DQ_ACC_ODR2 | AIS328DQ_ACC_ODR1000 },
	{1000, AIS328DQ_ACC_ODR1 | AIS328DQ_ACC_ODR1000 },
	{2000, AIS328DQ_ACC_ODRHALF | AIS328DQ_ACC_ODR1000 },
};

struct ais328dq_acc_data {
	struct i2c_client *client;
	struct ais328dq_acc_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	int selftest_enabled;

	atomic_t enabled;
	int on_before_suspend;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

#ifdef DEBUG
	u8 reg_addr;
#endif
};


static int ais328dq_acc_i2c_read(struct ais328dq_acc_data *acc,
				  u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
		 .flags = acc->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = acc->client->addr,
		 .flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		return -EIO;
	}
	return 0;
}

static int ais328dq_acc_i2c_write(struct ais328dq_acc_data *acc,
				   u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
		 .flags = acc->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		return -EIO;
	}
	return err;
}

static int ais328dq_acc_hw_init(struct ais328dq_acc_data *acc)
{
	int err = -1;
	u8 buf[6];

	printk(KERN_INFO "%s: hw init start\n", AIS328DQ_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = ais328dq_acc_i2c_read(acc, buf, 1);
	if (err < 0){
		dev_warn(&acc->client->dev, "Error reading WHO_AM_I: is device "
			"available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_AIS328DQ_ACC) {
		dev_err(&acc->client->dev,
			"device unknown. Expected: 0x%x,"
			" Replies: 0x%x\n", WHOAMI_AIS328DQ_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = CTRL_REG1;
	buf[1] = acc->resume_state[RES_CTRL_REG1];
	err = ais328dq_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | INT_THS1);
	buf[1] = acc->resume_state[RES_INT_THS1];
	buf[2] = acc->resume_state[RES_INT_DUR1];
	err = ais328dq_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = acc->resume_state[RES_INT_CFG1];
	err = ais328dq_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | INT_THS2);
	buf[1] = acc->resume_state[RES_INT_THS2];
	buf[2] = acc->resume_state[RES_INT_DUR2];
	err = ais328dq_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG2;
	buf[1] = acc->resume_state[RES_INT_CFG2];
	err = ais328dq_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | CTRL_REG2);
	buf[1] = acc->resume_state[RES_CTRL_REG2];
	buf[2] = acc->resume_state[RES_CTRL_REG3];
	buf[3] = acc->resume_state[RES_CTRL_REG4];
	buf[4] = acc->resume_state[RES_CTRL_REG5];
	err = ais328dq_acc_i2c_write(acc, buf, 4);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	printk(KERN_INFO "%s: hw init done\n", AIS328DQ_ACC_DEV_NAME);
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void ais328dq_acc_device_power_off(struct ais328dq_acc_data *acc)
{
	int err;
	u8 buf[2] = { CTRL_REG1, AIS328DQ_ACC_PM_OFF };

	err = ais328dq_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->power_off) {
		if(acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}

}

static int ais328dq_acc_device_power_on(struct ais328dq_acc_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on();
		if (err < 0) {
			dev_err(&acc->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}

	if (!acc->hw_initialized) {
		err = ais328dq_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			ais328dq_acc_device_power_off(acc);
			return err;
		}
	}

	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}
	return 0;
}

static irqreturn_t ais328dq_acc_isr1(int irq, void *dev)
{
	struct ais328dq_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);
	printk(KERN_INFO "%s: isr1 queued\n", AIS328DQ_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t ais328dq_acc_isr2(int irq, void *dev)
{
	struct ais328dq_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	printk(KERN_INFO "%s: isr2 queued\n", AIS328DQ_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void ais328dq_acc_irq1_work_func(struct work_struct *work)
{

	struct ais328dq_acc_data *acc =
	container_of(work, struct ais328dq_acc_data, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:ais328dq_acc_get_int1_source(acc); */
	;
	/*  */
	printk(KERN_INFO "%s: IRQ1 triggered\n", AIS328DQ_ACC_DEV_NAME);
exit:
	enable_irq(acc->irq1);
}

static void ais328dq_acc_irq2_work_func(struct work_struct *work)
{

	struct ais328dq_acc_data *acc =
	container_of(work, struct ais328dq_acc_data, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:ais328dq_acc_get_tap_source(acc); */
	;
	/*  */

	printk(KERN_INFO "%s: IRQ2 triggered\n", AIS328DQ_ACC_DEV_NAME);
exit:
	enable_irq(acc->irq2);
}

int ais328dq_acc_update_g_range(struct ais328dq_acc_data *acc, u8 new_g_range)
{
	int err = -1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = AIS328DQ_ACC_FS_MASK;

	switch (new_g_range) {
	case AIS328DQ_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case AIS328DQ_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case AIS328DQ_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid g range requested: %u\n",
				new_g_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		buf[0] = CTRL_REG4;
		err = ais328dq_acc_i2c_read(acc, buf, 1);
		if (err < 0)
			goto error;
		init_val = buf[0];
		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = CTRL_REG4;
		err = ais328dq_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;
	}


	return err;
error:
	dev_err(&acc->client->dev, "update g range failed 0x%x,0x%x: %d\n",
			buf[0], buf[1], err);

	return err;
}

int ais328dq_acc_update_odr(struct ais328dq_acc_data *acc,
							int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(ais328dq_acc_odr_table) - 1; i >= 0; i--) {
		if (ais328dq_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config[1] = ais328dq_acc_odr_table[i].mask;

	config[1] |= AIS328DQ_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		config[0] = CTRL_REG1;
		err = ais328dq_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG1] = config[1];
	}

	return err;

error:
	dev_err(&acc->client->dev, "update odr failed 0x%x,0x%x: %d\n",
			config[0], config[1], err);

	return err;
}



static int ais328dq_acc_register_write(struct ais328dq_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
	buf[0] = reg_address;
	buf[1] = new_value;
	err = ais328dq_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		return err;

	return err;
}

static int ais328dq_acc_register_read(struct ais328dq_acc_data *acc, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = ais328dq_acc_i2c_read(acc, buf, 1);
	return err;
}

static int ais328dq_acc_register_update(struct ais328dq_acc_data *acc,
			u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = ais328dq_acc_register_read(acc, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = ais328dq_acc_register_write(acc, buf, reg_address,
				updated_val);
	}
	return err;
}



static int ais328dq_acc_selftest(struct ais328dq_acc_data *acc, u8 enable)
{
	int err = -1;
	u8 buf[2]={0x00,0x00};
	char reg_address, mask, bit_values;

	reg_address = CTRL_REG4;
	mask = 0x0A;
	if (enable > 0)
		bit_values = AIS328DQ_SELFTEST_EN |
						AIS328DQ_SELFTEST_POS;
	else
		bit_values = AIS328DQ_SELFTEST_DIS |
						AIS328DQ_SELFTEST_POS;
	if (atomic_read(&acc->enabled)) {
		mutex_lock(&acc->lock);
		err = ais328dq_acc_register_update(acc, buf, reg_address,
				mask, bit_values);
		acc->selftest_enabled = enable;
		mutex_unlock(&acc->lock);
		if (err < 0)
			return err;
		acc->resume_state[RES_CTRL_REG4] = ((mask & bit_values) |
				( ~mask & acc->resume_state[RES_CTRL_REG4]));
	}
	return err;
}

static int ais328dq_acc_get_acceleration_data(struct ais328dq_acc_data *acc,
					       int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	acc_data[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	err = ais328dq_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;


	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		  : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		  : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		  : (hw_d[acc->pdata->axis_map_z]));

#ifdef DEBUG
	/*
		printk(KERN_INFO "%s read x=%d, y=%d, z=%d\n",
			AIS328DQ_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	*/
#endif
	return err;
}

static void ais328dq_acc_report_values(struct ais328dq_acc_data *acc,
					int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static int ais328dq_acc_enable(struct ais328dq_acc_data *acc)
{
	int err;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = ais328dq_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		schedule_delayed_work(&acc->input_work,
			msecs_to_jiffies(acc->pdata->poll_interval));
	}

	return 0;
}

static int ais328dq_acc_disable(struct ais328dq_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		ais328dq_acc_device_power_off(acc);
	}

	return 0;
}


static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	int rc = 0;

	u8 data = reg;
	rc = ais328dq_acc_i2c_read(acc, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;

}

static int write_reg(struct device *dev, const char *buf, u8 reg)
{
	int rc = 0;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	x[0] = reg;
	x[1] = val;
	rc = ais328dq_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return rc;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;
	ais328dq_acc_update_odr(acc, interval_ms);
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	char val;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	char range = 2;
	mutex_lock(&acc->lock);
	val = acc->pdata->g_range ;
	switch (val) {
	case AIS328DQ_ACC_G_2G:
		range = 2;
		break;
	case AIS328DQ_ACC_G_4G:
		range = 4;
		break;
	case AIS328DQ_ACC_G_8G:
		range = 8;
		break;
	}
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->g_range = val;
	ais328dq_acc_update_g_range(acc, val);
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		ais328dq_acc_enable(acc);
	else
		ais328dq_acc_disable(acc);

	return size;
}

static ssize_t attr_get_selftest(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int val;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->selftest_enabled;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_selftest(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	ais328dq_acc_selftest(acc, val);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_thresh1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}

static ssize_t attr_set_intconfig2(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG2);
}

static ssize_t attr_get_intconfig2(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG2);
}

static ssize_t attr_set_duration2(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR2);
}

static ssize_t attr_get_duration2(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR2);
}

static ssize_t attr_set_thresh2(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS2);
}

static ssize_t attr_get_thresh2(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THS2);
}
static ssize_t attr_get_source2(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC2);
}




#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = ais328dq_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = ais328dq_acc_i2c_read(acc, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ais328dq_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0664, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0664, attr_get_enable, attr_set_enable),
	__ATTR(enable_selftest, 0664, attr_get_selftest, attr_set_selftest),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0664, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(int2_config, 0664, attr_get_intconfig2, attr_set_intconfig2),
	__ATTR(int2_duration, 0664, attr_get_duration2, attr_set_duration2),
	__ATTR(int2_threshold, 0664, attr_get_thresh2, attr_set_thresh2),
	__ATTR(int2_source, 0444, attr_get_source2, NULL),
#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void ais328dq_acc_input_work_func(struct work_struct *work)
{
	struct ais328dq_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			    struct ais328dq_acc_data, input_work);

	mutex_lock(&acc->lock);
	err = ais328dq_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		ais328dq_acc_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work,
			      msecs_to_jiffies(acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

int ais328dq_acc_input_open(struct input_dev *input)
{
	struct ais328dq_acc_data *acc = input_get_drvdata(input);

	return ais328dq_acc_enable(acc);
}

void ais328dq_acc_input_close(struct input_dev *dev)
{
	struct ais328dq_acc_data *acc = input_get_drvdata(dev);

	ais328dq_acc_disable(acc);
}

static int ais328dq_acc_validate_pdata(struct ais328dq_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
					acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
	    acc->pdata->axis_map_y > 2 || acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			acc->pdata->axis_map_x, acc->pdata->axis_map_y,
			acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1 ||
	    acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			acc->pdata->negate_x, acc->pdata->negate_y,
			acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int ais328dq_acc_input_init(struct ais328dq_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, ais328dq_acc_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->open = ais328dq_acc_input_open;
	acc->input_dev->close = ais328dq_acc_input_close;
	acc->input_dev->name = AIS328DQ_ACC_DEV_NAME;
	acc->input_dev->id.bustype = BUS_I2C;
	acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);


	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
			"unable to register input device %s\n",
			acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void ais328dq_acc_input_cleanup(struct ais328dq_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int ais328dq_acc_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ais328dq_acc_data *acc;
	int err = -1;
	//int tempvalue;

	pr_info("%s: probe start.\n", AIS328DQ_ACC_DEV_NAME);

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	/*
	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE |
					I2C_FUNC_SMBUS_BYTE_DATA |
					I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "client not smb-i2c capable:2\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}


	if (!i2c_check_functionality(client->adapter,
						I2C_FUNC_SMBUS_I2C_BLOCK)){
		dev_err(&client->dev, "client not smb-i2c capable:3\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}
	*/

	acc = kzalloc(sizeof(struct ais328dq_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}


	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
	i2c_set_clientdata(client, acc);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}

	memcpy(acc->pdata, client->dev.platform_data, sizeof(*acc->pdata));

	err = ais328dq_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}


	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	if(acc->pdata->gpio_int1 >= 0){
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		printk(KERN_INFO "%s: %s has set irq1 to irq: %d "
							"mapped on gpio:%d\n",
		AIS328DQ_ACC_DEV_NAME, __func__, acc->irq1,
							acc->pdata->gpio_int1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		printk(KERN_INFO "%s: %s has set irq2 to irq: %d "
						"mapped on gpio:%d\n",
			AIS328DQ_ACC_DEV_NAME, __func__, acc->irq2,
							acc->pdata->gpio_int2);
	}

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[RES_CTRL_REG1] = AIS328DQ_ACC_ENABLE_ALL_AXES;
	acc->resume_state[RES_CTRL_REG2] = 0x00;
	acc->resume_state[RES_CTRL_REG3] = 0x00;
	acc->resume_state[RES_CTRL_REG4] = (0x00 | AIS328DQ_ACC_BDU_EN);
	acc->resume_state[RES_CTRL_REG5] = 0x00;
	acc->resume_state[RES_REFERENCE] = 0x00;

	acc->resume_state[RES_INT_CFG1] = 0x00;
	acc->resume_state[RES_INT_THS1] = 0x00;
	acc->resume_state[RES_INT_DUR1] = 0x00;
	acc->resume_state[RES_INT_CFG2] = 0x00;
	acc->resume_state[RES_INT_THS2] = 0x00;
	acc->resume_state[RES_INT_DUR2] = 0x00;

	err = ais328dq_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1);

	err = ais328dq_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto err_power_off;
	}

	err = ais328dq_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err_power_off;
	}

	err = ais328dq_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}


	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
			"device AIS328DQ_ACC_DEV_NAME "
						"sysfs register failed\n");
		goto err_input_cleanup;
	}


	ais328dq_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	if(acc->pdata->gpio_int1 >= 0){
		INIT_WORK(&acc->irq1_work, ais328dq_acc_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("ais328dq_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(acc->irq1, ais328dq_acc_isr1,
			IRQF_TRIGGER_RISING, "ais328dq_acc_irq1", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		INIT_WORK(&acc->irq2_work, ais328dq_acc_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("ais328dq_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
				"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, ais328dq_acc_isr2,
			IRQF_TRIGGER_RISING, "ais328dq_acc_irq2", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
				goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}



	mutex_unlock(&acc->lock);

	dev_info(&client->dev, "%s: probed\n", AIS328DQ_ACC_DEV_NAME);

	return 0;

err_destoyworkqueue2:
	if(acc->pdata->gpio_int2 >= 0)
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
	free_irq(acc->irq1, acc);
err_destoyworkqueue1:
	if(acc->pdata->gpio_int1 >= 0)
		destroy_workqueue(acc->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
	ais328dq_acc_input_cleanup(acc);
err_power_off:
	ais328dq_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
//err_freedata:
	kfree(acc);
exit_check_functionality_failed:
	printk(KERN_ERR "%s: Driver Init failed\n", AIS328DQ_ACC_DEV_NAME);
	return err;
}

static int ais328dq_acc_remove(struct i2c_client *client)
{
	struct ais328dq_acc_data *acc = i2c_get_clientdata(client);

	if(acc->pdata->gpio_int1 >= 0){
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}

	if(acc->pdata->gpio_int2 >= 0){
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}

	ais328dq_acc_input_cleanup(acc);
	ais328dq_acc_device_power_off(acc);
	remove_sysfs_interfaces(&client->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM
static int ais328dq_acc_resume(struct i2c_client *client)
{
	struct ais328dq_acc_data *acc = i2c_get_clientdata(client);

	if (acc->on_before_suspend)
		return ais328dq_acc_enable(acc);
	return 0;
}

static int ais328dq_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ais328dq_acc_data *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->enabled);
	return ais328dq_acc_disable(acc);
}
#else
#define lis3dh_acc_suspend	NULL
#define lis3dh_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id ais328dq_acc_id[]
		= { { AIS328DQ_ACC_DEV_NAME, 0}, { },};

MODULE_DEVICE_TABLE(i2c, ais328dq_acc_id);

static struct i2c_driver ais328dq_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = AIS328DQ_ACC_DEV_NAME,
	},
	.probe = ais328dq_acc_probe,
	.remove = ais328dq_acc_remove,
	.resume = ais328dq_acc_resume,
	.suspend = ais328dq_acc_suspend,
	.id_table = ais328dq_acc_id,
};

static int __init ais328dq_acc_init(void)
{
	printk(KERN_DEBUG "%s accelerometer driver: init\n",
						AIS328DQ_ACC_DEV_NAME);
	return i2c_add_driver(&ais328dq_acc_driver);
}

static void __exit ais328dq_acc_exit(void)
{
	i2c_del_driver(&ais328dq_acc_driver);
#ifdef DEBUG
	printk(KERN_DEBUG "%s accelerometer driver exit\n",
						AIS328DQ_ACC_DEV_NAME);
#endif /* DEBUG */
	return;
}

module_init(ais328dq_acc_init);
module_exit(ais328dq_acc_exit);

MODULE_DESCRIPTION("ais328dq accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, STMicroelectronics");
MODULE_LICENSE("GPL");

