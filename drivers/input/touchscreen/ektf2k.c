/* drivers/input/touchscreen/ektf2k.c - ELAN EKTF2K verions of driver
 *
 * Copyright (C) 2011 Elan Microelectronics Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/ioctl.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/uaccess.h>
#if defined(CONFIG_FB)
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#define ELAN_KTF2K_NAME "elan-ktf2k-touch"

// Analog voltage @2.7 V
#define ELAN_VTG_MIN_UV 2850000
#define ELAN_VTG_MAX_UV 3300000
#define ELAN_ACTIVE_LOAD_UA 15000
#define ELAN_LPM_LOAD_UA 10

#define ELAN_I2C_VTG_MIN_UV 1800000
#define ELAN_I2C_VTG_MAX_UV 1800000
#define ELAN_I2C_LOAD_UA 10000
#define ELAN_I2C_LPM_LOAD_UA 10

// Support 2 fingers packet
#define PACKET_SIZE 8

#define TOUCH_2127 0x21
#define TOUCH_2227 0x22

#define CMD_S_PKT 0x52
#define CMD_R_PKT 0x53
#define CMD_W_PKT 0x54
#define HELLO_PKT 0x55
#define TWO_FINGERS_PKT 0x5A
#define FIVE_FINGERS_PKT 0x5D
#define MTK_FINGERS_PKT 0x6D
#define TEN_FINGERS_PKT 0x62
#define BUFFER_PKT 0x63
#define CALIB_PKT 0x66

#define SYSTEM_RESET_PIN_SR 0
#define PAGERETRY 30
#define IAPRESTART 5
#define PAGENUM 249

// For Firmware Update
#define ELAN_IOCTLID 0xD0
#define IOCTL_I2C_SLAVE _IOW(ELAN_IOCTLID, 1, int)
#define IOCTL_MAJOR_FW_VER _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_FW_X_RESOLUTION _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_FW_Y_RESOLUTION _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_POWER_LOCK _IOR(ELAN_IOCTLID, 15, int)
#define IOCTL_POWER_UNLOCK _IOR(ELAN_IOCTLID, 16, int)
#define IOCTL_FW_UPDATE _IOR(ELAN_IOCTLID, 17, int)

#define CUSTOMER_IOCTLID 0xA0
#define IOCTL_CIRCUIT_CHECK _IOR(CUSTOMER_IOCTLID, 1, int)
#define IOCTL_GET_UPDATE_PROGREE _IOR(CUSTOMER_IOCTLID, 2, int)

/*
 * Shuang Board X/Y resolution
 */
int LCM_X_RESOLUTION = 480;
int LCM_Y_RESOLUTION = 800;

// Initialize global variables
uint8_t RECOVERY = 0x00;
uint8_t chip_type = 0x00;

int FW_ID = 0;
int FW_VERSION = 0;
int FW_X_RESOLUTION = 0;
int FW_Y_RESOLUTION = 0;
int power_lock = 0;
int update_progree = 0;
int work_lock = 0;

static int touch_panel_type;
static unsigned short chip_reset_flag = 0;
static unsigned long chip_mode_set = 0;
static unsigned long talking_mode_set = 0;

uint8_t I2C_DATA[3] = {0x15, 0x20, 0x21}; /* I2C devices address */

/*The newest firmware, if update must be changed here*/
static uint8_t file_fw_data_Truly_2127[] = {
#include "Truly_E1_V5507_BC2104_20140121.i"
};
static uint8_t file_fw_data_Eely_2127[] = {
#include "Eely_E1_V5507_BC2104_20140121.i"
};
static uint8_t file_fw_data_Truly[] = {
#include "Truly_IN1_V17_BC5568_20131119_RAM.i"
};
static uint8_t file_fw_data_Eely[] = {
#include "Eely_IN1_V17_BC5568_20131119_RAM.i"
};

static uint8_t *file_fw_data = NULL;
static uint8_t tp_sleep_status = 0;

enum { ACK_Fail = 0x00,
       ACK_OK = 0xAA,
       ACK_REWRITE = 0x55,
};

struct elan_ktf2k_i2c_platform_data {
	uint16_t version;
	int intr_gpio;
	int reset_gpio;
	int hw_det_gpio;

	u32 hw_det_gpio_flags;
	u32 reset_gpio_flags;
	u32 intr_gpio_flags;

	bool i2c_pull_up;
};

struct elan_ktf2k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	struct delayed_work check_work;
	struct elan_ktf2k_i2c_platform_data *pdata;
	struct regulator *vcc_ana;
	struct regulator *vcc_i2c;
	struct mutex lock;

	int intr_gpio;
	int reset_gpio;
	int fw_ver;
	int fw_id;
	int bc_ver;
	int x_resolution;
	int y_resolution;

	struct miscdevice firmware;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
};

static struct elan_ktf2k_ts_data *private_ts;
static struct kobject *android_touch_kobj;

static int elan_ktf2k_ts_parse_dt(struct device *,
				  struct elan_ktf2k_i2c_platform_data *);
static int elan_ktf2k_ts_set_mode_state(struct i2c_client *client, int mode);
static int elan_ktf2k_ts_get_mode_state(struct i2c_client *client);
static int elan_ktf2k_ts_set_talking_state(struct i2c_client *client, int mode);
static int elan_ktf2k_ts_get_talking_state(struct i2c_client *client);
static int elan_ktf2k_set_scan_mode(struct i2c_client *client, int mode);
static int elan_ktf2k_ts_setup(struct i2c_client *client);
static int elan_ktf2k_ts_hw_reset(struct i2c_client *client);
static int elan_ktf2k_ts_poll(struct i2c_client *client);
static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client);
static int elan_ktf2k_ts_resume(struct device *dev);
static int elan_ktf2k_ts_suspend(struct device *dev);

static int __hello_packet_handler(struct i2c_client *client);
static int __fw_packet_handler(struct i2c_client *client);

int Update_FW_One(struct i2c_client *client, int recovery);

// For Firmware Update
int elan_iap_open(struct inode *inode, struct file *filp) { return 0; }

int elan_iap_release(struct inode *inode, struct file *filp) { return 0; }

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count,
			      loff_t *offp)
{
	int ret;
	char *tmp;

	pr_info("%s: Enter\n", __func__);

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	if (copy_from_user(tmp, buff, count))
		return -EFAULT;

	ret = i2c_master_send(private_ts->client, tmp, count);
	kfree(tmp);

	return (ret == 1) ? count : ret;
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	char *tmp;
	int ret;
	long rc;

	pr_info("%s: Enter\n", __func__);

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	ret = i2c_master_recv(private_ts->client, tmp, count);
	if (ret >= 0)
		rc = copy_to_user(buff, tmp, count);

	kfree(tmp);

	return (ret == 1) ? count : ret;
}

static long elan_iap_ioctl(struct file *filp, unsigned int cmd,
			   unsigned long arg)
{
	int __user *ip = (int __user *)arg;
	pr_info("%s: Enter\n", __func__);
	pr_info("%s: cmd value %x\n", __func__, cmd);

	switch (cmd) {
	case IOCTL_I2C_SLAVE:
		private_ts->client->addr = (int __user)arg;
		pr_info("%s: IOCTL_I2C_SLAVE = [0x%x]\n", __func__,
			private_ts->client->addr);
		break;
	case IOCTL_MAJOR_FW_VER:
		break;
	case IOCTL_MINOR_FW_VER:
		break;
	case IOCTL_RESET:
		gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
		msleep(20);
		gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
		usleep_range(5000, 5500);
		pr_info("%s: IOCTL_RESET\n", __func__);
		break;
	case IOCTL_IAP_MODE_LOCK:
		cancel_delayed_work_sync(&private_ts->check_work);
		if (work_lock == 0) {
			work_lock = 1;
			disable_irq(private_ts->client->irq);
			flush_work(&private_ts->work);
			pr_info("%s: IOCTL_IAP_MODE_LOCK\n", __func__);
		}
		break;
	case IOCTL_IAP_MODE_UNLOCK:
		if (work_lock == 1) {
			work_lock = 0;
			enable_irq(private_ts->client->irq);
			pr_info("%s: IOCTL_IAP_MODE_UNLOCK\n", __func__);
		}
		schedule_delayed_work(&private_ts->check_work,
				      msecs_to_jiffies(2500));
		break;
	case IOCTL_CHECK_RECOVERY_MODE:
		return RECOVERY;
		break;
	case IOCTL_FW_VER:
		__fw_packet_handler(private_ts->client);
		return FW_VERSION;
		break;
	case IOCTL_FW_X_RESOLUTION:
		__fw_packet_handler(private_ts->client);
		return FW_X_RESOLUTION;
		break;
	case IOCTL_FW_Y_RESOLUTION:
		__fw_packet_handler(private_ts->client);
		return FW_Y_RESOLUTION;
		break;
	case IOCTL_FW_ID:
		__fw_packet_handler(private_ts->client);
		return FW_ID;
		break;
	case IOCTL_ROUGH_CALIBRATE:
		return elan_ktf2k_ts_rough_calibrate(private_ts->client);
	case IOCTL_I2C_INT:
		put_user(gpio_get_value(private_ts->intr_gpio), ip);
		break;
	case IOCTL_RESUME:
		elan_ktf2k_ts_resume(&(private_ts->client->dev));
		break;
	case IOCTL_POWER_LOCK:
		power_lock = 1;
		break;
	case IOCTL_POWER_UNLOCK:
		power_lock = 0;
		break;
	case IOCTL_GET_UPDATE_PROGREE:
		update_progree = (int __user)arg;
		break;
	case IOCTL_FW_UPDATE:
		Update_FW_One(private_ts->client, 0);
		break;
	case IOCTL_CIRCUIT_CHECK:
		return 1;
		break;
	default:
		pr_err("%s: Un-known IOCTL Command %d\n", __func__, cmd);
		break;
	}

	return 0;
}

struct file_operations elan_touch_fops = {
    .open = elan_iap_open,
    .write = elan_iap_write,
    .read = elan_iap_read,
    .release = elan_iap_release,
    .unlocked_ioctl = elan_iap_ioctl,
};

int EnterISPMode(struct i2c_client *client, uint8_t *isp_cmd)
{
	char buff[4] = {0};
	int len = 0;

	len = i2c_master_send(private_ts->client, isp_cmd, sizeof(isp_cmd));
	if (len != sizeof(buff)) {
		dev_err(&client->dev, "%s: Fail! len = [%d]\r\n", __func__,
			len);
		return -1;
	} else
		dev_info(&client->dev,
			 "%s: IAPMode write data successfully! cmd = [%2x, "
			 "%2x, %2x, %2x]\n",
			 __func__, isp_cmd[0], isp_cmd[1], isp_cmd[2],
			 isp_cmd[3]);

	return 0;
}

int ExtractPage(struct file *filp, uint8_t *szPage, int byte)
{
	int len = 0;

	len = filp->f_op->read(filp, szPage, byte, &filp->f_pos);
	if (len != byte) {
		pr_err("%s: Read page error, read error. len = [%d]\r\n",
		       __func__, len);
		return -1;
	}

	return 0;
}

int WritePage(uint8_t *szPage, int byte)
{
	int len = 0;

	len = i2c_master_send(private_ts->client, szPage, byte);
	if (len != byte) {
		pr_err("%s: write page error, write error. len = [%d]\r\n",
		       __func__, len);
		return -1;
	}

	return 0;
}

int GetAckData(struct i2c_client *client)
{
	int len = 0;

	char buff[2] = {0};

	len = i2c_master_recv(private_ts->client, buff, sizeof(buff));
	if (len != sizeof(buff)) {
		dev_err(&client->dev,
			"%s: read data error, write 50 times error. "
			"len = [%d]\r\n",
			__func__, len);
		return -1;
	}

	dev_info(&client->dev, "%s: GetAckData = [%x,%x]\n", __func__, buff[0],
		 buff[1]);
	if (buff[0] == 0xAA)
		return ACK_OK;
	else if (buff[0] == 0x55 && buff[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;

	return 0;
}

void print_progress(int page, int ic_num, int j)
{
	int i, percent, page_tatol, percent_tatol;
	char str[256];
	str[0] = '\0';
	for (i = 0; i < ((page) / 10); i++) {
		str[i] = '#';
		str[i + 1] = '\0';
	}

	page_tatol = page + 249 * (ic_num - j);
	percent = ((100 * page) / (249));
	percent_tatol = ((100 * page_tatol) / (249 * ic_num));

	if ((page) == (249))
		percent = 100;

	if ((page_tatol) == (249 * ic_num))
		percent_tatol = 100;

	pr_info("\rprogress %s | %d%%", str, percent);

	if (page == (249))
		pr_info("\n");
}

/*
 * Reset and (Send normal_command ???)
 * Get Hello Packet
 */
int Update_FW_One(struct i2c_client *client, int recovery)
{
	int res = 0, ic_num = 1;
	int iPage = 0, rewriteCnt = 0; // rewriteCnt for PAGE_REWRITE
	int i = 0;
	uint8_t data;
	int restartCnt = 0; // For IAP_RESTART

	uint8_t recovery_buffer[8] = {0};
	int byte_count;
	uint8_t *szBuff = NULL;
	int curIndex = 0;
	uint8_t isp_cmd_2227[] = {0x54, 0x00, 0x12, 0x34}; // for 2227e
	uint8_t isp_cmd_2127[] = {0x45, 0x49, 0x41, 0x50}; // for 2127e

	dev_dbg(&client->dev, "%s:  ic_num = [%d]\n", __func__, ic_num);
IAP_RESTART:
	data = I2C_DATA[0];
	dev_dbg(&client->dev, "%s: address data = [0x%x]\r\n", __func__, data);

	if (recovery != 0x80) {
		dev_info(&client->dev, "%s: Firmware upgrade normal mode!\n",
			 __func__);
		gpio_set_value(SYSTEM_RESET_PIN_SR, 0);
		mdelay(20);
		gpio_set_value(SYSTEM_RESET_PIN_SR, 1);
		mdelay(5);
		if (chip_type == TOUCH_2227) {
			// detect the status of int pin
			if (elan_ktf2k_ts_poll(private_ts->client) < 0)
				goto IAP_RESTART; // means poll fail

			res = i2c_master_recv(private_ts->client,
					      recovery_buffer, 8);
			// enter 2227e IAP mode
			res = EnterISPMode(private_ts->client, isp_cmd_2227);
		} else
			// enter 2127e IAP mode
			res = EnterISPMode(private_ts->client, isp_cmd_2127);
	} else
		dev_info(&client->dev, "%s: Firmware upgrade recovery mode!\n",
			 __func__);

	// 55 aa 33 cc
	res = i2c_master_recv(private_ts->client, recovery_buffer, 4);
	dev_info(&client->dev, "%s: recovery byte data = [%x,%x,%x,%x]\n",
		 __func__, recovery_buffer[0], recovery_buffer[1],
		 recovery_buffer[2], recovery_buffer[3]);

	// if (buffer[0] != aa), means we do not enter IAP mode
	if (recovery_buffer[1] != 0xaa && restartCnt < 5) {
		restartCnt++;
		goto IAP_RESTART;
	} else if (restartCnt >= 5) {
		dev_err(&client->dev, "%s: IAP fail!!!\n", __func__);
		return 0;
	}

	// Send Dummy Byte
	dev_info(&client->dev, "%s: send one byte data = [%x,%x]\n", __func__,
		 private_ts->client->addr, data);

	res = i2c_master_send(private_ts->client, &data, sizeof(data));
	if (res != sizeof(data))
		dev_err(&client->dev, "%s: dummy error code = %d\n", __func__,
			res);

	usleep_range(10000, 10500);

	// Start IAP
	for (iPage = 1; iPage <= PAGENUM; iPage++) {
	PAGE_REWRITE:
		// 8 bytes mode
		for (byte_count = 1; byte_count <= 17; byte_count++) {
			if (byte_count != 17) {
				szBuff = file_fw_data + curIndex;
				curIndex = curIndex + 8;
				res = WritePage(szBuff, 8);
			} else {
				szBuff = file_fw_data + curIndex;
				curIndex = curIndex + 4;
				res = WritePage(szBuff, 4);
			}
		} // end for

		if (iPage == 249 || iPage == 1)
			msleep(600);
		else
			msleep(50);

		res = GetAckData(private_ts->client);
		if (ACK_OK != res) {
			msleep(50);
			dev_err(&client->dev,
				"%s: GetAckData fail! res = [%d]\r\n", __func__,
				res);
			if (res == ACK_REWRITE) {
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY) {
					dev_err(&client->dev,
						"%s: ID 0x%02x %dth page "
						"ReWrite %d times fails!\n",
						__func__, data, iPage,
						PAGERETRY);
					return -1;
				} else {
					dev_info(&client->dev,
						 "%s: %d page ReWrite "
						 "%d times!\n",
						 __func__, iPage, rewriteCnt);
					goto PAGE_REWRITE;
				}
			} else {
				restartCnt = restartCnt + 1;
				if (restartCnt >= IAPRESTART) {
					dev_err(&client->dev,
						"%s: ID 0x%02x ReStart %d "
						"times fails!\n",
						__func__, data, IAPRESTART);
					return -1;
				} else {
					dev_info(&client->dev,
						 "%s: %d page ReStart "
						 "%d times!\n",
						 __func__, iPage, restartCnt);
					goto IAP_RESTART;
				}
			}
		} else {
			dev_info(&client->dev, "%s: data: 0x%02x ", __func__,
				 data);
			rewriteCnt = 0;
			print_progress(iPage, ic_num, i);
		}

		usleep_range(10000, 10500);
	} // end for

	dev_info(&client->dev, "%s: Read Hello packet data!\n", __func__);

	res = __hello_packet_handler(client);
	if (res > 0)
		dev_info(&client->dev,
			 "%s: Update ALL Firmware successfully!\n", __func__);

	return 0;
}
// End Firmware Update

// Start sysfs
static ssize_t elan_ktf2k_gpio_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	ret = gpio_get_value(ts->intr_gpio);
	sprintf(buf, "%s: GPIO_TP_INT_N = [%d]\n", __func__, ret);

	ret = strlen(buf) + 1;
	return ret;
}

static ssize_t elan_ktf2k_vendor_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct elan_ktf2k_ts_data *ts = private_ts;

	sprintf(buf, "%s_x%4.4x\n", "ELAN_KTF2K", ts->fw_ver);

	ret = strlen(buf) + 1;
	return ret;
}

static ssize_t elan_ktf2k_mode_set(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	if (strict_strtoul(buf, 10, &chip_mode_set))
		return -EINVAL;

	pr_info("%s: chip_mode_set = [%lu]\n", __func__, chip_mode_set);

	// if chip in the sleep mode, we do not need to do it
	if (tp_sleep_status == 0)
		return count;

	// if there is no exist work
	disable_irq(private_ts->client->irq);
	flush_work(&private_ts->work);
	cancel_delayed_work_sync(&private_ts->check_work);

	mutex_lock(&private_ts->lock);
	if (chip_type == TOUCH_2227)
		elan_ktf2k_set_scan_mode(private_ts->client, 0);

	elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);
	if (elan_ktf2k_ts_get_mode_state(private_ts->client) != chip_mode_set)
		elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);

	if (chip_type == TOUCH_2227) {
		usleep_range(10000, 10500);
		elan_ktf2k_set_scan_mode(private_ts->client, 1);
	}
	mutex_unlock(&private_ts->lock);

	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
	enable_irq(private_ts->client->irq);

	return count;
}

static ssize_t elan_ktf2k_mode_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	static unsigned long chip_mode_get;
	ssize_t ret = 0;

	chip_mode_get = elan_ktf2k_ts_get_mode_state(private_ts->client);

	ret = snprintf(buf, PAGE_SIZE, "%lu\n", chip_mode_get);
	return ret;
}

static ssize_t elan_ktf2k_talking_set(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	if (strict_strtoul(buf, 10, &talking_mode_set))
		return -EINVAL;

	pr_info("%s: talking_mode_set = [%lu]\n", __func__, talking_mode_set);

	// if chip in the sleep mode, we do not need to do it
	if (tp_sleep_status == 0)
		return count;

	// if there is no exist work
	disable_irq(private_ts->client->irq);
	flush_work(&private_ts->work);
	cancel_delayed_work_sync(&private_ts->check_work);

	mutex_lock(&private_ts->lock);
	if (chip_type == TOUCH_2227)
		elan_ktf2k_set_scan_mode(private_ts->client, 0);

	elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);
	if (elan_ktf2k_ts_get_talking_state(private_ts->client) !=
	    talking_mode_set)
		elan_ktf2k_ts_set_talking_state(private_ts->client,
						talking_mode_set);

	if (chip_type == TOUCH_2227) {
		usleep_range(10000, 10500);
		elan_ktf2k_set_scan_mode(private_ts->client, 1);
	}
	mutex_unlock(&private_ts->lock);

	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
	enable_irq(private_ts->client->irq);

	return count;
}

static ssize_t elan_ktf2k_talking_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	static unsigned long talking_mode_get;
	ssize_t ret = 0;

	talking_mode_get = elan_ktf2k_ts_get_talking_state(private_ts->client);

	ret = snprintf(buf, PAGE_SIZE, "%lu\n", talking_mode_get);
	return ret;
}

static ssize_t elan_ktf2k_reset(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	static unsigned long chip_reset;

	if (strict_strtoul(buf, 10, &chip_reset))
		return -EINVAL;

	pr_info("%s: Enter\n", __func__);

	if (chip_reset > 0) {
		if (work_lock == 0) {
			work_lock = 1;
			disable_irq(private_ts->client->irq);
			cancel_work_sync(&private_ts->work);
		}

		elan_ktf2k_ts_hw_reset(private_ts->client);
		if (elan_ktf2k_ts_setup(private_ts->client) < 0)
			pr_err("%s: No Elan chip inside\n", __func__);

		if (work_lock == 1) {
			work_lock = 0;
			enable_irq(private_ts->client->irq);
		}
	}

	return count;
}

static ssize_t elan_ktf2k_irq_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	static unsigned long chip_irq_set;

	if (strict_strtoul(buf, 10, &chip_irq_set))
		return -EINVAL;

	pr_info("%s: Enter\n", __func__);

	if (chip_irq_set > 0)
		enable_irq(private_ts->client->irq);
	else
		disable_irq(private_ts->client->irq);

	return count;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf2k_gpio_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, elan_ktf2k_vendor_show, NULL);
static DEVICE_ATTR(mode, 0644, elan_ktf2k_mode_show, elan_ktf2k_mode_set);
static DEVICE_ATTR(hw_reset, 0644, NULL, elan_ktf2k_reset);
static DEVICE_ATTR(talking_set, 0644, elan_ktf2k_talking_show,
		   elan_ktf2k_talking_set);
static DEVICE_ATTR(set_irq, 0644, NULL, elan_ktf2k_irq_set);

static int elan_ktf2k_touch_sysfs_init(void)
{
	int ret;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		pr_err("%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		pr_err("%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		pr_err("%s: sysfs_create_group failed\n", __func__);
		return ret;
	}

	//[Arima Edison] add mode select set/get ++
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_mode.attr);
	if (ret) {
		pr_err("%s: sysfs_create_group failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_hw_reset.attr);
	if (ret) {
		pr_err("%s: sysfs_create_group failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_talking_set.attr);
	if (ret) {
		pr_err("%s: sysfs_create_group failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_set_irq.attr);
	if (ret) {
		pr_err("%s: sysfs_create_group failed\n", __func__);
		return ret;
	}
	//[Arima Edison] add mode select set/get --

	return 0;
}

static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_mode.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_hw_reset.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_talking_set.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_set_irq.attr);

	kobject_del(android_touch_kobj);
}
// end sysfs

static int __elan_ktf2k_ts_poll(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 20;

	do {
		status = gpio_get_value(ts->intr_gpio);
		retry--;
		// [Arima Edison] we do not need delay if chip work normally
		if (status == 1)
			msleep(25);
	} while (status == 1 && retry > 0);

	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf2k_ts_poll(struct i2c_client *client)
{
	return __elan_ktf2k_ts_poll(client);
}

static int elan_ktf2k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
				  uint8_t *buf, size_t size)
{
	int rc;

	dev_info(&client->dev, "%s: enter\n", __func__);

	if (buf == NULL)
		return -EINVAL;

	if ((i2c_master_send(client, cmd, 4)) != 4) {
		dev_err(&client->dev, "%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0)
		return -EINVAL;
	else {
		if (i2c_master_recv(client, buf, size) != size ||
		    buf[0] != CMD_S_PKT)
			return -EINVAL;
	}

	return 0;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[8] = {0};

	rc = elan_ktf2k_ts_poll(client);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Int poll failed!\n", __func__);
		RECOVERY = 0x80;
		return 0x88; // means we do not get irq status
	}

	rc = i2c_master_recv(client, buf_recv, 8);
	//[Arima Edison] do the receive it again ++
	if (rc < 0)
		rc = i2c_master_recv(client, buf_recv, 8);
	//[Arima Edison] do the receive it again --

	dev_info(&client->dev,
		 "%s: hello packet %2x:%2X:%2x:%2x:%2x:%2x:%2x:%2x\n", __func__,
		 buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3],
		 buf_recv[4], buf_recv[5], buf_recv[6], buf_recv[7]);

	if (buf_recv[0] == 0x55 && buf_recv[1] == 0x55 && buf_recv[2] == 0x80 &&
	    buf_recv[3] == 0x80) {
		if (buf_recv[6] == 0x04 && buf_recv[7] == TOUCH_2127)
			chip_type = TOUCH_2127;
		RECOVERY = 0x80;
		return RECOVERY;
	}

	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};    // Get Firmware ver.
	uint8_t cmd_x[] = {CMD_R_PKT, 0x60, 0x00, 0x00};  // Get x resolution
	uint8_t cmd_y[] = {CMD_R_PKT, 0x63, 0x00, 0x00};  // Get y resolution
	uint8_t cmd_id[] = {CMD_R_PKT, 0xf0, 0x00, 0x01}; // Get firmware ID
	uint8_t cmd_bc[] = {CMD_R_PKT, 0x01, 0x00, 0x01}; // Get BootCode ver.
	uint8_t buf_recv[4] = {0};

	// Firmware version
	rc = elan_ktf2k_ts_get_data(client, cmd, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;
	FW_VERSION = ts->fw_ver;

	//[Arima Edison] add to set chip type ++
	if (major == 0x55)
		chip_type = TOUCH_2127;
	else
		chip_type = TOUCH_2227;
	//[Arima Edison] add to set chip type --

	// Firmware ID
	rc = elan_ktf2k_ts_get_data(client, cmd_id, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;
	FW_ID = ts->fw_id;

	// Bootcode version
	rc = elan_ktf2k_ts_get_data(client, cmd_bc, buf_recv, 4);
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->bc_ver = major << 8 | minor;

	// X Resolution
	rc = elan_ktf2k_ts_get_data(client, cmd_x, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->x_resolution = minor;
	FW_X_RESOLUTION = ts->x_resolution;

	// Y Resolution
	rc = elan_ktf2k_ts_get_data(client, cmd_y, buf_recv, 4);
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	ts->y_resolution = minor;
	FW_Y_RESOLUTION = ts->y_resolution;

	dev_info(&client->dev, "%s: Firmware version: 0x%4.4x\n", __func__,
		 ts->fw_ver);
	dev_info(&client->dev, "%s: Firmware ID: 0x%4.4x\n", __func__,
		 ts->fw_id);
	dev_info(&client->dev, "%s: Bootcode Version: 0x%4.4x\n", __func__,
		 ts->bc_ver);
	dev_info(&client->dev, "%s: x resolution: %d, y resolution: %d\n",
		 __func__, FW_X_RESOLUTION, FW_Y_RESOLUTION);

	return 0;
}

static inline int elan_ktf2k_ts_parse_xy(uint8_t *data, uint16_t *x,
					 uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}

static int elan_ktf2k_ts_setup(struct i2c_client *client)
{
	int rc = 0;
	int touch_retry = 0;

	touch_retry = 3;

	do {
		elan_ktf2k_ts_hw_reset(client);
		rc = __hello_packet_handler(client);
		touch_retry--;
	} while (rc == 0x88 && touch_retry > 0);

	dev_info(&client->dev, "%s: First hellopacket's rc = [%d]\n", __func__,
		 rc);

	mdelay(10);

	if (rc != 0x80 && rc != 0x88) {
		rc = __fw_packet_handler(client);
		if (rc < 0)
			dev_err(&client->dev,
				"%s: fw_packet_handler fail, rc = %d", __func__,
				rc);

		dev_info(&client->dev, "%s: firmware checking done\n",
			 __func__);

		// Check for FW_VERSION, if 0x0000 means FW update fail!
		if (FW_VERSION == 0x00) {
			rc = 0x80;
			dev_err(&client->dev,
				"%s: FW_VERSION = %d, last FW update fail\n",
				__func__, FW_VERSION);
		}
	}
	return -rc;
}

static int elan_ktf2k_ts_rough_calibrate(struct i2c_client *client)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};

	dev_info(&client->dev, "%s: enter\n", __func__);
	dev_info(&client->dev, "%s: dump cmd: %02x, %02x, %02x, %02x\n",
		 __func__, cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev, "%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

//[Arima Edison] add to set or get different mode(charging and not charging) ++
/*
 * for 2227e:
 * 54 56 01 01 while charging
 * 54 56 00 01 while not charging
 *
 * for 2127e:
 *     0x54 0x5C 0x01 0x01 enable AC mode
 * get 0x54 0x57 0x00 0x01
 *     0x54 0x5D 0x01 0x01 enable RF mode
 */
static int elan_ktf2k_ts_set_mode_state(struct i2c_client *client, int mode)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x56, 0x01, 0x01};

	dev_info(&client->dev, "%s: Enter\n", __func__);

	if (chip_type == TOUCH_2127)
		cmd[1] = 0x5C;

	cmd[2] = mode;

	dev_info(&client->dev, "%s: dump cmd: %02x, %02x, %02x, %02x\n",
		 __func__, cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev, "%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

/* 53 56 00 01 */
static int elan_ktf2k_ts_get_mode_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x56, 0x00, 0x01};
	uint8_t buf[4] = {0}, mode_state = 0;

	if (chip_type == TOUCH_2127)
		cmd[1] = 0x5C;

	rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	if (rc)
		return rc;

	/* third parameter is the one to distinquish the mode */
	mode_state = buf[2];
	dev_info(&client->dev, "%s: dump repsponse: %0x\n", __func__,
		 mode_state);

	return mode_state;
}

static int elan_ktf2k_ts_set_talking_state(struct i2c_client *client, int mode)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x57, 0x01, 0x01};

	dev_info(&client->dev, "%s: Enter\n", __func__);

	if (chip_type == TOUCH_2127)
		cmd[1] = 0x5D;

	cmd[2] = mode;

	dev_info(&client->dev, "%s: dump cmd: %02x, %02x, %02x, %02x\n",
		 __func__, cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev, "%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_get_talking_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x57, 0x00, 0x01};
	uint8_t buf[4] = {0}, mode_state = 0;

	if (chip_type == TOUCH_2127)
		cmd[1] = 0x5D;

	dev_info(&client->dev, "%s: dump cmd: %02x, %02x, %02x, %02x\n",
		 __func__, cmd[0], cmd[1], cmd[2], cmd[3]);

	rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	if (rc)
		return rc;

	/* third parameter is the one to distinquish the mode */
	mode_state = buf[2];
	dev_info(&client->dev, "%s: dump repsponse: %0x\n", __func__,
		 mode_state);

	return mode_state;
}
//[Arima Edison] add to set different mode(charging and not charging) --

//[Arima Edison] add ram clear command ++
static void elan_ktf2k_clear_ram(struct i2c_client *client)
{
	uint8_t clear_cmd[] = {CMD_R_PKT, 0x0A, 0x00, 0x01};

	if ((i2c_master_send(client, clear_cmd, sizeof(clear_cmd))) !=
	    sizeof(clear_cmd)) {
		dev_err(&client->dev, "%s: i2c_master_send failed\n", __func__);
		return;
	}
	__elan_ktf2k_ts_poll(private_ts->client);
	i2c_master_recv(private_ts->client, clear_cmd, 4);
	dev_info(&client->dev, "%s: %2x,%2x,%2x,%2x\n", __func__, clear_cmd[0],
		 clear_cmd[1], clear_cmd[2], clear_cmd[3]);
}
//[Arima Edison] add ram clear command --

static int elan_ktf2k_set_scan_mode(struct i2c_client *client, int mode)
{
	uint8_t stop_cmd[] = {CMD_W_PKT, 0x9F, 0x01, 0x00, 0x00, 0x01};
	uint8_t start_cmd[] = {CMD_W_PKT, 0x9F, 0x00, 0x00, 0x00, 0x01};

	if (chip_type != TOUCH_2227) {
		pr_err("%s: Only 2227 Truly/Eely panel is allowed here, get "
		       "away!\n",
		       __func__);
		return 0;
	}

	if (mode) {
		if ((i2c_master_send(client, start_cmd, sizeof(start_cmd))) !=
		    sizeof(start_cmd)) {
			dev_err(&client->dev,
				"%s: i2c_master_send failed, mode:%d\n",
				__func__, mode);
			return -EINVAL;
		}
	} else {
		if ((i2c_master_send(client, stop_cmd, sizeof(stop_cmd))) !=
		    sizeof(stop_cmd)) {
			dev_err(&client->dev,
				"%s: i2c_master_send failed, mode:%d\n",
				__func__, mode);
			return -EINVAL;
		}
	}

	return 0;
}

static int elan_ktf2k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};

	dev_info(&client->dev, "%s: enter\n", __func__);

	cmd[1] |= (state << 3);

	dev_info(&client->dev, "%s: dump cmd: %02x, %02x, %02x, %02x\n",
		 __func__, cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd))) != sizeof(cmd)) {
		dev_err(&client->dev, "%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf2k_ts_hw_reset(struct i2c_client *client)
{
	gpio_direction_output(SYSTEM_RESET_PIN_SR, 0);
	msleep(20);
	gpio_direction_output(SYSTEM_RESET_PIN_SR, 1);
	msleep(150);

	return 0;
}

static int elan_ktf2k_ts_recv_data(struct i2c_client *client, uint8_t *buf,
				   int bytes_to_recv)
{
	int rc;
	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);

	rc = i2c_master_recv(client, buf, 8);
	if (rc != 8) {
		dev_err(&client->dev, "%s: Read the first package error\n",
			__func__);
		msleep(30);
		return -1;
	}

	return rc;
}

static void elan_ktf2k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;
	uint16_t x, y;
	uint16_t fbits = 0;
	uint8_t i, num_firgers, reported = 0;
	uint8_t idx;
	int max_firgers;

	switch (buf[0]) {
	case TEN_FINGERS_PKT:
		max_firgers = 10;
		num_firgers = buf[2] & 0x0f;
		fbits = buf[2] & 0x30;
		fbits = (fbits << 4) | buf[1];
		idx = 3;
		break;
	case MTK_FINGERS_PKT:
	case FIVE_FINGERS_PKT:
		max_firgers = 5;
		num_firgers = buf[1] & 0x07;
		fbits = buf[1] >> 3;
		idx = 2;
		break;
	default:
		max_firgers = 2;
		num_firgers = buf[7] & 0x03;
		fbits = buf[7] & 0x03;
		idx = 1;
		break;
	}

	switch (buf[0]) {
	case 0x78:
		break;
	case HELLO_PKT:
		// chip may reset due to watch dog
		if (chip_type == TOUCH_2227 && buf[1] == 0x55 &&
		    buf[2] == 0x55 && buf[3] == 0x55) {
			dev_info(&client->dev,
				 "%s: get tp chip int gpio status: %d\n",
				 __func__,
				 gpio_get_value(private_ts->intr_gpio));

			mutex_lock(&private_ts->lock);

			elan_ktf2k_set_scan_mode(private_ts->client, 0);

			elan_ktf2k_ts_set_mode_state(private_ts->client,
						     chip_mode_set);
			if (elan_ktf2k_ts_get_mode_state(private_ts->client) !=
			    chip_mode_set)
				elan_ktf2k_ts_set_mode_state(private_ts->client,
							     chip_mode_set);

			elan_ktf2k_ts_set_talking_state(private_ts->client,
							talking_mode_set);
			if (elan_ktf2k_ts_get_talking_state(
				private_ts->client) != talking_mode_set)
				elan_ktf2k_ts_set_talking_state(
				    private_ts->client, talking_mode_set);

			usleep_range(10000, 10500);
			elan_ktf2k_set_scan_mode(private_ts->client, 1);

			mutex_unlock(&private_ts->lock);
		}
		dev_info(&client->dev, "%s: tp chip reset event may happen\n",
			 __func__);
		break;
	case CALIB_PKT:
		if (buf[1] == CALIB_PKT && buf[2] == CALIB_PKT &&
		    buf[3] == CALIB_PKT)
			dev_info(&client->dev, "%s: calibration packet\n",
				 __func__);
		else
			dev_err(&client->dev, "%s: unknow packet type\n",
				__func__);
		break;
	case MTK_FINGERS_PKT:
	case TWO_FINGERS_PKT:
	case FIVE_FINGERS_PKT:
	case TEN_FINGERS_PKT:
		dev_dbg(&client->dev, "%s: %d fingers\n", __func__,
			num_firgers);
		for (i = 0; i < max_firgers; i++) {
			if ((fbits & 0x01)) {
				elan_ktf2k_ts_parse_xy(&buf[idx], &x, &y);
				x = x * (LCM_X_RESOLUTION) / FW_X_RESOLUTION;
				y = y * (LCM_Y_RESOLUTION) / FW_Y_RESOLUTION;

				if (!((x <= 0) || (y <= 0) ||
				      (x >= LCM_X_RESOLUTION) ||
				      (y >= LCM_Y_RESOLUTION))) {
					input_report_abs(idev,
							 ABS_MT_TOUCH_MAJOR, 8);
					input_report_abs(idev,
							 ABS_MT_POSITION_X, x);
					input_report_abs(idev,
							 ABS_MT_POSITION_Y, y);
					input_report_key(idev, BTN_TOUCH, 1);
					input_mt_sync(idev);
					reported++;
				} // end if border
			}	 // end if finger status
			fbits = fbits >> 1;
			idx += 3;
		} // end for

		if (!reported)
			input_mt_sync(idev);
		input_sync(idev);
		break;

	default:
		dev_err(&client->dev, "%s: unknown packet type: %0x\n",
			__func__, buf[0]);
		break;
	} // end switch

	return;
}

static void elan_ktf2k_ts_check_work_func(struct work_struct *work)
{
	int do_tp_reset = 0;
	int touch_retry = 0;

	disable_irq(private_ts->client->irq);
	flush_work(&private_ts->work);

	if (chip_reset_flag == 0) {
		chip_reset_flag = 1;
		schedule_delayed_work(&private_ts->check_work,
				      msecs_to_jiffies(2500));
		enable_irq(private_ts->client->irq);
		return;
	}

	pr_info("%s: Chip may crash, we need to reset it\n", __func__);

	touch_retry = 3;
	do {
		elan_ktf2k_ts_hw_reset(private_ts->client);
		do_tp_reset = __hello_packet_handler(private_ts->client);
		touch_retry--;
	} while (do_tp_reset != 0 && touch_retry > 0);

	if (do_tp_reset != 0)
		pr_err("%s: Receive hello package fail\n", __func__);
	else {
		mutex_lock(&private_ts->lock);
		if (chip_type == TOUCH_2227) {
			elan_ktf2k_set_scan_mode(private_ts->client, 0);
		}

		elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);
		if (elan_ktf2k_ts_get_mode_state(private_ts->client) !=
		    chip_mode_set) {
			elan_ktf2k_ts_set_mode_state(private_ts->client,
						     chip_mode_set);
		}

		elan_ktf2k_ts_set_talking_state(private_ts->client,
						talking_mode_set);
		if (elan_ktf2k_ts_get_talking_state(private_ts->client) !=
		    talking_mode_set) {
			elan_ktf2k_ts_set_talking_state(private_ts->client,
							talking_mode_set);
		}

		if (chip_type == TOUCH_2227) {
			usleep_range(10000, 10500);
			elan_ktf2k_set_scan_mode(private_ts->client, 1);
		}
		mutex_unlock(&private_ts->lock);
	}

	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
	enable_irq(private_ts->client->irq);
}

static void elan_ktf2k_ts_work_func(struct work_struct *work)
{
	int rc;
	struct elan_ktf2k_ts_data *ts =
	    container_of(work, struct elan_ktf2k_ts_data, work);
	uint8_t buf[4 + PACKET_SIZE] = {0};
	chip_reset_flag = 0;

	if (gpio_get_value(ts->intr_gpio)) {
		pr_err("%s: Detected the jitter on INT pin\n", __func__);
		enable_irq(ts->client->irq);
		return;
	}

	rc = elan_ktf2k_ts_recv_data(ts->client, buf, 4 + PACKET_SIZE);
	if (rc < 0) {
		pr_err("%s: Received the packet Error\n", __func__);
		enable_irq(ts->client->irq);
		return;
	}

	elan_ktf2k_ts_report_data(ts->client, buf);

	enable_irq(ts->client->irq);
	return;
}

static irqreturn_t elan_ktf2k_ts_irq_handler(int irq, void *dev_id)
{
	struct elan_ktf2k_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(ts->elan_wq, &ts->work);

	return IRQ_HANDLED;
}

static int elan_ktf2k_ts_register_interrupt(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	dev_info(&client->dev, "%s: Enter\n", __func__);
	err = request_irq(client->irq, elan_ktf2k_ts_irq_handler,
			  IRQF_TRIGGER_LOW, client->name, ts);
	if (err)
		dev_err(&client->dev, "%s: request_irq %d failed\n", __func__,
			client->irq);

	return err;
}

//[Arima Edison] add to configure and enable the vreg ++
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0)
		   ? regulator_set_optimum_mode(reg, load_uA)
		   : 0;
}

static int elan_ktf2k_ts_power_on(struct elan_ktf2k_ts_data *data, bool on)
{
	int rc;

	if (on == false)
		goto power_off;

	rc = reg_set_optimum_mode_check(data->vcc_ana, ELAN_ACTIVE_LOAD_UA);
	if (rc < 0) {
		dev_err(&data->client->dev,
			"%s: Regulator vcc_ana set_opt failed rc = [%d]\n",
			__func__, rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_ana);
	if (rc) {
		dev_err(&data->client->dev,
			"%s: Regulator vcc_ana enable failed rc = [%d]\n",
			__func__, rc);
		goto error_reg_en_vcc_ana;
	}

	if (data->pdata->i2c_pull_up) {
		rc =
		    reg_set_optimum_mode_check(data->vcc_i2c, ELAN_I2C_LOAD_UA);
		if (rc < 0) {
			dev_err(
			    &data->client->dev,
			    "%s: Regulator vcc_i2c set_opt failed rc = [%d]\n",
			    __func__, rc);
			goto error_reg_opt_vcc;
		}

		rc = regulator_enable(data->vcc_i2c);
		if (rc) {
			dev_err(
			    &data->client->dev,
			    "%s: Regulator vcc_i2c enable failed rc = [%d]\n",
			    __func__, rc);
			goto error_reg_en_vcc_i2c;
		}
	} else {
		dev_info(&data->client->dev, "%s: We do not have i2c_pull_up\n",
			 __func__);
	}

	return 0;

error_reg_en_vcc_i2c:
	if (data->pdata->i2c_pull_up)
		reg_set_optimum_mode_check(data->vcc_i2c, 0);

error_reg_opt_vcc:
	regulator_disable(data->vcc_ana);

error_reg_en_vcc_ana:
	reg_set_optimum_mode_check(data->vcc_ana, 0);

power_off:
	reg_set_optimum_mode_check(data->vcc_ana, 0);
	regulator_disable(data->vcc_ana);
	if (data->pdata->i2c_pull_up) {
		reg_set_optimum_mode_check(data->vcc_i2c, 0);
		regulator_disable(data->vcc_i2c);
	}
	msleep(50);

	return 0;
}

static int elan_ktf2k_ts_regulator_configure(struct elan_ktf2k_ts_data *data,
					     bool on)
{
	int rc = 0;

	if (on == false)
		goto hw_shutdown;

	data->vcc_ana = regulator_get(&data->client->dev, "vdd_ana");
	if (IS_ERR(data->vcc_ana)) {
		rc = PTR_ERR(data->vcc_ana);
		dev_err(&data->client->dev,
			"%s: Regulator get failed vcc_ana rc = [%d]\n",
			__func__, rc);
		return rc;
	}

	if (regulator_count_voltages(data->vcc_ana) > 0) {
		rc = regulator_set_voltage(data->vcc_ana, ELAN_VTG_MIN_UV,
					   ELAN_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"%s: Regulator set_vtg failed rc = [%d]\n",
				__func__, rc);
			goto error_set_vtg_vcc_ana;
		}
	}

	if (data->pdata->i2c_pull_up) {
		data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
		if (IS_ERR(data->vcc_i2c)) {
			rc = PTR_ERR(data->vcc_i2c);
			dev_err(&data->client->dev,
				"%s: Regulator get failed rc = [%d]\n",
				__func__, rc);
			goto error_set_vtg_vcc_ana;
		}
		if (regulator_count_voltages(data->vcc_i2c) > 0) {
			rc = regulator_set_voltage(data->vcc_i2c,
						   ELAN_I2C_VTG_MIN_UV,
						   ELAN_I2C_VTG_MAX_UV);
			if (rc) {
				dev_err(
				    &data->client->dev,
				    "%s: regulator set_vtg failed rc = [%d]\n",
				    __func__, rc);
				goto error_set_vtg_i2c;
			}
		}
	}

	return 0;

error_set_vtg_i2c:
	regulator_put(data->vcc_i2c);

error_set_vtg_vcc_ana:
	regulator_put(data->vcc_ana);
	return rc;

hw_shutdown:
	if (regulator_count_voltages(data->vcc_ana) > 0)
		regulator_set_voltage(data->vcc_ana, 0, ELAN_VTG_MAX_UV);
	regulator_put(data->vcc_ana);

	if (data->pdata->i2c_pull_up) {
		if (regulator_count_voltages(data->vcc_i2c) > 0)
			regulator_set_voltage(data->vcc_i2c, 0,
					      ELAN_I2C_VTG_MAX_UV);
		regulator_put(data->vcc_i2c);
	}

	return 0;
}
//[Arima Edison] add to enable the vreg --

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct elan_ktf2k_ts_data *elan_dev_data =
	    container_of(self, struct elan_ktf2k_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
	    elan_dev_data && elan_dev_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			elan_ktf2k_ts_resume(&elan_dev_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			elan_ktf2k_ts_suspend(&elan_dev_data->client->dev);
	}

	return 0;
}
#endif

static int elan_ktf2k_ts_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	int err = 0;
	int fw_err = 0;
	struct elan_ktf2k_i2c_platform_data *pdata;
	struct elan_ktf2k_ts_data *ts;
	int New_FW_ID;
	int New_FW_VER;

	dev_info(&client->dev, "%s: client->addr = [0x%x], name = [%s]\n",
		 __func__, client->addr, client->name);

	//[Arima Edison] get data from device tree ++
	if (client->dev.of_node) {
		pdata = devm_kzalloc(
		    &client->dev, sizeof(struct elan_ktf2k_i2c_platform_data),
		    GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "%s: Failed to allocate memory\n",
				__func__);
			return -ENOMEM;
		}
		err = elan_ktf2k_ts_parse_dt(&client->dev, pdata);
		if (err)
			return err;
	} else
		pdata = client->dev.platform_data;
	//[Arima Edison] get data from device tree --

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c check functionality error\n",
			__func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct elan_ktf2k_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(&client->dev,
			"%s: allocate elan_ktf2k_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!ts->elan_wq) {
		dev_err(&client->dev, "%s: create workqueue failed\n",
			__func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&ts->work, elan_ktf2k_ts_work_func);
	INIT_DELAYED_WORK(&ts->check_work, elan_ktf2k_ts_check_work_func);

	mutex_init(&ts->lock);

	ts->client = client;
	ts->pdata = pdata;

	i2c_set_clientdata(client, ts);

	//[Arima Ediosn] get/set power/gpio ++
	elan_ktf2k_ts_regulator_configure(ts, true);
	elan_ktf2k_ts_power_on(ts, true);
	dev_info(&client->dev,
		 "%s: pdata->intr_gpio = [%d], pdata->reset_gpio = [%d]\n",
		 __func__, pdata->intr_gpio, pdata->reset_gpio);

	if (gpio_is_valid(pdata->intr_gpio)) {
		/* configure touchscreen irq gpio */
		err = gpio_request(pdata->intr_gpio, "elan_intr_gpio");
		if (err)
			dev_err(&client->dev,
				"%s: unable to request gpio [%d]\n", __func__,
				pdata->intr_gpio);

		err = gpio_direction_input(pdata->intr_gpio);
		client->irq = gpio_to_irq(pdata->intr_gpio);
		dev_info(&client->dev,
			 "%s: elan_intr_gpio = [%d], request gpio = [%d]\n",
			 __func__, pdata->intr_gpio, client->irq);
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		/* configure touchscreen irq gpio */
		err = gpio_request(pdata->reset_gpio, "elan_reset_gpio");
		if (err)
			dev_err(&client->dev,
				"%s: unable to request gpio [%d]\n", __func__,
				pdata->reset_gpio);

		gpio_direction_output(pdata->reset_gpio, 1);
		msleep(150);
	}
	//[Arima Ediosn] get/set power/gpio --

	if (gpio_is_valid(pdata->hw_det_gpio)) {
		touch_panel_type = gpio_get_value(pdata->hw_det_gpio);
		dev_info(&client->dev, "%s: touch_panel_type = %d\n", __func__,
			 touch_panel_type);
	} else
		dev_err(&client->dev, "%s: ELAN TOUCH HW DET GPIO IS WRONG\n",
			__func__);

	if (likely(pdata != NULL))
		ts->intr_gpio = pdata->intr_gpio;

	fw_err = elan_ktf2k_ts_setup(client);
	if (fw_err == -136) {
		/*
		 * Means we can not poll the int pin at the init state, maybe
		 * there is no panel
		 */
		dev_info(&client->dev,
			 "%s: No Elan chip inside, fw_err = [%d]\n", __func__,
			 fw_err);
		err = -ENODEV;
		goto err_create_wq_failed;
	} else if (fw_err < 0)
		dev_info(&client->dev,
			 "%s: No Elan chip inside??, fw_err = [%d]\n", __func__,
			 fw_err);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "%s: Failed to allocate input device\n",
			__func__);
		goto err_input_dev_alloc_failed;
	}

	//[Arima Edison] take chip_type and panel_type as condition to set fw ++
	if (!touch_panel_type && chip_type == TOUCH_2227) {
		file_fw_data = file_fw_data_Truly;
		dev_info(&client->dev, "%s: touch: 2227 Truly panel\n",
			 __func__);
	} else if (!touch_panel_type && chip_type == TOUCH_2127) {
		file_fw_data = file_fw_data_Truly_2127;
		dev_info(&client->dev, "%s: touch: 2127 Truly panel\n",
			 __func__);
	} else if (touch_panel_type && chip_type == TOUCH_2227) {
		file_fw_data = file_fw_data_Eely;
		dev_info(&client->dev, "%s: touch: 2227 Eely panel\n",
			 __func__);
	} else {
		file_fw_data = file_fw_data_Eely_2127;
		dev_info(&client->dev, "%s: touch: 2127 Eely panel\n",
			 __func__);
	}
	//[Arima Edison] take chip_type and panel_type as condition to set fw --

	private_ts = ts;

	ts->input_dev->name = ELAN_KTF2K_NAME;
	ts->input_dev->id.bustype = BUS_I2C;
	//[Arima Edison] add vendor name and version 20130801 ++
	ts->input_dev->id.vendor = touch_panel_type;
	ts->input_dev->id.version = FW_VERSION;
	//[Arima Edison] add vendor name and version 20130801 --
	ts->input_dev->dev.parent = &client->dev;
	ts->input_dev->open = NULL;
	ts->input_dev->close = NULL;

	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(BTN_TOUCH, ts->input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	/* For single touch */
	input_set_abs_params(ts->input_dev, ABS_X, 0, LCM_X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, LCM_Y_RESOLUTION, 0, 0);
	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,
			     LCM_X_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
			     LCM_Y_RESOLUTION, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	// Start Firmware auto Update
	work_lock = 1;
	power_lock = 1;
	/* FW ID & FW VER*/
	if (chip_type == TOUCH_2227) {
		// 2227e
		dev_info(&client->dev,
			 "%s: [7E65] = [0x%02x], [7E64] = [0x%02x], "
			 "[0x7E67] = [0x%02x], [0x7E66] = [0x%02x]\n",
			 __func__, file_fw_data[0x7E65], file_fw_data[0x7E64],
			 file_fw_data[0x7E67], file_fw_data[0x7E66]);
		New_FW_ID = file_fw_data[0x7E67] << 8 | file_fw_data[0x7E66];
		New_FW_VER = file_fw_data[0x7E65] << 8 | file_fw_data[0x7E64];
	} else {
		// 2127e
		dev_info(&client->dev,
			 "%s: [7D97] = [0x%02x], [7D96] = [0x%02x], "
			 "[0x7D99] = [0x%02x], [0x7D98] = [0x%02x]\n",
			 __func__, file_fw_data[0x7D97], file_fw_data[0x7D96],
			 file_fw_data[0x7D99], file_fw_data[0x7D98]);
		New_FW_ID = file_fw_data[0x7D67] << 8 | file_fw_data[0x7D66];
		New_FW_VER = file_fw_data[0x7D65] << 8 | file_fw_data[0x7D64];
	}
	dev_info(&client->dev, "%s: FW_ID= [0x%x], New_FW_ID= [0x%x]\n",
		 __func__, FW_ID, New_FW_ID);
	dev_info(
	    &client->dev,
	    "%s: FW_VERSION= [0x%x], New_FW_VER= [0x%x], chip_type= [0x%x]\n",
	    __func__, FW_VERSION, New_FW_VER, chip_type);

	// for firmware auto-upgrade
	if (New_FW_ID == FW_ID) {
		if (New_FW_VER > (FW_VERSION)) {
			Update_FW_One(client, RECOVERY);
			FW_VERSION = New_FW_VER;
			ts->input_dev->id.version = FW_VERSION;
		} else
			dev_info(&client->dev,
				 "%s: We do not NEED update TOUCH FW !!\n",
				 __func__);
	} else
		dev_info(&client->dev, "%s: FW_ID is different!", __func__);

	if (FW_ID == 0) {
		RECOVERY = 0x80;
		Update_FW_One(client, RECOVERY);
		FW_VERSION = New_FW_VER;
		ts->input_dev->id.version = FW_VERSION;
	}

	power_lock = 0;
	work_lock = 0;
	// End Firmware auto Update

	err = input_register_device(ts->input_dev);
	if (err) {
		dev_err(&client->dev,
			"%s: unable to register %s: input device\n", __func__,
			ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	elan_ktf2k_touch_sysfs_init();

	dev_info(&client->dev, "%s: Start touchscreen %s: in interrupt mode\n",
		 __func__, ts->input_dev->name);

#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&ts->fb_notif);
	if (err)
		dev_err(&client->dev,
			"%s: Unable to register fb_notifier: %d\n", __func__,
			err);
#endif

	// Firmware Update ++
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG | S_IRWXUGO;

	if (misc_register(&ts->firmware) < 0)
		dev_err(&client->dev, "%s: misc_register failed!!", __func__);
	else
		dev_info(&client->dev, "%s: misc_register finished!!",
			 __func__);
	// Firmware Update --

	if (chip_type == TOUCH_2227)
		elan_ktf2k_clear_ram(private_ts->client);

	// set chip to resume mode
	tp_sleep_status = 1;

	elan_ktf2k_ts_register_interrupt(ts->client);
	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
	dev_info(&client->dev, "%s: Finish\n", __func__);

	return 0;

err_input_register_device_failed:
	if (ts->input_dev)
		input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);

err_create_wq_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:
	return err;
}

static int elan_ktf2k_ts_remove(struct i2c_client *client)
{
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);

	elan_touch_sysfs_deinit();

	free_irq(client->irq, ts);

	if (ts->elan_wq)
		destroy_workqueue(ts->elan_wq);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

static int elan_ktf2k_ts_parse_dt(struct device *dev,
				  struct elan_ktf2k_i2c_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "elan,reset-gpio", 0,
						    &pdata->reset_gpio_flags);
	pdata->intr_gpio = of_get_named_gpio_flags(np, "elan,irq-gpio", 0,
						   &pdata->intr_gpio_flags);
	pdata->hw_det_gpio = of_get_named_gpio_flags(np, "elan,hw-det-gpio", 0,
						     &pdata->hw_det_gpio_flags);
	pdata->i2c_pull_up = of_property_read_bool(np, "elan,i2c-pull-up");

	return 0;
}

static int elan_ktf2k_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ktf2k_ts_data *ts = i2c_get_clientdata(client);
	struct input_dev *idev = ts->input_dev;

	pr_info("%s: Enter\n", __func__);

	// if chip in the sleep mode, we do not need to do it
	if (tp_sleep_status == 0)
		return 0;

	// The power_lock can be removed when firmware upgrade procedure
	// will not be enter into suspend mode.
	if (power_lock == 1)
		return 0;

	disable_irq(client->irq);

	// release all touches
	input_report_key(idev, BTN_TOUCH, 0);
	input_sync(idev);

	flush_work(&ts->work);
	cancel_delayed_work_sync(&ts->check_work);

	mutex_lock(&private_ts->lock);
	elan_ktf2k_ts_set_power_state(client, 0);
	mutex_unlock(&private_ts->lock);

	// set chip to sleep mode
	tp_sleep_status = 0;

	return 0;
}

static int elan_ktf2k_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	pr_info("%s: Enter\n", __func__);

	// if chip in the resume mode, we do not need to do it
	if (tp_sleep_status == 1)
		return 0;

	// The power_lock can be removed when firmware upgrade procedure will
	// not be enter into resume mode.
	if (power_lock == 1)
		return 0;

	mutex_lock(&private_ts->lock);

	gpio_direction_output(SYSTEM_RESET_PIN_SR, 0);
	msleep(20);
	gpio_direction_output(SYSTEM_RESET_PIN_SR, 1);
	msleep(150);

	if (__hello_packet_handler(private_ts->client) < 0)
		pr_err("%s: hellopacket's receive fail\n", __func__);

	if (chip_type == TOUCH_2227)
		elan_ktf2k_set_scan_mode(private_ts->client, 0);

	elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);
	if (elan_ktf2k_ts_get_mode_state(private_ts->client) != chip_mode_set)
		elan_ktf2k_ts_set_mode_state(private_ts->client, chip_mode_set);

	elan_ktf2k_ts_set_talking_state(private_ts->client, talking_mode_set);
	if (elan_ktf2k_ts_get_talking_state(private_ts->client) !=
	    talking_mode_set)
		elan_ktf2k_ts_set_talking_state(private_ts->client,
						talking_mode_set);

	if (chip_type == TOUCH_2227) {
		usleep_range(10000, 10500);
		elan_ktf2k_set_scan_mode(private_ts->client, 1);
	}
	mutex_unlock(&private_ts->lock);

	schedule_delayed_work(&private_ts->check_work, msecs_to_jiffies(2500));
	enable_irq(client->irq);

	// set chip to resume mode
	tp_sleep_status = 1;

	return 0;
}

static const struct dev_pm_ops ktf2k_pm_ops = {
#if (!defined(CONFIG_FB))
    .suspend = elan_ktf2k_ts_suspend, .resume = elan_ktf2k_ts_resume,
#endif
};

//[Arima Edison] modify for device tree ++
static const struct i2c_device_id elan_ktf2k_ts_id[] = {{ELAN_KTF2K_NAME, 0},
							{}};

MODULE_DEVICE_TABLE(i2c, elan_ktf2k_ts_id);

static struct of_device_id elan_ktf2k_ts_match_table[] = {
    {
	.compatible = "elan,ktf2k_ts",
    },
    {},
};

static struct i2c_driver ktf2k_driver = {
    .driver =
	{
	    .name = "elan_ktf2k_ts",
	    .owner = THIS_MODULE,
	    .of_match_table = elan_ktf2k_ts_match_table,
	    .pm = &ktf2k_pm_ops,
	},
    .probe = elan_ktf2k_ts_probe,
    .remove = __devexit_p(elan_ktf2k_ts_remove),
    .id_table = elan_ktf2k_ts_id,
};

module_i2c_driver(ktf2k_driver);
//[Arima Edison] modify for device tree --

MODULE_DESCRIPTION("ELAN KTF2K Touchscreen Driver");
MODULE_LICENSE("GPL");
