#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/bitfield.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>

#include "ICM20602.h"

#define DEV_CNT   1
#define DEV_NAME  "ICM20602"
#define _IF_DEBUG 0

#define ICM20602_FIFO_FRAME_SIZE		sizeof(struct icm20602_sensor_data)
#define ICM20602_FIFO_DEBUG_MAX_FRAMES	8

#define ICM20602_CHAN_ACCEL(_axis, _addr, _scan_idx)			\
{									                            \
	.type = IIO_ACCEL,						                    \
	.modified = 1,							                    \
	.channel2 = IIO_MOD_##_axis,					            \
	.address = (_addr),						                    \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			    \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	            \
	.scan_index = (_scan_idx),					                \
	.scan_type = {							                    \
		.sign = 's',						                    \
		.realbits = 16,						                    \
		.storagebits = 16,					                    \
		.endianness = IIO_BE,					                \
	},								                            \
}

#define ICM20602_CHAN_GYRO(_axis, _addr, _scan_idx)			    \
{									                            \
	.type = IIO_ANGL_VEL,						                \
	.modified = 1,							                    \
	.channel2 = IIO_MOD_##_axis,					            \
	.address = (_addr),						                    \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			    \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	            \
	.scan_index = (_scan_idx),					                \
	.scan_type = {							                    \
		.sign = 's',						                    \
		.realbits = 16,						                    \
		.storagebits = 16,					                    \
		.endianness = IIO_BE,					                \
	},								                            \
}

struct icm20602_scale_entry {
	int uscale;     /* scale 的微小数部分，配合 IIO_VAL_INT_PLUS_MICRO */
	u8 regval;      /* 要写入寄存器量程位的值 */
};

static const struct icm20602_scale_entry icm20602_accel_scale_table[] = {
	{ 598,  0x00 }, /* ±2g */
	{ 1197, 0x08 }, /* ±4g */
	{ 2394, 0x10 }, /* ±8g */
	{ 4788, 0x18 }, /* ±16g */
};

static int icm20602_accel_scale_idx_from_reg(u8 regval)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(icm20602_accel_scale_table); i++) {
		if (icm20602_accel_scale_table[i].regval == (regval & 0x18))
			return i;
	}

	return -EINVAL;
}

static const struct icm20602_scale_entry icm20602_gyro_scale_table[] = {
	{ 133,  0x00 }, /* ±250 dps，先用 micro 近似 */
	{ 266,  0x08 }, /* ±500 dps */
	{ 532,  0x10 }, /* ±1000 dps */
	{ 1064, 0x18 }, /* ±2000 dps */
};

static int icm20602_gyro_scale_idx_from_reg(u8 regval)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(icm20602_gyro_scale_table); i++) {
		if (icm20602_gyro_scale_table[i].regval == (regval & 0x18))
			return i;
	}

	return -EINVAL;
}

static const int icm20602_odr_table[] = {
	4, 8, 10, 20, 25, 50, 100, 125, 200, 250, 500, 1000
};

static int icm20602_odr_to_div(int hz, u8 *div, int *real_hz)
{
	int tmp_div;
	int tmp_real_hz;

	if (hz <= 0 || hz > 1000)
		return -EINVAL;

	tmp_div = 1000 / hz - 1;
	if (tmp_div < 0 || tmp_div > 255)
		return -EINVAL;

	tmp_real_hz = 1000 / (tmp_div + 1);

	if (div)
		*div = tmp_div;
	if (real_hz)
		*real_hz = tmp_real_hz;

	return 0;
}

static const struct regmap_config ICM20602_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .read_flag_mask = 0x80,
    .write_flag_mask = 0x00,
};

struct icm20602_config {
	u8 pwr_mgmt_1;
	u8 pwr_mgmt_2;
	u8 config;
	u8 smplrt_div;
	u8 gyro_config;
	u8 accel_config;
	u8 accel_config_2;
};

struct icm20602_sensor_data {
	__be16 accel_x;
	__be16 accel_y;
	__be16 accel_z;
	__be16 temp;
	__be16 gyro_x;
	__be16 gyro_y;
	__be16 gyro_z;
} __packed;

struct icm20602_scan {
	__be16 accel[3];
	__be16 gyro[3];
	s64 timestamp;
} __aligned(8);

struct icm20602_data{
	struct spi_device *spi;
	struct regmap *regmap;
	struct mutex lock;

    /* 默认寄存器配置模板，可被 DT 覆盖后再应用 */
    struct icm20602_config config;

    /* 当前运行时状态 */
	int accel_scale_idx;     /* 当前 accel 量程在表中的索引 */
	int gyro_scale_idx;      /* 当前 gyro 量程在表中的索引 */
	int sampling_frequency;  /* 当前 ODR / 采样频率，单位 Hz */

    /* buffer / trigger / fifo 相关运行时状态 */
	bool buffer_enabled;
	bool trigger_enabled;
	bool fifo_enabled;
	int fifo_watermark;

    /* IRQ / trigger resource */
	int irq;
	bool has_irq;
	struct iio_trigger *trig;
	bool drdy_trigger_enabled;
};

static const struct icm20602_config icm20602_default_config = {
    .pwr_mgmt_1         = 0x01,           //时钟设置
    .pwr_mgmt_2         = 0x00,          //开启陀螺仪和加速度计
    .config             = 0x01,           //176HZ 1KHZ
    .smplrt_div         = 0x07,           //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    .gyro_config        = 0x18,           //±2000 dps
    .accel_config       = 0x10,           //±8g
    .accel_config_2     = 0x03,           //Average 4 samples   44.8HZ   //0x23 Average 16 samples
};


static const struct iio_chan_spec icm20602_channels[] = {
    ICM20602_CHAN_ACCEL(X, ICM20602_ACCEL_XOUT_H, 0),
    ICM20602_CHAN_ACCEL(Y, ICM20602_ACCEL_YOUT_H, 1),
    ICM20602_CHAN_ACCEL(Z, ICM20602_ACCEL_ZOUT_H, 2),

    ICM20602_CHAN_GYRO(X, ICM20602_GYRO_XOUT_H, 3),
    ICM20602_CHAN_GYRO(Y, ICM20602_GYRO_YOUT_H, 4),
    ICM20602_CHAN_GYRO(Z, ICM20602_GYRO_ZOUT_H, 5),
    IIO_CHAN_SOFT_TIMESTAMP(6),
};

static int icm20602_read_burst(struct icm20602_data *data,
			       struct icm20602_sensor_data *frame)
{
	return regmap_bulk_read(data->regmap,
				ICM20602_ACCEL_XOUT_H,
				frame,
				sizeof(*frame));
}

static int icm20602_get_axis_from_frame(const struct iio_chan_spec *chan,
	const struct icm20602_sensor_data *frame, int *val)
{
    switch(chan->type){
        case (IIO_ACCEL):{
            switch(chan->channel2){
                case IIO_MOD_X:
                    *val = (s16)be16_to_cpu(frame->accel_x);
                    return 0;
                case IIO_MOD_Y:
                    *val = (s16)be16_to_cpu(frame->accel_y);
                    return 0;
                case IIO_MOD_Z:
                    *val = (s16)be16_to_cpu(frame->accel_z);
                    return 0;
                default:
                    return -EINVAL;
            }
        }

        case (IIO_ANGL_VEL):{
            switch (chan->channel2) {
                case IIO_MOD_X:
                    *val = (s16)be16_to_cpu(frame->gyro_x);
                    return 0;
                case IIO_MOD_Y:
                    *val = (s16)be16_to_cpu(frame->gyro_y);
                    return 0;
                case IIO_MOD_Z:
                    *val = (s16)be16_to_cpu(frame->gyro_z);
                    return 0;
                default:
                    return -EINVAL;
            }
        }
        default:
            return -EINVAL;
    }
}


/*
* 当前没有考虑缓冲区溢出的情况
* 如果 FIFO 缓冲区溢出，状态位 FIFO_OFLOW_INT 将自动设置为 1。
* 该位位于 INT_STATUS（寄存器 58）中。
* 当 FIFO 缓冲区溢出时，最旧的数据将丢失，新数据将写入 FIFO，
* 除非寄存器 26 CONFIG 的位 [6] FIFO_MODE = 1。
*/ 

/*
* 下面的fifo_reset，fifo_enable，fifo_disable，fifo_get_count
* 函数不加锁，默认由调用者在需要时持有 data->lock
*/
static int icm20602_fifo_reset(struct icm20602_data *data)
{
	int ret;

	/*
	 * FIFO reset bit is usually self-clearing.
	 *
	 * 调用该函数前，建议已经关闭 FIFO_EN 以及 USER_CTRL.FIFO_EN，
	 * 避免一边写 FIFO 一边 reset。
	 */

	ret = regmap_update_bits(data->regmap,
				 ICM20602_USER_CTRL,
				 ICM20602_USER_CTRL_FIFO_RST,
				 ICM20602_USER_CTRL_FIFO_RST);
	if (ret)
		return ret;

	usleep_range(1000, 2000);
	return 0;
}

static int icm20602_fifo_enable(struct icm20602_data *data)
{
	int ret;
	/*
	 * enable 流程：
	 * 1. disable FIFO_EN 中 accel + gyro
	 * 2. disable USER_CTRL.FIFO_EN
	 * 3. reset FIFO
	 * 4. enable USER_CTRL.FIFO_EN
	 * 5. enable FIFO_EN 中 accel + gyro
	 * 6. data->fifo_enabled = true
	 */
	ret = regmap_update_bits(data->regmap,
				 ICM20602_FIFO_EN,
				 ICM20602_FIFO_EN_ACCEL_GYRO_MASK,
				 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(data->regmap,
				 ICM20602_USER_CTRL,
				 ICM20602_USER_CTRL_FIFO_EN,
				 0);

	if (ret)
		return ret;
	
	ret = icm20602_fifo_reset(data);
	if (ret)
		return ret;
	
	ret = regmap_update_bits(data->regmap,
				 ICM20602_USER_CTRL,
				 ICM20602_USER_CTRL_FIFO_EN,
				 ICM20602_USER_CTRL_FIFO_EN);
	if (ret)
		return ret;
	
	ret = regmap_update_bits(data->regmap,
				 ICM20602_FIFO_EN,
				 ICM20602_FIFO_EN_ACCEL_GYRO_MASK,
				 ICM20602_FIFO_EN_ACCEL_GYRO_MASK);

	if (ret)
		return ret;
	
	data->fifo_enabled = true;

	return 0;

}

static int icm20602_fifo_disable(struct icm20602_data *data)
{
	int ret;

	/*
	 * disable 流程：
	 * 1. disable FIFO_EN 中 accel + gyro
	 * 2. disable USER_CTRL.FIFO_EN
	 * 3. reset FIFO
	 * 4. data->fifo_enabled = false
	 */

	ret = regmap_update_bits(data->regmap,
				 ICM20602_FIFO_EN,
				 ICM20602_FIFO_EN_ACCEL_GYRO_MASK,
				 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(data->regmap,
				 ICM20602_USER_CTRL,
				 ICM20602_USER_CTRL_FIFO_EN,
				 0);
	if (ret)
		return ret;

	ret = icm20602_fifo_reset(data);
	if (ret)
		return ret;

	data->fifo_enabled = false;

	return 0;
}

static int icm20602_fifo_get_count(struct icm20602_data *data, int *count)
{
	int ret;
	u8 buf[2];

	if (!count)
		return -EINVAL;

	ret = regmap_bulk_read(data->regmap,
			       ICM20602_FIFO_COUNTH,
			       buf,
			       sizeof(buf));
	if (ret)
		return ret;

	*count = ((int)buf[0] << 8) | buf[1];

	return 0;
}


/*
* 如果 FIFO 缓冲区为空，则读取寄存器 FIFO_DATA 将返回唯一值 0xFF，直到有新数据可用为止。
* 普通数据永远不会指示 0xFF，因此 0xFF 给出了 FIFO 空的可靠指示。
*/

// 读单帧
static int icm20602_fifo_read_frame(struct icm20602_data *data,
				    struct icm20602_sensor_data *frame)
{
	if (!frame)
		return -EINVAL;

	return regmap_bulk_read(data->regmap,
				ICM20602_FIFO_R_W,
				frame,
				sizeof(*frame));
}

/*
* 这个函数用于：
* 读取 FIFO_COUNT
* 计算 FIFO 中完整 frame 数
* 最多读取少量 frame
* 用 dev_dbg() 打印前几帧数据
* 不接入 IIO buffer
*/
static int icm20602_fifo_debug_drain(struct icm20602_data *data)
{
	struct icm20602_sensor_data frame;
	int ret;
	int count;
	int frames;
	int leftover;
	int i;

	ret = icm20602_fifo_get_count(data, &count);
	if (ret)
		return ret;

	frames = count / ICM20602_FIFO_FRAME_SIZE;
	leftover = count % ICM20602_FIFO_FRAME_SIZE;

	dev_info(&data->spi->dev,
		"fifo debug: count=%d frame_size=%zu frames=%d leftover=%d\n",
		count,
		ICM20602_FIFO_FRAME_SIZE,
		frames,
		leftover);

	if (!frames)
		return 0;

	if (frames > ICM20602_FIFO_DEBUG_MAX_FRAMES)
		frames = ICM20602_FIFO_DEBUG_MAX_FRAMES;

	for (i = 0; i < frames; i++) {
		ret = icm20602_fifo_read_frame(data, &frame);
		if (ret)
			return ret;

		dev_info(&data->spi->dev,
			"fifo[%d]: ax=%d ay=%d az=%d temp=%d gx=%d gy=%d gz=%d\n",
			i,
			(s16)be16_to_cpu(frame.accel_x),
			(s16)be16_to_cpu(frame.accel_y),
			(s16)be16_to_cpu(frame.accel_z),
			(s16)be16_to_cpu(frame.temp),
			(s16)be16_to_cpu(frame.gyro_x),
			(s16)be16_to_cpu(frame.gyro_y),
			(s16)be16_to_cpu(frame.gyro_z));
	}

	return 0;
}

// 调试打印信息函数
static int icm20602_fifo_debug_check_read(struct icm20602_data *data)
{
	int ret;
	int count_before;
	int count_after;

	mutex_lock(&data->lock);

	ret = icm20602_fifo_enable(data);
	if (ret)
		goto out_unlock;

	ret = icm20602_fifo_get_count(data, &count_before);
	if (ret)
		goto out_disable;

	msleep(50);

	ret = icm20602_fifo_get_count(data, &count_after);
	if (ret)
		goto out_disable;

	dev_dbg(&data->spi->dev,
		"fifo debug read: count_before=%d count_after=%d\n",
		count_before, count_after);

	ret = icm20602_fifo_debug_drain(data);

out_disable:
	icm20602_fifo_disable(data);

out_unlock:
	mutex_unlock(&data->lock);

	return ret;
}

static int icm20602_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct icm20602_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		mutex_lock(&data->lock);
		{
			struct icm20602_sensor_data frame;

			ret = icm20602_read_burst(data, &frame);
			if (!ret)
				ret = icm20602_get_axis_from_frame(chan, &frame, val);
			if (!ret)
				ret = IIO_VAL_INT;
		}
		mutex_unlock(&data->lock);

		iio_device_release_direct_mode(indio_dev);
		return ret;

	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&data->lock);
		switch (chan->type) {
		case IIO_ACCEL:
			*val = 0;
			*val2 = icm20602_accel_scale_table[data->accel_scale_idx].uscale;
			ret = IIO_VAL_INT_PLUS_MICRO;
			break;
		case IIO_ANGL_VEL:
			*val = 0;
			*val2 = icm20602_gyro_scale_table[data->gyro_scale_idx].uscale;
			ret = IIO_VAL_INT_PLUS_MICRO;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		mutex_unlock(&data->lock);
		return ret;

	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&data->lock);
		*val = data->sampling_frequency;
		mutex_unlock(&data->lock);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int icm20602_set_accel_scale(struct icm20602_data *data, int val, int val2)
{
	
    int i, ret;
    if(val != 0 )
        return -EINVAL;
    
    for (i=0; i < ARRAY_SIZE(icm20602_accel_scale_table); i++){
        if(val2 ==icm20602_accel_scale_table[i].uscale){
            ret =regmap_update_bits(data->regmap, 
                        ICM20602_ACCEL_CONFIG,
                        0X18,
                        icm20602_accel_scale_table[i].regval);
            if (ret)
                return ret;

            data->accel_scale_idx = i;
            return 0;
        }
    }

    return -EINVAL;
}

static int icm20602_set_gyro_scale(struct icm20602_data *data, int val, int val2)
{
	int i, ret;

	if (val != 0)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(icm20602_gyro_scale_table); i++) {
		if (val2 == icm20602_gyro_scale_table[i].uscale) {
			ret = regmap_update_bits(data->regmap,
						 ICM20602_GYRO_CONFIG,
						 0x18,
						 icm20602_gyro_scale_table[i].regval);
			if (ret)
			 return ret;

			data->gyro_scale_idx = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int icm20602_set_sampling_frequency(struct icm20602_data *data, int val)
{
	int div, real_hz, ret;

	if (val <= 0 || val > 1000)
		return -EINVAL;

	div = 1000 / val - 1;
	if (div < 0 || div > 255)
		return -EINVAL;

	real_hz = 1000 / (div + 1);

	ret = regmap_write(data->regmap, ICM20602_SMPLRT_DIV, div);
	if (ret)
		return ret;

	data->sampling_frequency = real_hz;
	return 0;
}



static int icm20602_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct icm20602_data *data = iio_priv(indio_dev);
	int ret;

	if (iio_buffer_enabled(indio_dev))
		return -EBUSY;
	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	mutex_lock(&data->lock);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_ACCEL)
			ret = icm20602_set_accel_scale(data, val, val2);
		else if (chan->type == IIO_ANGL_VEL)
			ret = icm20602_set_gyro_scale(data, val, val2);
		else
			ret = -EINVAL;
		break;

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (val2 != 0)
			ret = -EINVAL;
		else
			ret = icm20602_set_sampling_frequency(data, val);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&data->lock);
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int icm20602_read_avail(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, const int **vals, int *type, int *length, long mask)
{
    switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_ACCEL) {
			static const int accel_avail[] = { 0, 598, 0, 1197, 0, 2394, 0, 4788 };
			*vals = accel_avail;
			*type = IIO_VAL_INT_PLUS_MICRO;
			*length = ARRAY_SIZE(accel_avail);
			return IIO_AVAIL_LIST;
		}

		if (chan->type == IIO_ANGL_VEL) {
			static const int gyro_avail[] = { 0, 133, 0, 266, 0, 532, 0, 1064 };
			*vals = gyro_avail;
			*type = IIO_VAL_INT_PLUS_MICRO;
			*length = ARRAY_SIZE(gyro_avail);
			return IIO_AVAIL_LIST;
		}
		return -EINVAL;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = icm20602_odr_table;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(icm20602_odr_table);
		return IIO_AVAIL_LIST;

	default:
		return -EINVAL;
	}
}

static irqreturn_t icm20602_trigger_handler(int irq, void *p)
{
    struct iio_poll_func *pf = p;
    struct iio_dev *indio_dev = pf->indio_dev;
    struct icm20602_data *data = iio_priv(indio_dev);
    struct icm20602_sensor_data frame;
	struct icm20602_scan scan;

    int ret;

    // 擦除scan缓冲区的数据
    memset(&scan, 0, sizeof(scan));

    mutex_lock(&data->lock);

    ret = icm20602_read_burst(data, &frame);
    if (!ret) {
		scan.accel[0] = frame.accel_x;
		scan.accel[1] = frame.accel_y;
		scan.accel[2] = frame.accel_z;
		scan.gyro[0] = frame.gyro_x;
		scan.gyro[1] = frame.gyro_y;
		scan.gyro[2] = frame.gyro_z;
	}
	mutex_unlock(&data->lock);

	if (ret)
		goto out_done;

	ret = iio_push_to_buffers_with_timestamp(indio_dev, &scan, pf->timestamp);
	if (ret < 0)
		dev_dbg(&data->spi->dev,
			"failed to push buffer data: %d\n", ret);

out_done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}


static const struct iio_info icm20602_info = {
    .read_raw = icm20602_read_raw,
    .write_raw = icm20602_write_raw,
    .read_avail = icm20602_read_avail,
};

// setip ops

// 在启用缓冲区之前运行的函数
static int icm20602_buffer_preenable(struct iio_dev *indio_dev)
{
    struct icm20602_data *data = iio_priv(indio_dev);

	dev_info(&data->spi->dev, "buffer preenable\n");

	mutex_lock(&data->lock);
	data->buffer_enabled = true;
	data->trigger_enabled = true;
	mutex_unlock(&data->lock);

	return 0;
}

// 标记缓冲区禁用后要运行的函数
static int icm20602_buffer_postdisable(struct iio_dev *indio_dev)
{
    struct icm20602_data *data = iio_priv(indio_dev);

	dev_info(&data->spi->dev, "buffer postdisable\n");

	mutex_lock(&data->lock);
	data->buffer_enabled = false;
	data->trigger_enabled = false;
	mutex_unlock(&data->lock);

	return 0;
}

static int icm20602_buffer_postenable(struct iio_dev *indio_dev)
{
	struct icm20602_data *data = iio_priv(indio_dev);

	dev_info(&data->spi->dev, "buffer postenable\n");

	return iio_triggered_buffer_postenable(indio_dev);
}

static int icm20602_buffer_predisable(struct iio_dev *indio_dev)
{
	struct icm20602_data *data = iio_priv(indio_dev);

	dev_info(&data->spi->dev, "buffer predisable\n");

	return iio_triggered_buffer_predisable(indio_dev);
}


static const struct iio_buffer_setup_ops icm20602_buffer_ops = {
    .preenable = icm20602_buffer_preenable,
	.postenable = icm20602_buffer_postenable,
	.predisable = icm20602_buffer_predisable,
	.postdisable = icm20602_buffer_postdisable,
};

static int icm20602_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct icm20602_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);

	ret = regmap_update_bits(data->regmap,
				 ICM20602_INT_ENABLE,
				 ICM20602_DATA_RDY_INT_EN,
				 state ? ICM20602_DATA_RDY_INT_EN : 0);
	if (!ret)
		data->drdy_trigger_enabled = state;

	mutex_unlock(&data->lock);

	return ret;
}

static int icm20602_validate_trigger_device(struct iio_trigger *trig, 
				struct iio_dev *indio_dev)
{
	struct iio_dev *own_indio_dev = iio_trigger_get_drvdata(trig);
	if (indio_dev != own_indio_dev)
		return -EINVAL;
	return 0;
}

/**
* @set_trigger_state：根据需求开启/关闭触发器
* @validate_device：当当前触发器发生变化时用于验证设备的函数。
**/

static const struct iio_trigger_ops icm20602_trigger_ops = {
	.set_trigger_state = icm20602_set_trigger_state,
	.validate_device = icm20602_validate_trigger_device,
};


static irqreturn_t icm20602_irq_handler(int irq, void *dev_id)
{	
	struct iio_dev *indio_dev = dev_id;
	struct icm20602_data *data = iio_priv(indio_dev);

	iio_trigger_poll(data->trig);
	return IRQ_HANDLED;
}

static int icm20602_setup_trigger(struct iio_dev *indio_dev)
{
	struct icm20602_data *data = iio_priv(indio_dev);
	struct device *dev = &data->spi->dev;
	int ret;

	if (!data->has_irq)
		return 0;

	data->trig = devm_iio_trigger_alloc(dev, "%s-drdy-%s",
					    indio_dev->name,
					    dev_name(dev));
	if (!data->trig)
		return -ENOMEM;

	data->trig->dev.parent = dev;
	data->trig->ops = &icm20602_trigger_ops;

	iio_trigger_set_drvdata(data->trig, indio_dev);

	ret = devm_iio_trigger_register(dev, data->trig);

	if (ret) {
		dev_err(dev, "failed to register drdy trigger: %d\n", ret);
		return ret;
	}

	ret = devm_request_irq(dev,
					data->irq,
					icm20602_irq_handler,
					0,
					dev_name(dev),
					indio_dev);

	if (ret) {
		dev_err(dev, "failed to request irq %d: %d\n",
			data->irq, ret);
		return ret;
	}

	dev_info(dev, "registered data-ready trigger, irq=%d\n", data->irq);

	return 0;

}

static int icm20602_check_chip(struct icm20602_data *data)
{
    int ret;
    unsigned int id;
    ret = regmap_read(data->regmap, ICM20602_WHO_AM_I, &id);
    if (ret) {
        dev_err(&data->spi->dev, "failed to read WHO_AM_I: %d\n", ret);
        return ret;
    }

    dev_info(&data->spi->dev, "WHO_AM_I = 0x%02x\n", id);

    if (id != 0x12) {
        dev_err(&data->spi->dev,
                "unexpected chip id 0x%02x, expected 0x12\n", id);
        return -ENODEV;
    }

    return 0;
}

static int icm20602_soft_reset(struct icm20602_data *data)
{
    int ret;
    unsigned int val;
    int retry;

    ret = regmap_write(data->regmap, ICM20602_PWR_MGMT_1, 0x80);
    if (ret) {
        dev_err(&data->spi->dev, "failed to reset device: %d\n", ret);
        return ret;
    }

    /*
     * 官方代码是 Delay_ms(2) 后循环读 PWR_MGMT_1，
     * 直到读到 0x41。
     */
    usleep_range(2000, 3000);

    for (retry = 0; retry < 20; retry++) {
        ret = regmap_read(data->regmap, ICM20602_PWR_MGMT_1, &val);
        if (ret)
            return ret;

        if (val == 0x41)
            return 0;

        usleep_range(1000, 2000);
    }

    dev_err(&data->spi->dev,
            "reset timeout, PWR_MGMT_1 = 0x%02x\n", val);
    return -ETIMEDOUT;
}

static int icm20602_parse_dt(struct spi_device *spi, struct icm20602_data *data)
{
	struct device *dev = &spi->dev;
	struct device_node *np = dev->of_node;
	u32 val;
	u8 div;
	int real_hz;
	int ret;

	data->config = icm20602_default_config;

	data->irq = spi->irq;
	data->has_irq = data->irq > 0;

	if (!np)
		return 0;

	ret = of_property_read_u32(np, "invensense,accel-range", &val);
	if (!ret) {
		switch (val) {
		case 2:
			data->config.accel_config = 0x00;
			break;
		case 4:
			data->config.accel_config = 0x08;
			break;
		case 8:
			data->config.accel_config = 0x10;
			break;
		case 16:
			data->config.accel_config = 0x18;
			break;
		default:
			dev_warn(dev,
				 "invalid invensense,accel-range=%u, use default\n",
				 val);
			break;
		}
	}

	/* default gyro range: 250 / 500 / 1000 / 2000 (dps) */
	ret = of_property_read_u32(np, "invensense,gyro-range", &val);
	if (!ret) {
		switch (val) {
		case 250:
			data->config.gyro_config = 0x00;
			break;
		case 500:
			data->config.gyro_config = 0x08;
			break;
		case 1000:
			data->config.gyro_config = 0x10;
			break;
		case 2000:
			data->config.gyro_config = 0x18;
			break;
		default:
			dev_warn(dev,
				 "invalid invensense,gyro-range=%u, use default\n",
				 val);
			break;
		}
	}

	/* default sampling-frequency */
	ret = of_property_read_u32(np, "sampling-frequency", &val);
	if (!ret) {
		ret = icm20602_odr_to_div(val, &div, &real_hz);
		if (ret) {
			dev_warn(dev,
				 "invalid sampling-frequency=%u, use default\n",
				 val);
		} else {
			data->config.smplrt_div = div;
		}
	}
	
// out_irq:
// 	data->irq = -1;
// 	data->has_irq = -1;

	return 0;
}

static int icm20602_apply_settings(struct icm20602_data *data)
{
    int ret;
    const struct icm20602_config *cfg = &data->config;

    ret = regmap_write(data->regmap, ICM20602_PWR_MGMT_1, cfg->pwr_mgmt_1);
    if (ret)
        return ret;

    ret = regmap_write(data->regmap, ICM20602_PWR_MGMT_2, cfg->pwr_mgmt_2);
    if (ret)
        return ret;

    ret = regmap_write(data->regmap, ICM20602_CONFIG, cfg->config);
    if (ret)
        return ret;

    ret = regmap_write(data->regmap, ICM20602_SMPLRT_DIV, cfg->smplrt_div);
    if (ret)
        return ret;

    ret = regmap_write(data->regmap, ICM20602_GYRO_CONFIG, cfg->gyro_config);
    if (ret)
        return ret;

    ret = regmap_write(data->regmap, ICM20602_ACCEL_CONFIG, cfg->accel_config);
    if (ret)
        return ret;

    ret = regmap_write(data->regmap, ICM20602_ACCEL_CONFIG_2, cfg->accel_config_2);
    if (ret)
        return ret;

	ret = regmap_write(data->regmap,
		   ICM20602_INT_PIN_CFG,
		   ICM20602_INT_RD_CLEAR);
	if (ret)
		return ret;

    return 0;
}

static int icm20602_init_device(struct icm20602_data *data)
{   
    int ret;
    ret = icm20602_check_chip(data);
    if(ret)
        return ret;

    //官方代码在寄存器初始化前休眠2ms 我们换成软件复位
    ret = icm20602_soft_reset(data);
    if (ret)
        return ret;
	
	ret = icm20602_apply_settings(data);
    if (ret)
        return ret;

	/* 与默认模板保持一致的运行时状态 */
    data->accel_scale_idx =
		icm20602_accel_scale_idx_from_reg(data->config.accel_config);
	if (data->accel_scale_idx < 0)
		return data->accel_scale_idx;
    
	data->gyro_scale_idx =
		icm20602_gyro_scale_idx_from_reg(data->config.gyro_config);
	
	if (data->gyro_scale_idx < 0)
		return data->gyro_scale_idx;
	
	data->sampling_frequency = 1000 / (data->config.smplrt_div + 1);

    data->buffer_enabled = false;
	data->trigger_enabled = false;
	data->drdy_trigger_enabled = false;
	data->fifo_enabled = false;
	data->fifo_watermark = 0;

		/* 临时验证 FIFO_COUNT 是否增长，用完删除 */
	#if _IF_DEBUG
	ret = icm20602_fifo_debug_check_read(data);
	if (ret)
		dev_warn(&data->spi->dev, "fifo debug check failed: %d\n", ret);
	#endif

    return 0;
}

static void icm20602_dump_init_config(struct icm20602_data *data)
{
	dev_dbg(&data->spi->dev,
		"init config: PWR_MGMT_1=0x%02x PWR_MGMT_2=0x%02x CONFIG=0x%02x SMPLRT_DIV=0x%02x GYRO_CONFIG=0x%02x ACCEL_CONFIG=0x%02x ACCEL_CONFIG_2=0x%02x irq=%d has_irq=%d\n",
		data->config.pwr_mgmt_1,
		data->config.pwr_mgmt_2,
		data->config.config,
		data->config.smplrt_div,
		data->config.gyro_config,
		data->config.accel_config,
		data->config.accel_config_2,
		data->irq,
		data->has_irq);
}

static int icm20602_probe(struct spi_device *spi)
{
    int ret;
    struct icm20602_data *data;
    struct iio_dev *indio_dev;

    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;

    data = iio_priv(indio_dev);
    data->spi = spi;
    
    mutex_init(&data->lock);

    data->regmap = devm_regmap_init_spi(spi, &ICM20602_regmap_config);
    if (IS_ERR(data->regmap)){
        ret = PTR_ERR(data->regmap);
        dev_err(&spi->dev, "failed to init regmap: %d\n", ret);
        return ret;
    }

	// 解析设备树
	ret = icm20602_parse_dt(spi, data);
	if (ret) {
		dev_err(&spi->dev, "failed to parse dt: %d\n", ret);
		return ret;
	}

	icm20602_dump_init_config(data);

    indio_dev->dev.parent = &spi->dev;
    indio_dev->name = "icm20602";
    indio_dev->channels = icm20602_channels;
    indio_dev->num_channels = ARRAY_SIZE(icm20602_channels);
    indio_dev->modes = INDIO_DIRECT_MODE| \
                    INDIO_BUFFER_TRIGGERED;
    indio_dev->info = &icm20602_info;

    spi_set_drvdata(spi, indio_dev);

    ret = icm20602_init_device(data);
    if (ret){
        dev_err(&spi->dev, "device init failed: %d\n", ret);
        return ret;
    }

    ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
				      iio_pollfunc_store_time,
				      icm20602_trigger_handler,
				      &icm20602_buffer_ops);
    if (ret) {
		dev_err(&spi->dev, "failed to setup triggered buffer: %d\n", ret);
		return ret;
	}

	ret = icm20602_setup_trigger(indio_dev);
	if (ret)
		return ret;


    ret = devm_iio_device_register(&spi->dev, indio_dev);
    if (ret){
        dev_err(&spi->dev, "failed to register iio\n");
        return ret;
    }

    dev_info(&spi->dev, "probe success!\n");
    return 0;
}

static int icm20602_remove(struct spi_device *spi)
{
    dev_info(&spi->dev, "removed\n");
    return 0;
}

static const struct spi_device_id icm20602_device_id[]={
    {"fire,ICM20602", 0},
    {}
};
MODULE_DEVICE_TABLE(spi, icm20602_device_id);

static const struct of_device_id icm20602_of_match_table[]={
    {.compatible = "fire,ICM20602"},
    {}
};

MODULE_DEVICE_TABLE(of, icm20602_of_match_table);
struct spi_driver icm20602_driver = {
    .probe = icm20602_probe,
    .remove = icm20602_remove,
    .id_table = icm20602_device_id,
    .driver = {
        .name = "ICM20602",
        .owner = THIS_MODULE,
        .of_match_table = icm20602_of_match_table,
    },
};

static int __init icm20602_driver_init(void)
{
    return spi_register_driver(&icm20602_driver);
}

static void __exit icm20602_driver_exit(void)
{
    spi_unregister_driver(&icm20602_driver);
}

module_init(icm20602_driver_init);
module_exit(icm20602_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LIU");