#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
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

#include "BME280.h"

#define DEV_CNT   1
#define DEV_NAME  "BME280"



struct bme280_calib {
    u16 dig_T1;
    s16 dig_T2;
    s16 dig_T3;
    u16 dig_P1;
    s16 dig_P2;
    s16 dig_P3;
    s16 dig_P4;
    s16 dig_P5;
    s16 dig_P6;
    s16 dig_P7;
    s16 dig_P8;
    s16 dig_P9;
    u8  dig_H1;
    s16 dig_H2;
    u8  dig_H3;
    s16 dig_H4;
    s16 dig_H5;
    s8  dig_H6;
};

struct bme280_data{

    struct i2c_client *client;
    struct regmap *regmap; 
    struct mutex lock;

    struct bme280_calib calib;
    s32 t_fine;

    u8 osrs_t;
    u8 osrs_p;
    u8 osrs_h;
    u8 filter;
    u8 standby;
    u8 mode;
    u8 saved_mode;
};

static const struct regmap_config bme280_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
};

static const int bme280_osrs_avail[] = { 0, 1, 2, 4, 8, 16 };

static int bme280_osrs_reg_to_val(u8 reg)
{
    switch (reg) {
    case BME280_OSRS_SKIPPED: return 0;
    case BME280_OSRS_1X:      return 1;
    case BME280_OSRS_2X:      return 2;
    case BME280_OSRS_4X:      return 4;
    case BME280_OSRS_8X:      return 8;
    case BME280_OSRS_16X:     return 16;
    default:                  return -EINVAL;
    }
}

static int bme280_osrs_val_to_reg(int val, u8 *reg)
{
    switch (val) {
    case 0:  *reg = BME280_OSRS_SKIPPED; break;
    case 1:  *reg = BME280_OSRS_1X;      break;
    case 2:  *reg = BME280_OSRS_2X;      break;
    case 4:  *reg = BME280_OSRS_4X;      break;
    case 8:  *reg = BME280_OSRS_8X;      break;
    case 16: *reg = BME280_OSRS_16X;     break;
    default:
        return -EINVAL;
    }

    return 0;
}

static u8 *bme280_chan_osrs_ptr(struct bme280_data *data,
                                const struct iio_chan_spec *chan)
{
    switch (chan->type) {
    case IIO_TEMP:
        return &data->osrs_t;
    case IIO_PRESSURE:
        return &data->osrs_p;
    case IIO_HUMIDITYRELATIVE:
        return &data->osrs_h;
    default:
        return NULL;
    }
}

static const struct iio_chan_spec bme280_channels[] = {
    {
        .type = IIO_TEMP,
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED)|
            BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
    },
    {
        .type = IIO_PRESSURE,
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED)|
            BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
    },
    {
        .type = IIO_HUMIDITYRELATIVE,
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED)|
            BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
    },

};

static int bme280_read_adc(struct bme280_data*data,
			   u32 *adc_temp, u32 *adc_press, u32 *adc_hum)
{   
    int ret;
    u8 buf[8];
    ret = regmap_bulk_read(data->regmap, BME280_REG_PRESS_MSB, buf, sizeof(buf));

    if (ret)
        return ret;

    *adc_press = ((u32)buf[0] << 12) |
                 ((u32)buf[1] << 4)  |
                 ((u32)buf[2] >> 4);

    *adc_temp  = ((u32)buf[3] << 12) |
                 ((u32)buf[4] << 4)  |
                 ((u32)buf[5] >> 4);

    *adc_hum   = ((u32)buf[6] << 8) |
                 (u32)buf[7];

    return 0;
}


static s32 bme280_compensate_temp(struct bme280_data*data, s32 adc_temp)
{
    s32 var1, var2, t;
    struct bme280_calib *c = &data->calib;

    var1 = ((((adc_temp >> 3) - ((s32)c->dig_T1 << 1))) *
             ((s32)c->dig_T2)) >> 11;

    var2 = (((((adc_temp >> 4) - ((s32)c->dig_T1)) *
              ((adc_temp >> 4) - ((s32)c->dig_T1))) >> 12) *
            ((s32)c->dig_T3)) >> 14;

    data->t_fine = var1 + var2;
    t = (data->t_fine * 5 + 128) >> 8;   /* 0.01 degC */

    return t;
}

static u32 bme280_compensate_press(struct bme280_data*data, s32 adc_press)
{
    s32 var1, var2;
    u32 p;
    struct bme280_calib *c = &data->calib;

    var1 = (data->t_fine >> 1) - 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((s32)c->dig_P6);
    var2 = var2 + ((var1 * ((s32)c->dig_P5)) << 1);
    var2 = (var2 >> 2) + (((s32)c->dig_P4) << 16);

    var1 = (((c->dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
            ((((s32)c->dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((s32)c->dig_P1)) >> 15);

    if (var1 == 0)
        return 0;

    p = (((u32)(((s32)1048576) - adc_press) - (var2 >> 12))) * 3125;
    if (p < 0x80000000U)
        p = (p << 1) / (u32)var1;
    else
        p = (p / (u32)var1) * 2;

    var1 = (((s32)c->dig_P9) * ((s32)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((s32)(p >> 2)) * ((s32)c->dig_P8)) >> 13;
    p = (u32)((s32)p + ((var1 + var2 + c->dig_P7) >> 4));

    return p;
}

static u32 bme280_compensate_hum(struct bme280_data*data, s32 adc_hum)
{
    s32 v_x1;
    struct bme280_calib *c = &data->calib;

    v_x1 = data->t_fine - ((s32)76800);

    v_x1 = (((((adc_hum << 14) - (((s32)c->dig_H4) << 20) -
                (((s32)c->dig_H5) * v_x1)) + ((s32)16384)) >> 15) *
            (((((((v_x1 * ((s32)c->dig_H6)) >> 10) *
                 (((v_x1 * ((s32)c->dig_H3)) >> 11) + ((s32)32768))) >> 10) +
               ((s32)2097152)) * ((s32)c->dig_H2) + 8192) >> 14));

    v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) *
                    ((s32)c->dig_H1)) >> 4);

    if (v_x1 < 0)
        v_x1 = 0;
    if (v_x1 > 419430400)
        v_x1 = 419430400;

    return (u32)(v_x1 >> 12);   /* Q22.10, RH = val / 1024 */
}

static int bme280_osrs_reg_to_ratio(u8 osrs_reg)
{
    switch (osrs_reg) {
    case 0: return 0;
    case 1: return 1;
    case 2: return 2;
    case 3: return 4;
    case 4: return 8;
    case 5: return 16;
    default:
        return -EINVAL;
    }
}

static int bme280_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
		int *val, int *val2, long mask)
{   
    struct bme280_data*data = iio_priv(indio_dev);
    u32 adc_temp, adc_press, adc_hum;
    s32 temp;
    u32 press, hum;
    u8 osrs_reg;
    int ret;

    ret = pm_runtime_get_sync(&data->client->dev);
    if (ret < 0) {
        pm_runtime_put_noidle(&data->client->dev);
        return ret;
    }

    mutex_lock(&data->lock);

    switch (mask){
        case IIO_CHAN_INFO_PROCESSED:{
            ret = bme280_read_adc(data, &adc_temp, &adc_press, &adc_hum);
            if (ret)
                goto out;
            temp = bme280_compensate_temp(data, adc_temp);

            switch(chan->type){
            case IIO_TEMP:{
                *val = temp / 100;
                *val2 = (temp % 100) * 10000;   /* 0.01 degC -> micro */
                ret = IIO_VAL_INT_PLUS_MICRO;
                break;
            }
            case IIO_PRESSURE:{
                press = bme280_compensate_press(data, adc_press);
                *val = press / 1000;
                *val2 = (press % 1000) * 1000;  /* Pa -> int + micro-ish style */
                ret = IIO_VAL_INT_PLUS_MICRO;
                break;
            }
            case IIO_HUMIDITYRELATIVE:{
                hum = bme280_compensate_hum(data, adc_hum); /* hum/1024 = %RH */
                *val = hum / 1024;
                *val2 = ((hum % 1024) * 1000000) / 1024;
                ret = IIO_VAL_INT_PLUS_MICRO;
                break;
            }
            default:
                ret = -EINVAL;
                break;
            }
            break;
        }

        case IIO_CHAN_INFO_OVERSAMPLING_RATIO:{
            ret = 0;
            switch(chan->type){
                case IIO_TEMP:
                    osrs_reg = data->osrs_t;
                    break;
                case IIO_PRESSURE:
                    osrs_reg = data->osrs_p;
                    break;
                case IIO_HUMIDITYRELATIVE:
                    osrs_reg = data->osrs_h;
                    break;
                default:
                    ret = -EINVAL;
                    break;
            }
            if (ret)
                break;
            ret = bme280_osrs_reg_to_ratio(osrs_reg);
            if (ret < 0)
                break;

            *val = ret;
            *val2 = 0;
            ret = IIO_VAL_INT;
            break;
        }
        default:
            ret = -EINVAL;
            break;
    }

out:
    mutex_unlock(&data->lock);
    pm_runtime_mark_last_busy(&data->client->dev);
    pm_runtime_put_autosuspend(&data->client->dev);
    return ret;
}

static int bme280_write_raw(struct iio_dev *indio_dev,
                            const struct iio_chan_spec *chan,
                            int val, int val2, long mask)
{
    struct bme280_data *data = iio_priv(indio_dev);
    u8 *osrs;
    u8 old_osrs,new_osrs;
    
    int ret;

    if (val2 != 0)
        return -EINVAL;

    ret = pm_runtime_get_sync(&data->client->dev);
    if (ret < 0) {
        pm_runtime_put_noidle(&data->client->dev);
        return ret;
    }

    mutex_lock(&data->lock);

    switch (mask) {
    case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
        osrs = bme280_chan_osrs_ptr(data, chan);
        if (!osrs) {
            ret = -EINVAL;
            break;
        }

        ret = bme280_osrs_val_to_reg(val, &new_osrs);
        if (ret)
            break;

        if (*osrs == new_osrs) {
            ret = 0;
            break;
        }

        /*
         * 工程上更稳妥的做法：
         * 1) 若正在测量，先等 measuring 清零
         * 2) 更新缓存
         * 3) 统一走 apply_settings
         */
        ret = bme280_wait_status_clear(data, BME280_STATUS_MEASURING,
                                       2000, 50000);
        if (ret)
            break;

        old_osrs = *osrs;
        *osrs = new_osrs;
        ret = bme280_apply_settings(data);
        if (ret)
            *osrs = old_osrs;
        break;

    default:
        ret = -EINVAL;
        break;
    }

    mutex_unlock(&data->lock);
    pm_runtime_mark_last_busy(&data->client->dev);
    pm_runtime_put_autosuspend(&data->client->dev);
    return ret;
}

static int bme280_read_avail(struct iio_dev *indio_dev,
                             struct iio_chan_spec const *chan,
                             const int **vals, int *type, int *length,
                             long mask)
{
    switch (mask) {
    case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
        *vals = bme280_osrs_avail;
        *type = IIO_VAL_INT;
        *length = ARRAY_SIZE(bme280_osrs_avail);
        return IIO_AVAIL_LIST;

    default:
        return -EINVAL;
    }
}

static const struct iio_info bme280_info = {
	.read_raw		= bme280_read_raw,
	.write_raw		= bme280_write_raw,
    .read_avail = bme280_read_avail,
};

static int bme280_check_chip_id(struct bme280_data*data)
{
    unsigned int id;
    int ret;
    ret = regmap_read(data->regmap, BME280_REG_ID, &id);
    if (ret){
        dev_err(&data->client->dev, "Unable to read the %s chip ID!\n", DEV_NAME);
        return ret;
    }
    if (id != BME280_CHIP_ID){
        dev_err(&data->client->dev, "%s: The read-in chip ID does not match the expected ID.\n", DEV_NAME);
        return -ENODEV;
    }
    return 0;
}

static int bme280_read_calib(struct bme280_data*data)
{
    int ret;
    u8 buf1[24];   /* 0x88 ~ 0x9F */
    u8 buf2[7];    /* 0xE1 ~ 0xE7 */
    unsigned int h1;
    struct bme280_calib *c = &data->calib;

    /* 读 0x88 ~ 0x9F: T1~T3, P1~P9 */
    ret = regmap_bulk_read(data->regmap, BME280_REG_CALIB00, buf1, sizeof(buf1));
    if (ret)
        return ret;

    /* 读 0xA1: H1 */
    ret = regmap_read(data->regmap, BME280_REG_H1, &h1);
    if (ret)
        return ret;

    /* 读 0xE1 ~ 0xE7: H2~H6 */
    ret = regmap_bulk_read(data->regmap, BME280_REG_CALIB26, buf2, sizeof(buf2));
    if (ret)
        return ret;

    /* Temperature calibration */
    c->dig_T1 = (u16)((buf1[1] << 8) | buf1[0]);
    c->dig_T2 = (s16)((buf1[3] << 8) | buf1[2]);
    c->dig_T3 = (s16)((buf1[5] << 8) | buf1[4]);

    /* Pressure calibration */
    c->dig_P1 = (u16)((buf1[7]  << 8) | buf1[6]);
    c->dig_P2 = (s16)((buf1[9]  << 8) | buf1[8]);
    c->dig_P3 = (s16)((buf1[11] << 8) | buf1[10]);
    c->dig_P4 = (s16)((buf1[13] << 8) | buf1[12]);
    c->dig_P5 = (s16)((buf1[15] << 8) | buf1[14]);
    c->dig_P6 = (s16)((buf1[17] << 8) | buf1[16]);
    c->dig_P7 = (s16)((buf1[19] << 8) | buf1[18]);
    c->dig_P8 = (s16)((buf1[21] << 8) | buf1[20]);
    c->dig_P9 = (s16)((buf1[23] << 8) | buf1[22]);

    /* Humidity calibration */
    c->dig_H1 = (u8)h1;
    c->dig_H2 = (s16)((buf2[1] << 8) | buf2[0]);
    c->dig_H3 = buf2[2];

    /*
     * H4/H5 不是普通对齐的 16-bit：
     * dig_H4 = E4 << 4 | (E5 & 0x0F)
     * dig_H5 = E6 << 4 | (E5 >> 4)
     */
    c->dig_H4 = (s16)(((s16)buf2[3] << 4) | (buf2[4] & 0x0F));
    c->dig_H5 = (s16)(((s16)buf2[5] << 4) | (buf2[4] >> 4));
    c->dig_H6 = (s8)buf2[6];

    return 0;
}

static int bme280_wait_status_clear(struct bme280_data *data,
    u8 mask, int sleep_us, int timeout_us)
{
    unsigned int status;
    int ret;
    int elapsed = 0;

    do {
        ret = regmap_read(data->regmap, BME280_REG_STATUS, &status);
        if (ret)
            return ret;

        if (!(status & mask))
            return 0;

        usleep_range(sleep_us, sleep_us + 200);
        elapsed += sleep_us;
    } while (elapsed < timeout_us);

    dev_err(&data->client->dev,
            "status 0x%02x still set, mask=0x%02x\n", status, mask);
    return -ETIMEDOUT;
}

static int bme280_soft_reset(struct bme280_data *data)
{
    int ret;

    ret = regmap_write(data->regmap, BME280_REG_RESET, BME280_SOFT_RESET);
    if (ret)
        return ret;
    /*
     * reset 后 NVM copy 期间 im_update=1，
     * 等它清零后再读 calibration / 下发配置。
     */
    return bme280_wait_status_clear(data, BME280_STATUS_IM_UPDATE,
                                    2000, 20000);
}

static void bme280_init_defaults(struct bme280_data *data)
{
    data->osrs_t = BME280_OSRS_1X;
    data->osrs_p = BME280_OSRS_1X;
    data->osrs_h = BME280_OSRS_1X;
    data->filter = BME280_FILTER_OFF;
    data->standby = 5; /* 对应你当前的 t_sb=5 */
    data->mode = BME280_MODE_NORMAL;
}

static int bme280_apply_settings(struct bme280_data *data)
{
    u8 ctrl_hum, ctrl_meas, config;
    int ret;

    /*
     * BME280 要求改 humidity oversampling 后，
     * 需再写一次 ctrl_meas 才会生效。
     */
    ctrl_hum  = data->osrs_h & 0x07;
    ctrl_meas = ((data->osrs_t & 0x07) << 5) |
                ((data->osrs_p & 0x07) << 2) |
                (data->mode & 0x03);
    config    = ((data->standby & 0x07) << 5) |
                ((data->filter  & 0x07) << 2);

    ret = regmap_write(data->regmap, BME280_REG_CTRL_HUM, ctrl_hum);
    if (ret)
        return ret;

    ret = regmap_write(data->regmap, BME280_REG_CONFIG, config);
    if (ret)
        return ret;

    ret = regmap_write(data->regmap, BME280_REG_CTRL_MEAS, ctrl_meas);
    if (ret)
        return ret;

    return 0;
}

static int bme280_check_device(struct bme280_data*data)
{   
    int ret;
    ret = bme280_check_chip_id(data);
    if(ret)
        return ret;
    //软复位
    ret = bme280_soft_reset(data);
    if (ret)
        return ret;

    ret = bme280_read_calib(data);
    if (ret)
        return ret;

    ret = bme280_apply_settings(data);
    if (ret)
        return ret;

    return 0;
}

static int bme280_read_dt(struct bme280_data *data)
{
    struct device *dev = &data->client->dev;
    struct device_node *np = dev->of_node;
    u32 val;
    u8 reg;
    int ret;

    if (!np)
        return 0;

    if (!of_property_read_u32(np, "bosch,osrs-temp", &val)) {
        ret = bme280_osrs_val_to_reg(val, &reg);
        if (ret){
            dev_err(dev, "invalid bosch,osrs-temp=%u\n", val);
            return -EINVAL;
        }
        data->osrs_t = reg;
    }

    if (!of_property_read_u32(np, "bosch,osrs-press", &val)) {
        ret = bme280_osrs_val_to_reg(val, &reg);
        if (ret){
            dev_err(dev, "invalid bosch,osrs-press=%u\n", val);
            return -EINVAL;
        }
        data->osrs_p = reg;
    }

    if (!of_property_read_u32(np, "bosch,osrs-hum", &val)) {
        ret = bme280_osrs_val_to_reg(val, &reg);
        if (ret){
            dev_err(dev, "invalid bosch,osrs-hum=%u\n", val);
            return -EINVAL;
        }

        data->osrs_h = reg;
    }

    if (!of_property_read_u32(np, "bosch,filter", &val)) {
        if (val > BME280_FILTER_16X){
            dev_err(dev, "invalid bosch,filter=%u\n", val);
            return -EINVAL;
        }
        data->filter = val;
    }

    if (!of_property_read_u32(np, "bosch,standby", &val)) {
        if (val > 7){
            dev_err(dev, "invalid bosch,standby=%u\n", val);
            return -EINVAL;
        }
        data->standby = val;
    }

    return 0;
}

static int bme280_runtime_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct iio_dev *indio_dev = i2c_get_clientdata(client);
    struct bme280_data *data = iio_priv(indio_dev);

    int ret;
    u8 old_mode;

    mutex_lock(&data->lock);

    ret = bme280_wait_status_clear(data, BME280_STATUS_MEASURING,
                                   2000, 50000);
    if (ret)
        goto out;

    old_mode = data->mode;
    data->saved_mode = old_mode;
    data->mode = BME280_MODE_SLEEP;

    ret = bme280_apply_settings(data);
    if (ret)
        data->mode = old_mode;

out:
    mutex_unlock(&data->lock);
    return ret;
}

static int bme280_runtime_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct iio_dev *indio_dev = i2c_get_clientdata(client);
    struct bme280_data *data = iio_priv(indio_dev);
    int ret;
    u8 old_mode;

    mutex_lock(&data->lock);

    old_mode = data->mode;
    data->mode = data->saved_mode;

    ret = bme280_apply_settings(data);
    if (ret)
        data->mode = old_mode;

    mutex_unlock(&data->lock);
    return ret;
}

static int bme280_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    struct iio_dev *indio_dev;
    struct bme280_data *data;
    int ret;

    indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;

    data = iio_priv(indio_dev);
    data->client = client;
    mutex_init(&data->lock);

    data->regmap = devm_regmap_init_i2c(client, &bme280_regmap_config);
    if (IS_ERR(data->regmap)){
        ret = PTR_ERR(data->regmap);
        dev_err(&client->dev, "failed to init regmap: %d\n", ret);
        return ret;
    }
    bme280_init_defaults(data);

    ret = bme280_read_dt(data);
    if (ret)
        return ret;

    indio_dev->dev.parent = &client->dev;
    indio_dev->name = "bme280";
    indio_dev->channels = bme280_channels;
    indio_dev->num_channels = ARRAY_SIZE(bme280_channels);
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->info = &bme280_info;

    i2c_set_clientdata(client, indio_dev);

    ret = bme280_check_device(data);
    if (ret){
        dev_err(&client->dev, "device init failed: %d\n", ret);
        return ret;
    }
    pm_runtime_set_active(&client->dev);
    pm_runtime_set_autosuspend_delay(&client->dev, 1000);
    pm_runtime_use_autosuspend(&client->dev);
    pm_runtime_enable(&client->dev);
    pm_runtime_mark_last_busy(&client->dev);
    pm_runtime_put_autosuspend(&client->dev);

    ret = devm_iio_device_register(&client->dev, indio_dev);
    if (ret){
        dev_err(&client->dev, "failed to register iio\n");
        return ret;
    }

    dev_info(&client->dev,
             "probed: osrs_t=%d osrs_p=%d osrs_h=%d filter=%u standby=%u mode=%u\n",
             bme280_osrs_reg_to_val(data->osrs_t),
             bme280_osrs_reg_to_val(data->osrs_p),
             bme280_osrs_reg_to_val(data->osrs_h),
             data->filter, data->standby, data->mode);

    return 0;
}

static int bme280_remove(struct i2c_client *client)
{
    // struct iio_dev *indio_dev = i2c_get_clientdata(client);
	// struct bme280_data*data = iio_priv(indio_dev);
    pm_runtime_disable(&client->dev);
    dev_info(&client->dev, "removed\n");
    return 0;
}

static const struct i2c_device_id gtp_device_id[] = {
    {"fire,i2c_bme280", 0},
    { }
};
MODULE_DEVICE_TABLE(i2c, gtp_device_id);

static const struct of_device_id bme280_of_match_table[] = {
    { .compatible = "fire,i2c_bme280" },
    { }
};
MODULE_DEVICE_TABLE(of, bme280_of_match_table);

static const struct dev_pm_ops bme280_pm_ops = {
    SET_RUNTIME_PM_OPS(bme280_runtime_suspend,
                       bme280_runtime_resume,
                       NULL)
};

static struct i2c_driver bme_driver = {
    .probe = bme280_probe,
    .remove = bme280_remove,
    .id_table = gtp_device_id,
    .driver = {
        .name = "fire,i2c_bme280",
        .of_match_table = bme280_of_match_table,
        .pm = &bme280_pm_ops,
    },
};

static int __init bme280_driver_init(void)
{   
    return i2c_add_driver(&bme_driver);
}

static void __exit bme280_driver_exit(void)
{   
    i2c_del_driver(&bme_driver);
}

module_init(bme280_driver_init);
module_exit(bme280_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("LIU");