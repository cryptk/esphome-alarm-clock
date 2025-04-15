import esphome.codegen as cg
from esphome.components import i2c, sensor, binary_sensor
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_MOTION,
    ICON_BRIEFCASE_DOWNLOAD,
    ICON_SCREEN_ROTATION,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_DEGREE_PER_SECOND,
    UNIT_METER_PER_SECOND_SQUARED,
)

DEPENDENCIES = ["i2c", "qmi8658"]

CONF_QMI8658_ID = "qmi8658_id"
CONF_ACCEL_ODR = "accel_output_data_rate"
CONF_ACCEL_SCALE = "accel_scale"
CONF_GYRO_ODR = "gyro_output_data_rate"
CONF_GYRO_SCALE = "gyro_scale"

qmi8658_ns = cg.esphome_ns.namespace('qmi8658')
QMI8658Component = qmi8658_ns.class_(
    'QMI8658Component', cg.PollingComponent, i2c.I2CDevice
)

ACCEL_OUTPUT_DATA_RATE_OPTIONS = {1000, 500, 250, 125, 63, 31}
ACCEL_SCALE_SENSITIVITY = {
    "2g": 16384,
    "4g": 8192,
    "8g": 4096,
    "16g": 2048
}

GYRO_OUTPUT_DATA_RATE_OPTIONS = {7174, 3587, 1794, 897, 448, 224, 112, 56, 28}
GYRO_SCALE_SENSITIVITY = {
    "16dps": 2048,
    "32dps": 1024,
    "64dps": 512,
    "128dps": 256,
    "256dps": 128,
    "512dps": 64,
    "1024dps": 32,
    "2048dps": 16
}

CONFIG_SCHEMA = cv.Schema(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(QMI8658Component),
            cv.Optional(CONF_ACCEL_ODR, default=500): cv.one_of(
                *ACCEL_OUTPUT_DATA_RATE_OPTIONS
            ),
            cv.Optional(CONF_ACCEL_SCALE, default="4g"): cv.one_of(
                *ACCEL_SCALE_SENSITIVITY.keys()
            ),
            cv.Optional(CONF_GYRO_ODR, default=897): cv.one_of(
                *GYRO_OUTPUT_DATA_RATE_OPTIONS
            ),
            cv.Optional(CONF_GYRO_SCALE, default="128dps"): cv.one_of(
                *GYRO_SCALE_SENSITIVITY.keys()
            ),
        }
    )
    # .extend(cv.polling_component_schema("1s"))
    .extend(i2c.i2c_device_schema(0x6A))
) 

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)


    cg.add(var.set_accel_output_data_rate(config[CONF_ACCEL_ODR]))
    accel_scale = int(config[CONF_ACCEL_SCALE].removesuffix("g"))
    cg.add(var.set_accel_scale(accel_scale))
    cg.add(var.set_accel_sensitivity(ACCEL_SCALE_SENSITIVITY[config[CONF_ACCEL_SCALE]]))

    cg.add(var.set_gyro_output_data_rate(config[CONF_GYRO_ODR]))
    gyro_scale = int(config[CONF_GYRO_SCALE].removesuffix("dps"))
    cg.add(var.set_gyro_scale(gyro_scale))
    cg.add(var.set_gyro_sensitivity(GYRO_SCALE_SENSITIVITY[config[CONF_GYRO_SCALE]]))
