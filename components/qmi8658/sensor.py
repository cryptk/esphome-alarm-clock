import esphome.codegen as cg
from esphome.components import i2c, sensor  # Added import for sensor
import esphome.config_validation as cv
from esphome import automation
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    ICON_BRIEFCASE_DOWNLOAD,
    ICON_SCREEN_ROTATION,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_DEGREE_PER_SECOND,
    UNIT_METER_PER_SECOND_SQUARED,
)

DEPENDENCIES = ["i2c"]

CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"
CONF_GYRO_X = "gyro_x"
CONF_GYRO_Y = "gyro_y"
CONF_GYRO_Z = "gyro_z"

qmi8658_ns = cg.esphome_ns.namespace('qmi8658')
QMI8658Component = qmi8658_ns.class_(
    'QMI8658Component', cg.PollingComponent, i2c.I2CDevice
)

accel_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)
gyro_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREE_PER_SECOND,
    icon=ICON_SCREEN_ROTATION,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)
temperature_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = cv.Schema(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(QMI8658Component),
            cv.Optional(CONF_ACCEL_X): accel_schema,
            cv.Optional(CONF_ACCEL_Y): accel_schema,
            cv.Optional(CONF_ACCEL_Z): accel_schema,
            cv.Optional(CONF_GYRO_X): gyro_schema,
            cv.Optional(CONF_GYRO_Y): gyro_schema,
            cv.Optional(CONF_GYRO_Z): gyro_schema,
            cv.Optional(CONF_TEMPERATURE): temperature_schema,
        }
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(i2c.i2c_device_schema(0x6A))
) 

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    for d in ["x", "y", "z"]:
        accel_key = f"accel_{d}"
        if accel_key in config:
            sens = await sensor.new_sensor(config[accel_key])
            cg.add(getattr(var, f"set_accel_{d}_sensor")(sens))
        
        gyro_key = f"gyro_{d}"
        if gyro_key in config:
            sens = await sensor.new_sensor(config[gyro_key])
            cg.add(getattr(var, f"set_gyro_{d}_sensor")(sens))

        if CONF_TEMPERATURE in config:
            sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
            cg.add(var.set_temperature_sensor(sens))