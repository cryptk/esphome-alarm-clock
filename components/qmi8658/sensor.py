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

DEPENDENCIES = ["i2c"]

CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"
CONF_ACCEL_ODR = "accel_output_data_rate"
CONF_GYRO_X = "gyro_x"
CONF_GYRO_Y = "gyro_y"
CONF_GYRO_Z = "gyro_z"
CONF_NO_MOTION = "no_motion"
CONF_ANY_MOTION = "any_motion"
CONF_SIGNIFICANT_MOTION = "significant_motion"
CONF_GYRO_ODR = "gyro_output_data_rate"
CONF_INT1_PIN = "interrupt1_pin"
CONF_INT2_PIN = "interrupt2_pin"

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
motion_schema = binary_sensor.binary_sensor_schema(
    device_class=DEVICE_CLASS_MOTION,
    icon=ICON_SCREEN_ROTATION,
)
temperature_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

ACCEL_OUTPUT_DATA_RATE_OPTIONS = {1000, 500, 250, 125, 63, 31}

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
            cv.Optional(CONF_ACCEL_ODR, default=500): cv.one_of(
                *ACCEL_OUTPUT_DATA_RATE_OPTIONS
            ),
            cv.Optional(CONF_NO_MOTION): motion_schema,
            cv.Optional(CONF_ANY_MOTION): motion_schema,
            cv.Optional(CONF_SIGNIFICANT_MOTION): motion_schema,
            cv.Optional(CONF_INT1_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_INT2_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_TEMPERATURE): temperature_schema,
        }
    )
    .extend(cv.polling_component_schema("1s"))
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

    if CONF_NO_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_NO_MOTION])
        cg.add(getattr(var, CONF_NO_MOTION)(sens))
    
    if CONF_ANY_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_ANY_MOTION])
        cg.add(getattr(var, CONF_ANY_MOTION)(sens))

    if CONF_SIGNIFICANT_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_SIGNIFICANT_MOTION])
        cg.add(getattr(var, CONF_SIGNIFICANT_MOTION)(sens))

    cg.add(var.set_accel_output_data_rate(config[CONF_ACCEL_ODR]))

    if int1_pin := config.get(CONF_INT1_PIN):
        int1 = await cg.gpio_pin_expression(int1_pin)
        cg.add(var.set_interrupt1_pin(int1))
    
    if int2_pin := config.get(CONF_INT2_PIN):
        int2 = await cg.gpio_pin_expression(int2_pin)
        cg.add(var.set_interrupt2_pin(int2))
