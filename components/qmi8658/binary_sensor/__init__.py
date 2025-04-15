import esphome.codegen as cg
from esphome.components import i2c, binary_sensor
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

from .. import (
    CONF_QMI8658_ID,
    qmi8658_ns,
    QMI8658Component
)

DEPENDENCIES = ["qmi8658"]

CONF_NO_MOTION = "no_motion"
CONF_ANY_MOTION = "any_motion"
CONF_SIGNIFICANT_MOTION = "significant_motion"
CONF_INT1_PIN = "interrupt1_pin"
CONF_INT2_PIN = "interrupt2_pin"

QMI8658BinarySensorComponent = qmi8658_ns.class_(
    'QMI8658BinarySensorComponent', cg.Component
)

motion_schema = binary_sensor.binary_sensor_schema(
    device_class=DEVICE_CLASS_MOTION,
    icon=ICON_SCREEN_ROTATION,
)

CONFIG_SCHEMA = cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(QMI8658BinarySensorComponent),
            cv.GenerateID(CONF_QMI8658_ID): cv.use_id(QMI8658Component),
            cv.Optional(CONF_NO_MOTION): motion_schema,
            cv.Optional(CONF_ANY_MOTION): motion_schema,
            # TODO: Figure out a way to require no_motion and any_motion to be configured if sig_motion is configured
            cv.Optional(CONF_SIGNIFICANT_MOTION): motion_schema,
            cv.Optional(CONF_INT1_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_INT2_PIN): pins.gpio_input_pin_schema,
        }
    )

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # TODO: need to pick one, either if ... in config or if ... := config.get and then do it throughout the whole component
    if CONF_NO_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_NO_MOTION])
        cg.add(var.set_no_motion_detected_sensor(sens))
    
    if CONF_ANY_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_ANY_MOTION])
        cg.add(var.set_any_motion_detected_sensor(sens))

    if CONF_SIGNIFICANT_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_SIGNIFICANT_MOTION])
        cg.add(var.set_significant_motion_detected_sensor(sens))

    if int1_pin := config.get(CONF_INT1_PIN):
        int1 = await cg.gpio_pin_expression(int1_pin)
        cg.add(var.set_interrupt1_pin(int1))
    
    if int2_pin := config.get(CONF_INT2_PIN):
        int2 = await cg.gpio_pin_expression(int2_pin)
        cg.add(var.set_interrupt2_pin(int2))

    parent = await cg.get_variable(config[CONF_QMI8658_ID])
    cg.add(var.set_parent(parent))
