import esphome.codegen as cg
from esphome.components import i2c, binary_sensor
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_MOTION,
    ICON_SCREEN_ROTATION,
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
CONF_X_THRESHOLD = "x_threshold"
CONF_Y_THRESHOLD = "y_threshold"
CONF_Z_THRESHOLD = "z_threshold"
CONF_WINDOW = "window"
CONF_WAIT_WINDOW = "wait_window"
CONF_CONFIRM_WINDOW = "confirm_window"

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
        cv.Optional(CONF_NO_MOTION): motion_schema.extend({
            cv.Optional(CONF_X_THRESHOLD, 1): cv.Range(min=1, max=255),
            cv.Optional(CONF_Y_THRESHOLD, 1): cv.Range(min=1, max=255),
            cv.Optional(CONF_Z_THRESHOLD, 1): cv.Range(min=1, max=255),
            cv.Optional(CONF_WINDOW, 255): cv.Range(min=1, max=255),
        }),
        cv.Optional(CONF_ANY_MOTION): motion_schema.extend({
            cv.Optional(CONF_X_THRESHOLD, 3): cv.Range(min=1, max=255),
            cv.Optional(CONF_Y_THRESHOLD, 3): cv.Range(min=1, max=255),
            cv.Optional(CONF_Z_THRESHOLD, 3): cv.Range(min=1, max=255),
            cv.Optional(CONF_WINDOW, 3): cv.Range(min=1, max=255),
        }),
        # TODO: Figure out a way to require no_motion and any_motion to be configured if sig_motion is configured
        cv.Optional(CONF_SIGNIFICANT_MOTION): motion_schema.extend({
            cv.Optional(CONF_WAIT_WINDOW, 100): cv.Range(min=0, max=65535),
            cv.Optional(CONF_CONFIRM_WINDOW, 65535): cv.Range(min=0, max=65535),
        }),
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
        cg.add(var.set_no_motion_x_threshold(config[CONF_NO_MOTION][CONF_X_THRESHOLD]))
        cg.add(var.set_no_motion_y_threshold(config[CONF_NO_MOTION][CONF_Y_THRESHOLD]))
        cg.add(var.set_no_motion_z_threshold(config[CONF_NO_MOTION][CONF_Z_THRESHOLD]))
        cg.add(var.set_no_motion_window(config[CONF_NO_MOTION][CONF_WINDOW]))
    
    if CONF_ANY_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_ANY_MOTION])
        cg.add(var.set_any_motion_detected_sensor(sens))
        cg.add(var.set_any_motion_x_threshold(config[CONF_ANY_MOTION][CONF_X_THRESHOLD]))
        cg.add(var.set_any_motion_y_threshold(config[CONF_ANY_MOTION][CONF_Y_THRESHOLD]))
        cg.add(var.set_any_motion_z_threshold(config[CONF_ANY_MOTION][CONF_Z_THRESHOLD]))
        cg.add(var.set_any_motion_window(config[CONF_ANY_MOTION][CONF_WINDOW]))

    if CONF_SIGNIFICANT_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_SIGNIFICANT_MOTION])
        cg.add(var.set_significant_motion_detected_sensor(sens))
        cg.add(var.set_significant_motion_wait_window(config[CONF_SIGNIFICANT_MOTION][CONF_WAIT_WINDOW]))
        cg.add(var.set_significant_motion_confirm_window(config[CONF_SIGNIFICANT_MOTION][CONF_CONFIRM_WINDOW]))

    if int1_pin := config.get(CONF_INT1_PIN):
        int1 = await cg.gpio_pin_expression(int1_pin)
        cg.add(var.set_interrupt1_pin(int1))
    
    if int2_pin := config.get(CONF_INT2_PIN):
        int2 = await cg.gpio_pin_expression(int2_pin)
        cg.add(var.set_interrupt2_pin(int2))

    parent = await cg.get_variable(config[CONF_QMI8658_ID])
    cg.add(var.set_parent(parent))
