import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.const import (
    CONF_PIN,
    CONF_ID,
    CONF_TRIGGER_ID,

)
from esphome.cpp_helpers import gpio_pin_expression


#this is mostly a cope any paste job with code from rf_bridge, remote_receiver and mayby more,

rf_sensors_ns = cg.esphome_ns.namespace("rf_sensors")
RF_SensorsComponent = rf_sensors_ns.class_(
    "RF_SensorsComponent", cg.Component
)

RF_SensorsData = rf_sensors_ns.struct("RF_SensorsData")


RF_SensorsReceivedSensorTrigger = rf_sensors_ns.class_(
    "RF_SensorsReceivedSensorTrigger", automation.Trigger.template(RF_SensorsData)
)


CONF_ON_SENSOR_RECEIVED = "on_sensor_received"


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RF_SensorsComponent),
            cv.Required(CONF_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_ON_SENSOR_RECEIVED): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                        RF_SensorsReceivedSensorTrigger
                    ),
                }
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pin = await gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))

    for conf in config.get(CONF_ON_SENSOR_RECEIVED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(RF_SensorsData, "data")], conf)



RF_Sensors_ID_SCHEMA = cv.Schema({cv.GenerateID(): cv.use_id(RF_SensorsComponent)})

