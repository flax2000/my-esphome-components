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

rf_raw_ns = cg.esphome_ns.namespace("rf_raw")
RF_RawComponent = rf_raw_ns.class_(
    "RF_RawComponent", cg.Component
)





CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RF_RawComponent),
            cv.Required(CONF_PIN): pins.internal_gpio_input_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pin = await gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))


RF_Raw_ID_SCHEMA = cv.Schema({cv.GenerateID(): cv.use_id(RF_RawComponent)})

