import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

border_router_ns = cg.esphome_ns.namespace('border_router')
BorderRouter = border_router_ns.class_('BorderRouter', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BorderRouter),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
