import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.ethernet import EthernetComponent
from esphome.components.meshmesh import MeshmeshComponent
from esphome.const import CONF_ID

CONF_MESHMEsh_ID = 'meshmesh_id'
CONF_ETHERNET_ID = 'ethernet_id'

border_router_ns = cg.esphome_ns.namespace('border_router')
BorderRouter = border_router_ns.class_('BorderRouter', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BorderRouter),
    cv.Required(CONF_MESHMEsh_ID): cv.use_id(MeshmeshComponent),
    cv.Required(CONF_ETHERNET_ID): cv.use_id(EthernetComponent),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    meshmesh = await cg.get_variable(config[CONF_MESHMEsh_ID])
    cg.add(var.set_meshmesh(meshmesh))

    ethernet = await cg.get_variable(config[CONF_ETHERNET_ID])
    cg.add(var.set_ethernet(ethernet))
