import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_BAUD_RATE, CONF_OFFSET, CONF_DATA, \
                          CONF_UPDATE_INTERVAL, CONF_DEVICE
from esphome.core import CORE, coroutine
from esphome.util import SimpleRegistry
from esphome.py_compat import text_type, binary_type, char_to_byte
from .const import CONF_DATA_BITS, CONF_PARITY, CONF_STOP_BITS, CONF_PREFIX, CONF_SUFFIX, \
                   CONF_CHECKSUM, CONF_CHECKSUM_LAMBDA, CONF_ACK, CONF_RS485_ID, \
                   CONF_PACKET_MONITOR, CONF_PACKET_MONITOR_ID, CONF_SUB_DEVICE, \
                   CONF_STATE_ON, CONF_STATE_OFF, CONF_COMMAND_ON, CONF_COMMAND_OFF, \
                   CONF_COMMAND_STATE, CONF_RX_WAIT, CONF_TX_WAIT, CONF_TX_RETRY_CNT

rs485_ns = cg.esphome_ns.namespace('rs485')
RS485Component = rs485_ns.class_('RS485Component', cg.Component)
SerialMonitor = rs485_ns.class_('SerialMonitor')
num_t_const = rs485_ns.class_('num_t').operator('const')
uint8_const = cg.uint8.operator('const')
uint8_ptr_const = uint8_const.operator('ptr')

MULTI_CONF = False

# Validate HEX: uint8_t[]
def validate_hex_data(value):
    if isinstance(value, list):
        return cv.Schema([cv.hex_uint8_t])(value)
    raise cv.Invalid("data must either be a list of bytes")

# State HEX (hex_t): int offset, uint8_t[] data
STATE_HEX_SCHEMA = cv.Schema({
    cv.Required(CONF_DATA): validate_hex_data,
    cv.Optional(CONF_OFFSET, default=0): cv.int_range(min=0, max=128)
})
def shorthand_state_hex(value):
    value = validate_hex_data(value)
    return STATE_HEX_SCHEMA({CONF_DATA: value, CONF_OFFSET: 0})
def state_hex_schema(value):
    if isinstance(value, dict):
        return STATE_HEX_SCHEMA(value)
    return shorthand_state_hex(value)

# Command HEX: uint8_t[] data, uint8_t[] ack
COMMAND_HEX_SCHEMA = cv.Schema({
    cv.Required(CONF_DATA): validate_hex_data,
    cv.Optional(CONF_ACK, default=[]): validate_hex_data
})
def shorthand_command_hex(value):
    value = validate_hex_data(value)
    return COMMAND_HEX_SCHEMA({CONF_DATA: value, CONF_ACK: []})
def command_hex_schema(value):
    if isinstance(value, dict):
        return COMMAND_HEX_SCHEMA(value)
    return shorthand_command_hex(value)


# RS485 Schema
CONFIG_SCHEMA = cv.All(cv.Schema({
    cv.GenerateID(): cv.declare_id(RS485Component),
    cv.GenerateID(CONF_PACKET_MONITOR_ID): cv.declare_id(SerialMonitor),
    cv.Required(CONF_BAUD_RATE): cv.int_range(min=1, max=115200),
    cv.Optional(CONF_DATA_BITS, default=8): cv.int_range(min=1, max=32),
    cv.Optional(CONF_PARITY, default=0): cv.int_range(min=0, max=3), # 0:No parity, 2:Even, 3:Odd
    cv.Optional(CONF_STOP_BITS, default=1): cv.int_range(min=0, max=1),
    cv.Optional(CONF_RX_WAIT, default=10): cv.int_range(min=1, max=2000),
    cv.Optional(CONF_TX_WAIT): cv.int_range(min=1, max=2000),
    cv.Optional(CONF_TX_RETRY_CNT): cv.int_range(min=1, max=10),
    cv.Optional(CONF_PREFIX): cv.hex_int,
    cv.Optional(CONF_SUFFIX): cv.hex_int,
    cv.Optional(CONF_CHECKSUM): cv.boolean,
    cv.Optional(CONF_CHECKSUM_LAMBDA): cv.returning_lambda,
    cv.Optional(CONF_UPDATE_INTERVAL, default='never'): cv.update_interval,
    cv.Optional(CONF_PACKET_MONITOR): cv.ensure_list(state_hex_schema),
}).extend(cv.COMPONENT_SCHEMA))


def to_code(config):
    cg.add_global(rs485_ns.using)
    var = cg.new_Pvariable(config[CONF_ID],
                           config[CONF_BAUD_RATE],
                           config[CONF_DATA_BITS],
                           config[CONF_PARITY],
                           config[CONF_STOP_BITS],
                           config[CONF_RX_WAIT])
    yield cg.register_component(var, config)

    if CONF_TX_WAIT in config:
        cg.add(var.set_tx_wait(config[CONF_TX_WAIT]))
    if CONF_TX_RETRY_CNT in config:
        cg.add(var.set_tx_retry_cnt(config[CONF_TX_RETRY_CNT]))

    if CONF_PREFIX in config:
        cg.add(var.set_prefix(config[CONF_PREFIX]))
    if CONF_SUFFIX in config:
        cg.add(var.set_suffix(config[CONF_SUFFIX]))
    if CONF_CHECKSUM_LAMBDA in config:
        template_ = yield cg.process_lambda(config[CONF_CHECKSUM_LAMBDA],
                                            [(uint8_const, 'prefix'), (uint8_ptr_const, 'data'),
                                             (num_t_const, 'len')],
                                            return_type=cg.uint8)
        cg.add(var.set_checksum_lambda(template_))
    if CONF_CHECKSUM in config:
        cg.add(var.set_checksum(config[CONF_CHECKSUM]))

    if CONF_PACKET_MONITOR in config:
        sm = cg.new_Pvariable(config[CONF_PACKET_MONITOR_ID])
        yield sm
        for conf in config[CONF_PACKET_MONITOR]:
            data = conf[CONF_DATA]
            offset = conf[CONF_OFFSET]
            cg.add(sm.add_filter([offset, data]))
        cg.add(var.register_listener(sm))


# A schema to use for all RS485 devices, all RS485 integrations must extend this!
RS485_DEVICE_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_RS485_ID): cv.use_id(RS485Component),
    cv.Required(CONF_DEVICE): state_hex_schema,
    cv.Optional(CONF_SUB_DEVICE): state_hex_schema,
    cv.Required(CONF_STATE_ON): state_hex_schema,
    cv.Required(CONF_STATE_OFF): state_hex_schema,
    cv.Required(CONF_COMMAND_ON): command_hex_schema,
    cv.Required(CONF_COMMAND_OFF): command_hex_schema,
    cv.Optional(CONF_COMMAND_STATE): command_hex_schema,
})


HEX_SCHEMA_REGISTRY = SimpleRegistry()

@coroutine
def register_rs485_device(var, config):
    paren = yield cg.get_variable(config[CONF_RS485_ID])
    cg.add(paren.register_listener(var))
    yield var

    device = yield state_hex_expression(config[CONF_DEVICE])
    cg.add(var.set_device(device))

    if CONF_SUB_DEVICE in config:
        sub_device = yield state_hex_expression(config[CONF_SUB_DEVICE])
        cg.add(var.set_sub_device(sub_device))

    state_on = yield state_hex_expression(config[CONF_STATE_ON])
    cg.add(var.set_state_on(state_on))

    state_off = yield state_hex_expression(config[CONF_STATE_OFF])
    cg.add(var.set_state_off(state_off))

    command_on = yield command_hex_expression(config[CONF_COMMAND_ON])
    cg.add(var.set_command_on(command_on))

    command_off = yield command_hex_expression(config[CONF_COMMAND_OFF])
    cg.add(var.set_command_off(command_off))

    if CONF_COMMAND_STATE in config:
        command_state = yield command_hex_expression(config[CONF_COMMAND_STATE])
        cg.add(var.set_command_state(command_state))



@coroutine
def state_hex_expression(conf):
    if conf is None:
        return
    data = conf[CONF_DATA]
    offset = conf[CONF_OFFSET]
    yield offset, data

@coroutine
def command_hex_expression(conf):
    if conf is None:
        return
    data = conf[CONF_DATA]
    if CONF_ACK in conf:
        ack = conf[CONF_ACK]
        #ack = [char_to_byte(x) for x in ack]
        yield data, ack
    else:
        yield data
