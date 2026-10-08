import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import microphone
from esphome.const import CONF_ID
from esphome.core import CORE

CODEOWNERS = ["@lucize"]
DEPENDENCIES = ["wifi", "microphone"]

rtsp_audio_ns = cg.esphome_ns.namespace("rtsp_audio")
RTSPAudioComponent = rtsp_audio_ns.class_("RTSPAudioComponent", cg.Component)

CONF_MICROPHONE = "microphone"
CONF_PORT = "port"
CONF_CHANNEL = "channel"  # backward-compatible alias for output channel index
CONF_AUDIO_CHANNEL = "audio_channel"
CONF_GAIN_FACTOR = "gain_factor"
CONF_RTP_PAYLOAD_TYPE = "rtp_payload_type"
CONF_CODEC = "codec"
CONF_OUTPUT_SAMPLE_RATE = "output_sample_rate"
CONF_PACKET_MS = "packet_ms"
CONF_DEBUG = "debug"
CONF_STATUS_INTERVAL = "status_interval"
CONF_BUFFER_MS = "buffer_ms"
CONF_USERNAME = "username"
CONF_PASSWORD = "password"
CONF_AUTH_REALM = "auth_realm"
CONF_MAX_CLIENTS = "max_clients"


def _validate_options(config):
    if CONF_CHANNEL in config and CONF_AUDIO_CHANNEL in config:
        raise cv.Invalid("Use only one of 'audio_channel' or 'channel' in rtsp_audio. Prefer 'audio_channel'.")
    has_user = CONF_USERNAME in config
    has_pass = CONF_PASSWORD in config
    if has_user != has_pass:
        raise cv.Invalid("Set both 'username' and 'password' to enable RTSP Basic authentication, or omit both.")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RTSPAudioComponent),
            cv.Required(CONF_MICROPHONE): cv.use_id(microphone.Microphone),
            cv.Optional(CONF_PORT, default=8554): cv.port,
            # Index inside the microphone entity output. For a mono INMP441 microphone this is 0.
            # This is intentionally named audio_channel to avoid confusion with the underlying
            # i2s_audio microphone channel option, which uses left/right.
            cv.Optional(CONF_AUDIO_CHANNEL, default=0): cv.int_range(min=0, max=7),
            # Backward-compatible alias. Prefer audio_channel in new configs/README examples.
            cv.Optional(CONF_CHANNEL): cv.int_range(min=0, max=7),
            # Integer gain applied by ESPHome MicrophoneSource before RTP packetization.
            cv.Optional(CONF_GAIN_FACTOR, default=4): cv.int_range(min=1, max=64),
            cv.Optional(CONF_CODEC, default="l16"): cv.enum({
                "l16": "l16",
                "pcm": "l16",
                "pcmu": "pcmu",
                "ulaw": "pcmu",
                "mulaw": "pcmu",
                "g711u": "pcmu",
                "g711_ulaw": "pcmu",
                "pcma": "pcma",
                "alaw": "pcma",
                "g711a": "pcma",
                "g711_alaw": "pcma",
            }, lower=True),
            cv.Optional(CONF_OUTPUT_SAMPLE_RATE): cv.int_range(min=8000, max=48000),
            # Optional override. Defaults are codec-aware: L16=96, PCMU=0, PCMA=8.
            cv.Optional(CONF_RTP_PAYLOAD_TYPE): cv.int_range(min=0, max=127),
            cv.Optional(CONF_PACKET_MS, default=20): cv.int_range(min=10, max=100),
            cv.Optional(CONF_BUFFER_MS, default=200): cv.int_range(min=60, max=2000),
            cv.Optional(CONF_MAX_CLIENTS, default=2): cv.int_range(min=1, max=6),
            cv.Optional(CONF_DEBUG, default=False): cv.boolean,
            cv.Optional(CONF_STATUS_INTERVAL, default="10s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_USERNAME): cv.string_strict,
            cv.Optional(CONF_PASSWORD): cv.string_strict,
            cv.Optional(CONF_AUTH_REALM, default="ESPHome RTSP Audio"): cv.string_strict,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_options,
)


def _validate_framework(config):
    if not CORE.is_esp32:
        raise cv.Invalid("rtsp_audio native microphone component currently supports ESP32 only")
    if getattr(CORE, "using_arduino", False):
        raise cv.Invalid("rtsp_audio native microphone component requires esp32.framework.type: esp-idf, not arduino")
    return config


FINAL_VALIDATE_SCHEMA = _validate_framework


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mic = await cg.get_variable(config[CONF_MICROPHONE])
    audio_channel = config.get(CONF_CHANNEL, config[CONF_AUDIO_CHANNEL])

    cg.add(var.set_microphone(mic))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_channel(audio_channel))
    cg.add(var.set_gain_factor(config[CONF_GAIN_FACTOR]))
    cg.add(var.set_codec(config[CONF_CODEC]))
    cg.add(var.set_rtp_config(config.get(CONF_RTP_PAYLOAD_TYPE, -1), config[CONF_PACKET_MS]))
    if CONF_OUTPUT_SAMPLE_RATE in config:
        cg.add(var.set_output_sample_rate(config[CONF_OUTPUT_SAMPLE_RATE]))
    cg.add(var.set_buffer_ms(config[CONF_BUFFER_MS]))
    cg.add(var.set_max_clients(config[CONF_MAX_CLIENTS]))
    cg.add(var.set_debug(config[CONF_DEBUG]))
    cg.add(var.set_status_interval(config[CONF_STATUS_INTERVAL].total_milliseconds))
    if CONF_USERNAME in config:
        cg.add(var.set_auth(config[CONF_USERNAME], config[CONF_PASSWORD], config[CONF_AUTH_REALM]))
