import time
import busio
import digitalio
import board
import RPi.GPIO as GPIO
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
from rtmidiw import MIDIInterface

# set up toggle switch pin
toggle_pin = 26
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(toggle_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# set midi cc code
CC_EXPRESSION = 11

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D22)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create 6 analog input channels
channels = [None] * 6
channels[0] = AnalogIn(mcp, MCP.P0)
channels[1] = AnalogIn(mcp, MCP.P1)
channels[2] = AnalogIn(mcp, MCP.P2)
channels[3] = AnalogIn(mcp, MCP.P3)
channels[4] = AnalogIn(mcp, MCP.P4)
channels[5] = AnalogIn(mcp, MCP.P5)

last_read = [0] * 6

tolerance = 400

# create midi interface
midi = MIDIInterface()


def remap_range(value, left_min, left_max, right_min, right_max):
    left_span = left_max - left_min
    right_span = right_max - right_min
    value_scaled = int(value - left_min) / int(left_span)
    return int(right_min + (value_scaled * right_span))


while True:

    channel_base = 0
    if GPIO.input(toggle_pin) == GPIO.HIGH:
        channel_base = 6

    for c in range(6):
        # read the analog pin
        pot_value = channels[c].value
        
        # how much have they changed since the last read
        pot_adjust = abs(pot_value - last_read[c])

        if pot_adjust > tolerance:
            # convert 16bit adc0 (0-65535) left knob read into 0-127 midi cc value
            value = remap_range(pot_value, 0, 65535, 0, 127)

            midi.send_cc_message(CC_EXPRESSION, c + channel_base, value)
            last_read[c] = pot_value

    time.sleep(0.0001)
