import os
import time
import RPi.GPIO as GPIO
import spidev
import numpy as np
import scipy.io.wavfile as wavfile

# Create the directory for storing urban noise files
save_dir = "/home/pi/urban_noise"
os.makedirs(save_dir, exist_ok=True)

# MCP3008 configuration
spi = spidev.SpiDev()
spi.open(0, 0)  # SPI bus 0, device 0
spi.max_speed_hz = 1000000  # Set SPI speed (1 MHz)

# GPIO pin configuration
left_motor_pin = 18
right_motor_pin = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(left_motor_pin, GPIO.OUT)
GPIO.setup(right_motor_pin, GPIO.OUT)

# Microphone audio callback function
def audio_callback(channel):
    # Read MCP3008 input for left and right microphones
    left_mic_value = read_mcp3008(0)
    right_mic_value = read_mcp3008(1)

    # Calculate current noise level
    left_mic_voltage = convert_to_voltage(left_mic_value)
    right_mic_voltage = convert_to_voltage(right_mic_value)
    left_decibel = voltage_to_decibel(left_mic_voltage)
    right_decibel = voltage_to_decibel(right_mic_voltage)

    print(f"Left dB level: {left_decibel:.2f} dB")
    print(f"Right dB level: {right_decibel:.2f} dB")

    if left_decibel > right_decibel:
        # Vibrate left motor
        GPIO.output(left_motor_pin, GPIO.HIGH)
        GPIO.output(right_motor_pin, GPIO.LOW)
    elif right_decibel > left_decibel:
        # Vibrate right motor
        GPIO.output(right_motor_pin, GPIO.HIGH)
        GPIO.output(left_motor_pin, GPIO.LOW)
    else:
        # Turn off both motors
        GPIO.output(left_motor_pin, GPIO.LOW)
        GPIO.output(right_motor_pin, GPIO.LOW)

# Function to read MCP3008 input
def read_mcp3008(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

# Function to convert MCP3008 value to voltage
def convert_to_voltage(value):
    voltage = (value * 3.3) / 1023  # 3.3V reference
    return voltage

# Function to convert voltage to decibel
def voltage_to_decibel(voltage):
    decibel = 20 * np.log10(voltage + 1e-6)  # Calculate decibel value (+1e-6 to avoid log10(0))
    return decibel

# Initialize GPIO pins
GPIO.output(left_motor_pin, GPIO.LOW)
GPIO.output(right_motor_pin, GPIO.LOW)

# Initialize recording variables
recording_flag = False
recording_start_time = 0.0
recording_data = []

# Open audio stream
samplerate = 44100  # Set sample rate

# Start audio recording
GPIO.add
