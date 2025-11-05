#!/usr/bin/env python3
"""
irtest.py – Test the Sharp GP2Y0A02YK IR distance sensor through MCP3008.
Shows raw ADC values and corresponding voltages in real time.
Press Ctrl+C to stop.
"""

import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# --- SPI & MCP3008 setup ---
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D5)  # Chip Select on GPIO5
mcp = MCP.MCP3008(spi, cs)

# --- Select analog input channel ---
chan0 = AnalogIn(mcp, MCP.P0)  # Assuming yellow IR signal wire → CH0

# --- Print header ---
print("Reading IR sensor via MCP3008 (Ctrl+C to exit)")
print("-" * 50)
print(f"{'ADC Value':>10s} | {'Voltage (V)':>12s}")
print("-" * 50)

try:
    while True:
        value = chan0.value        # 0–65535 internal 16-bit representation
        voltage = chan0.voltage    # in Volts
        adc_10bit = int((value / 65535) * 1023)  # Scale to 10-bit for intuition
        print(f"{adc_10bit:10d} | {voltage:12.3f}", end="\r")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting cleanly.")
