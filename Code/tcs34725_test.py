import time
import board, busio
import adafruit_tcs34725
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont

# I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Sensor
sensor = adafruit_tcs34725.TCS34725(i2c)
sensor.integration_time = 154  # ms
sensor.gain = 4                # default gain

# OLED
WIDTH, HEIGHT = 128, 64
oled = adafruit_ssd1306.SSD1306_I2C(WIDTH, HEIGHT, i2c, addr=0x3C)

# Pillow setup for text
font = ImageFont.load_default()

while True:
    # Raw RGBC values
    r, g, b, c = sensor.color_raw

    # Normalized fractions
    if c != 0:
        r_n, g_n, b_n = [x / c for x in (r, g, b)]
    else:
        r_n, g_n, b_n = (0, 0, 0)

    # 0–255 RGB bytes (Adafruit helper)
    rgb_bytes = sensor.color_rgb_bytes

    # Console log
    print(
        f"Raw RGBC=({r},{g},{b},{c}) | "
        f"Norm=({r_n:.2f},{g_n:.2f},{b_n:.2f}) | "
        f"RGB bytes={rgb_bytes}"
    )

    # OLED display
    image = Image.new("1", (WIDTH, HEIGHT))
    draw = ImageDraw.Draw(image)
    draw.text((0, 0), "RGBC Sensor", font=font, fill=255)
    draw.text((0, 16), f"R:{r_n:.2f}", font=font, fill=255)
    draw.text((0, 28), f"G:{g_n:.2f}", font=font, fill=255)
    draw.text((0, 40), f"B:{b_n:.2f}", font=font, fill=255)
    oled.image(image)
    oled.show()

    time.sleep(2)