# oled_smiley.py
# Display either a smiley face or "Jason" on the OLED
# Works with SSD1306 128x64 OLED

import time
import board
import busio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

# OLED setup (I2C)
i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

# Toggle to choose what to display
SHOW_SMILEY = True  # Set False to show "Jason"

# Clear display
oled.fill(0)
oled.show()

# Create blank image for drawing.
image = Image.new("1", (oled.width, oled.height))
draw = ImageDraw.Draw(image)

if SHOW_SMILEY:
    # Smiley face drawing
    cx, cy = 64, 32  # center
    r = 25
    # Outline face
    draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=255, fill=0)
    # Eyes
    draw.ellipse((cx - 10, cy - 10, cx - 5, cy - 5), fill=255)
    draw.ellipse((cx + 5, cy - 10, cx + 10, cy - 5), fill=255)
    # Smile arc
    draw.arc((cx - 10, cy - 5, cx + 10, cy + 15), start=0, end=180, fill=255)
else:
    # Text "Jason"
    font = ImageFont.load_default()
    draw.text((40, 28), "Jason is cool", font=font, fill=255)

# Display image
oled.image(image)
oled.show()

print("Displayed on OLED! (Press Ctrl+C to exit)")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    oled.fill(0)
    oled.show()
    print("Cleared OLED and exiting.")
