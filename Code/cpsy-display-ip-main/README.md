# cpsy-display-ip
Demonstrates the SSD1306 OLED by showing the IP address on the system boot.

This repository contains the file `display-ip.py` that displays an IP
address on the  SSD1306 OLED screen and the file `display-ip.service`
that executes the script on boot after the Raspberry Pi is connected
to a network.

To use the script, you first must connect the screen to the Pi's
I2C bus.

## Setup

Do the following steps on your Raspberry Pi.

1. Install [pillow](https://python-pillow.org/), numpy, and venv:
   > sudo apt install python3-venv python3-pil python3-numpy
2. Create a virtual environment, here called cpsy:
   > python3 -m venv cpsy
3. Edit the generated `pyvenv.cfg` to set `include-system-site-packages = true`
4. Activate the environment `source cpsy/bin/activate`
5. Install the display driver: `pip install adafruit-circuitpython-ssd1306`
6. Copy the display-ip.py program to your Raspberry Pi
7. Edit `display-ip.service` such that
   - User should be your user name.
   - the `WorkingDirectory` points to the directory containing `display-ip.py`
   - the path to python points to your virtual environment
8. Copy the edited `display-ip.service` to `/etc/systemd/system`
9. Enable the service `sudo systemctl enable display-ip.service`
10. Test by starting the service `sudo systemctl start display-ip.service`
