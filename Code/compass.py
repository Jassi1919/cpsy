#!/usr/bin/env python3
"""
Compass implementation for ICM-20948 IMU sensor
Red LED lights up when facing North (±10 degrees)
"""

import time
import math
import board
import busio
import digitalio
from adafruit_icm20x import ICM20948

class CompassSystem:
    def __init__(self):
        # Initialize I2C bus
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        # Initialize ICM-20948 sensor
        self.icm = ICM20948(self.i2c, address=0x68)
        print("ICM-20948 sensor initialized")
        
        # Initialize LED (GPIO 13)
        self.led = digitalio.DigitalInOut(board.D13)
        self.led.direction = digitalio.Direction.OUTPUT
        self.led.value = False
        
        # Calibration values (you'll need to calibrate these)
        self.mag_calibration = {
            'x_offset': 0.0,
            'y_offset': 0.0,
            'z_offset': 0.0,
            'x_scale': 1.0,
            'y_scale': 1.0,
            'z_scale': 1.0
        }
        
        # North detection threshold (degrees)
        self.north_threshold = 10.0
        
        print("Compass system ready. Rotate sensor to calibrate...")
    
    def calibrate_magnetometer(self, duration=30):
        """
        Calibrate magnetometer by collecting data while rotating sensor
        """
        print(f"Calibrating magnetometer - rotate sensor slowly for {duration} seconds...")
        
        min_x = max_x = min_y = max_y = min_z = max_z = 0
        samples = 0
        start_time = time.monotonic()
        
        while time.monotonic() - start_time < duration:
            try:
                # Read raw magnetometer data
                mag_x, mag_y, mag_z = self.icm.magnetic
                
                # Update min/max values
                if samples == 0:
                    min_x = max_x = mag_x
                    min_y = max_y = mag_y
                    min_z = max_z = mag_z
                else:
                    min_x = min(min_x, mag_x)
                    max_x = max(max_x, mag_x)
                    min_y = min(min_y, mag_y)
                    max_y = max(max_y, mag_y)
                    min_z = min(min_z, mag_z)
                    max_z = max(max_z, mag_z)
                
                samples += 1
                time.sleep(0.1)
                
            except Exception as e:
                print(f"Error during calibration: {e}")
                time.sleep(0.1)
        
        # Calculate calibration values
        self.mag_calibration['x_offset'] = (min_x + max_x) / 2
        self.mag_calibration['y_offset'] = (min_y + max_y) / 2
        self.mag_calibration['z_offset'] = (min_z + max_z) / 2
        
        self.mag_calibration['x_scale'] = (max_x - min_x) / 2
        self.mag_calibration['y_scale'] = (max_y - min_y) / 2
        self.mag_calibration['z_scale'] = (max_z - min_z) / 2
        
        # Normalize scales
        avg_scale = (self.mag_calibration['x_scale'] + 
                     self.mag_calibration['y_scale'] + 
                     self.mag_calibration['z_scale']) / 3
        
        self.mag_calibration['x_scale'] = avg_scale / self.mag_calibration['x_scale']
        self.mag_calibration['y_scale'] = avg_scale / self.mag_calibration['y_scale']
        self.mag_calibration['z_scale'] = avg_scale / self.mag_calibration['z_scale']
        
        print(f"Calibration complete! Samples: {samples}")
        print(f"Offsets: X={self.mag_calibration['x_offset']:.2f}, "
              f"Y={self.mag_calibration['y_offset']:.2f}, "
              f"Z={self.mag_calibration['z_offset']:.2f}")
        print(f"Scales: X={self.mag_calibration['x_scale']:.2f}, "
              f"Y={self.mag_calibration['y_scale']:.2f}, "
              f"Z={self.mag_calibration['z_scale']:.2f}")
    
    def read_calibrated_magnetic_field(self):
        """
        Read and calibrate magnetometer data
        """
        try:
            # Read raw data
            mag_x, mag_y, mag_z = self.icm.magnetic
            
            # Apply calibration
            mag_x = (mag_x - self.mag_calibration['x_offset']) * self.mag_calibration['x_scale']
            mag_y = (mag_y - self.mag_calibration['y_offset']) * self.mag_calibration['y_scale']
            mag_z = (mag_z - self.mag_calibration['z_offset']) * self.mag_calibration['z_scale']
            
            return mag_x, mag_y, mag_z
            
        except Exception as e:
            print(f"Error reading magnetic field: {e}")
            return 0, 0, 0
    
    def calculate_heading(self, mag_x, mag_y):
        """
        Calculate compass heading from magnetometer data
        """
        # Calculate heading in radians
        heading_rad = math.atan2(mag_y, mag_x)
        
        # Convert to degrees
        heading_deg = heading_rad * 180 / math.pi
        
        # Normalize to 0-360 degrees
        if heading_deg < 0:
            heading_deg += 360
        
        return heading_deg
    
    def is_facing_north(self, heading):
        """
        Check if heading is within north threshold
        """
        # North is at 0° (and 360°)
        return (abs(heading) < self.north_threshold or 
                abs(heading - 360) < self.north_threshold)
    
    def run_compass(self):
        """
        Main compass loop
        """
        # First, calibrate the magnetometer
        self.calibrate_magnetometer()
        
        print("Starting compass... Point towards North to test LED")
        print("Press Ctrl+C to exit")
        
        try:
            while True:
                # Read calibrated magnetic field
                mag_x, mag_y, mag_z = self.read_calibrated_magnetic_field()
                
                # Calculate heading
                heading = self.calculate_heading(mag_x, mag_y)
                
                # Check if facing north
                north_detected = self.is_facing_north(heading)
                
                # Control LED
                self.led.value = north_detected
                
                # Print status
                print(f"Heading: {heading:6.1f}° | "
                      f"North: {'YES' if north_detected else 'NO '} | "
                      f"Mag: X={mag_x:6.1f}, Y={mag_y:6.1f}, Z={mag_z:6.1f}")
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nExiting compass...")
            self.led.value = False

def main():
    """Main function"""
    try:
        compass = CompassSystem()
        compass.run_compass()
        
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure:")
        print("1. ICM-20948 is properly connected via I2C")
        print("2. I2C is enabled in raspi-config")
        print("3. Required libraries are installed")

if __name__ == "__main__":
    main()
