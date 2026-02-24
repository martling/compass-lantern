
# Compass Lantern

A lantern that uses an IMU (Inertial Measurement Unit) to track its own orientation and drives a circular LED strip such that the illuminated segment always points in the same absolute direction – its *bearing*. Like a compass needle made of light. Rotate the lantern casing however you like; the glowing arc stays fixed in space.

## Workflow
The lantern has three distinct states after power-up:

1. *Uncalibrated state*: On power-up the lantern has no bearing. It signals this by glowing dimly with irregular bright flashes.
2. *Calibration mode*: A button press enters calibration mode. The potentiometer adjusts the target bearing, visible as a single purple LED tracking the current heading. A second button press confirms the bearing and enters working mode.
3. *Working mode*: The lantern illuminates a soft arc of white LEDs locked to the configured bearing. The arc compensates for rotation automatically. The button and potentiometer have no further function until the next power-up.

## Technical background

An IMU (MPU-6050) continuously measures the lantern's yaw angle relative to its starting orientation. The Arduino uses this heading data to calculate which LEDs on the ring need to be lit so that the illuminated point always faces the same world direction. As the lantern is rotated, the active LEDs shift around the ring to compensate, keeping the light "locked" to a fixed bearing.

**Limitations** – The current version (using an MPU-6050) can only identify its position relative to the position at power-up. It cannot identify its absolute position on earth. Therefore, the bearing needs to be configured after each power-up or restart.

## Hardware

| Component | Example Part | Notes |
|---|---|---|
| Microcontroller | Arduino Nano / Uno | Both use the same pins for communication protocols (e.g. I2C with IMU). |
| IMU | MPU-6050 *or* <br> BNO055 | Currently used by implementation (6-axis). <br> Can read magnetic field for absolute orientation (9-axis). |
| LED Strip | WS2812B strip | NeoPixel-compatible |

**Additional components**: Potentiometer, Button, 300-400Ω Resistor, 10kΩ Resistor

### Wiring

![Circuit Diagram](circuit/diagram.png)

### Possible optimizations
- Do not source the LED strip's Vcc from the Arduino but use a separate source to enable larger ampere supply. This protects the Arduino from heating up due to a high amount of current running through its power regulator.
- Use a 1000 µF / 6.3 V capacitor connected to Vcc and GND of LED to prevent surges in power supply.

## Software

### Dependencies

Using PlatformIO should install all used libraries automatically.

If you want to build it using another IDE add the following libraries:
- **I2CDEV MPU6050** https://github.com/pkourany/I2CDEV_MPU6050
- **Adafruit NeoPixel** https://github.com/adafruit/Adafruit_NeoPixel
- **elapsedMillis** https://github.com/pfeerick/elapsedMillis


## Getting Started

1. **Wire up** the hardware according to the schematic in `circuit/diagram.png`.
2. **Configure** the constants at the top of `main.cpp`:
   - `NUM_PIXELS` — number of LEDs on your ring
   - (Optional) Pins for IMU-interrupt, button, and potentiometer can be changed from the preconfiguration used in the diagram.
3. (Optional) **Calibrate IMU** using a calibration sketch to determine gyro- and acceleration sensor offsets. They can be set at the top of `main.cpp`.
   - e.g. https://github.com/blinkmaker/Improved-MPU6050-calibration
   - Each MPU-6050 is slightly different after manufacturing. The calibration is not necessary to create a working lantern. But it can improve the accuracy and start-up time of the IMU.
4. **Build and upload project**.

## Known Issues

**Arduino freezes during runtime** — The LED strip will remain lit but it will not compensate movement of the lantern anymore.  
A watchdog is not a suitable solution for this problem because restarting the Arduino would require recalibration of the bearing.  
A guard against an MPU FIFO overflow is already in place. The FIFO should not be the problem.

## Further Development
**Get absolute position using magnetic field sensor** — Use a **BNO055** (or another 9-axis IMU) to get an absolute position after power-up. The bearing could then be stored outside the Arduino and read after each restart. This would remove the necessary calibration after start-up, enabling a nicer user experience and allowing the use of a watchdog timer to get rid of the freeze problem.

**Fix freeze-up problem** — The current code has a debug output on the serial monitor that writes the measured yaw for every second. This can be used to see how long it takes the Arduino to freeze up. The longest runtime is around 10 minutes. Possible approaches to this problem:
   - Use refactored code on existing hardware to reduce computational load on Arduino.
   - Use circuit optimizations discussed above to reduce physical load on Arduino.
   - Debug memory usage of Arduino to determine why it freezes.
   - Use more powerful microcontroller.