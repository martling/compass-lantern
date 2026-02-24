#define IMU_INTERRUPT_PIN 2 // interrupt pin connected to MPU6050; can be D2 or D3 for Arduino Uno or Nano
// PIN_WIRE_SDA        A4  already defined in "pins_arduino.h" for Uno/Nano
// PIN_WIRE_SCL        A5  already defined in "pins_arduino.h" for Uno/Nano

#define LED_DATA_PIN 6
#define NUM_PIXELS 46 // total number of RGB LEDs on strip; 46 pixels ~> 11 cm diameter @144 leds/m

#define POT_PIN A0 // potentiometer
#define BUTTON_PIN 10

// Calibration offsets specific to the used MPU-6050 unit (see README.md)
#define IMU_GYRO_OFFSET_X    190
#define IMU_GYRO_OFFSET_Y    -21
#define IMU_GYRO_OFFSET_Z     25
#define IMU_ACCEL_OFFSET_X -4143
#define IMU_ACCEL_OFFSET_Y -2982
#define IMU_ACCEL_OFFSET_Z   800

#include <Adafruit_NeoPixel.h>
#include <elapsedMillis.h>

// Watchdog timer library
// Currently deactivated, because we want to be able to see
// when the Arduino freezes.
// TODO: Reactivate watchdog timer after freeze problem was found.
// #include <avr/wdt.h>

// MotionApps utilizes the Digital Motion Processor (DMP) on the MPU-6050
// to fuse raw sensor data into Yaw/Pitch/Roll angles.
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// ================================================================
// ===                   Application States                     ===
// ================================================================

int g_targetBearing = 0; // degrees

bool g_performSetup = true;

// ================================================================
// ===                     Helper functions                     ===
// ================================================================

// Wraps an LED index around the strip boundaries,
// preventing out-of-bounds access on a circular strip.
int wrapIndex(int index)
{
  if (index < 0)
    return (index % NUM_PIXELS + NUM_PIXELS) % NUM_PIXELS;
  else
    return index % NUM_PIXELS;
}

// ================================================================
// ===                    LED stripe controls                   ===
// ================================================================

#define DIM_BRIGHTNESS 2 // 2 seems really low, but Adafruit lib has a weird way of setting brightness; found value through trial and error
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// Sets all LEDs of "pixels" to the given RGB values.
void setAllLEDs(int r, int g, int b)
{
  for (unsigned int i = 0; i < pixels.numPixels(); i++)
    pixels.setPixelColor(i, pixels.Color(r, g, b));
}

// Simulates a broken light source: dims all LEDs then briefly
// flashes random pixels at higher brightness.
void flickerEffect()
{
  static elapsedMillis timer;
  static unsigned int interval_ms = 0;

  if (timer < interval_ms)
    return;

  setAllLEDs(DIM_BRIGHTNESS, DIM_BRIGHTNESS, DIM_BRIGHTNESS);

  int flickerCount = random(1, 6);
  for (int i = 0; i < flickerCount; i++)
  {
    int flickerIndex = random(0, NUM_PIXELS);
    int flickerBrightness = random(100, 256);
    pixels.setPixelColor(flickerIndex, pixels.Color(flickerBrightness, flickerBrightness, flickerBrightness));
  }
  pixels.show();

  delay(40);

  setAllLEDs(DIM_BRIGHTNESS, DIM_BRIGHTNESS, DIM_BRIGHTNESS);
  pixels.show();

  timer = 0;
  interval_ms = random(700, 2500);
}

// Lights a 5-pixel arc with soft falloff around the direction
// of yaw_deg adjusted by the target bearing offset.
void illuminateHeading(float yaw_deg)
{
  int yaw_index = int(NUM_PIXELS * (180.0 + yaw_deg + g_targetBearing) / 360.0);

  pixels.clear();
  pixels.setPixelColor(wrapIndex(yaw_index - 2), pixels.Color(5, 5, 5));
  pixels.setPixelColor(wrapIndex(yaw_index - 1), pixels.Color(25, 25, 25));
  pixels.setPixelColor(yaw_index, pixels.Color(255, 255, 255));
  pixels.setPixelColor(wrapIndex(yaw_index + 1), pixels.Color(25, 25, 25));
  pixels.setPixelColor(wrapIndex(yaw_index + 2), pixels.Color(5, 5, 5));
  pixels.show();
}

// Lights a single purple pixel at the heading
// surrounded by the dim brightness of calibration mode.
void illuminateHeadingForCalibration(float yaw_deg)
{
  int yaw_index = int(NUM_PIXELS * (180.0 + yaw_deg + g_targetBearing) / 360.0);

  setAllLEDs(DIM_BRIGHTNESS, DIM_BRIGHTNESS, DIM_BRIGHTNESS);
  pixels.setPixelColor(yaw_index, pixels.Color(255, 0, 255));
  pixels.show();
}

// ================================================================
// ===         IMU configuration / Reading orientation          ===
// ================================================================

// I2C address: 0x68 (AD0 low, default) or 0x69 (AD0 high)
// Allows two MPU-6050 modules on the same I2C bus
MPU6050 mpu(0x68);

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// indicates whether MPU interrupt pin has gone high
volatile bool mpuDataReady = false;
void dmpDataReady()
{
  mpuDataReady = true;
}

// Reads yaw from the MPU-6050 DMP.
// Returns yaw in degrees, or NAN if no data was ready or FIFO overflowed.
float readYawDeg()
{
  // skip execution if no interrupt was received from IMU
  if (!mpuDataReady)
    return NAN;

  mpuDataReady = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return NAN;
  }

  if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    return ypr[0] * 180 / M_PI;
  }

  return NAN; // no data ready yet
}

// ================================================================
// ===                      Calibration                         ===
// ================================================================

// Indicates the heading with a purple light that
// can be moved around using the potentiometer.
// Returns when the button is pressed, locking in the bearing.
void calibrateTargetBearing()
{
  while (!digitalRead(BUTTON_PIN))
  {
    float yaw_deg = readYawDeg();
    if (!isnan(yaw_deg))
    {
      int potValue = analogRead(POT_PIN);
      g_targetBearing = map(potValue, 0, 1023, -180, 180);
      illuminateHeadingForCalibration(yaw_deg);
    }
  }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  // One watchdog timer will reset the device if it is unresponsive
  // for two seconds or more.
  // Currently deactivated, because we want to be able to see
  // when the Arduino freezes.
  // TODO: Reactivate watchdog timer after freeze problem was found.
  // wdt_enable(WDTO_2S);

  pinMode(BUTTON_PIN, INPUT);

  Serial.begin(38400);

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Start the NeoPixel device and turn all the LEDs off
  pixels.begin();
  pixels.clear();
  pixels.show();

  // Initialize, verify, and configure MPU-6050
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  uint8_t devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(IMU_GYRO_OFFSET_X);
  mpu.setYGyroOffset(IMU_GYRO_OFFSET_Y);
  mpu.setZGyroOffset(IMU_GYRO_OFFSET_Z);
  mpu.setXAccelOffset(IMU_ACCEL_OFFSET_X);
  mpu.setYAccelOffset(IMU_ACCEL_OFFSET_Y);
  mpu.setZAccelOffset(IMU_ACCEL_OFFSET_Z);

  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("Enabling interrupt detection..."));
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));

    // halt code execution; without working DMP entering loop() is pointless
    while (true)
      ;
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

// Temporary debug output — remove once freeze issue is resolved. (TODO)
void debugSerialOutput(float yaw_deg)
{
  static unsigned int starttime = millis();
  static elapsedMillis debugOutputTimer;
  static unsigned int debugOutputInterval_ms = 1000;

  if (debugOutputTimer > debugOutputInterval_ms)
  {
    Serial.print((millis() - starttime) / 1000);
    Serial.print("s: ");
    Serial.println(yaw_deg + g_targetBearing);
    debugOutputTimer = 0;
  }
}

void loop()
{
  // Send alive notice to watchdog.
  // Currently deactivated, because we want to be able to see
  // when the Arduino freezes.
  // TODO: Reactivate watchdog timer after freeze problem was found.
  // wdt_reset();

  if (g_performSetup) // flicker until calibrated, then enter working mode
  {
    while (!digitalRead(BUTTON_PIN))
      flickerEffect();

    while (digitalRead(BUTTON_PIN))
      ; // wait for button release

    calibrateTargetBearing();
    g_performSetup = false;
  }
  else // bearing is calibrated; illuminate heading continuously
  {
    float yaw_deg = readYawDeg();

    if (!isnan(yaw_deg))
    {
      illuminateHeading(yaw_deg);

      // TODO: remove debug output when not necessary anymore
      debugSerialOutput(yaw_deg);
    }
  }
}
