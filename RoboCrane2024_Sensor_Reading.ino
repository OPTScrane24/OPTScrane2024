#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <Adafruit_Sensor.h>


int SCALE_X = 0;
int SCALE_Y = 0;

Adafruit_MCP4725 dac;
// Set this value to 12,10 or 8 to adjust the resolution DAC 3 Click
#define DAC_RESOLUTION    (12)
#define DAC_ADDRESS 0x60

byte Arduino_RESOLUTION = 12;
byte BNO055_Address = 55;
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_Address);

int ERROR_OUTPUT=10;

void setup() {

  pinMode(ERROR_OUTPUT, OUTPUT);
  pinMode(A0, OUTPUT);

  analogWriteResolution(Arduino_RESOLUTION); // Set PWM resolution for the Arduino Wifi R4 (might be adjusted by analogWave)
  // Set this value to 8 to 12 bits to adjust the resolution for the Arduino Wifi R4

  Serial.begin(115200); //Sets the data rate in bits per second (baud) for serial data transmission
  //the fastest broadrate for this Arduino is 115200 bits per second
  Serial.println("");
  Serial.print("BNO055 address=");
  Serial.print(BNO055_Address);
  Serial.println("");
  Serial.print("DAC 3 click address=");
  Serial.println(DAC_ADDRESS);
  delay(3000);

  Serial.println("");
  Serial.println("BNO055 Sensor Test");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  else {
    Serial.println("BNO055 detected");
    Serial.println("");
  }

  delay(3000);

  bno.setExtCrystalUse(true);

  // Start I2C-communicatie
  Wire.begin();

  // Print een opstartbericht
  Serial.println("DAC 3 click Test");

  // Controleer of het DAC-apparaat aanwezig is
  if (!dac.begin(DAC_ADDRESS)) {
    Serial.println("no DAC 3 click detected ... Check your wiring or I2C ADDR!");
    while (1) {}
  } else {
    Serial.println("DAC 3 click detected");
  }

  delay(3000);
}

void loop(void) {

  sensors_event_t event;
  bno.getEvent(&event);
  Serial.println("");
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");

  //4095 possibilities for 12 bits
  //1024 possibilities for 10 bits
  //255 possibilities for 8 bits
  SCALE_X = map(event.orientation.z, 7.9375, -0.5, 4095, 0);
  SCALE_Y = map(event.orientation.y, 7.9375, -0.5, 4095, 0);
//19.25
  Serial.println("");
  Serial.print("SCALE_X=");
  Serial.println(SCALE_X);

  Serial.print("SCALE_Y=");
  Serial.println(SCALE_Y);


  dac.setVoltage(SCALE_X, false);

  analogWrite(A0, SCALE_Y);



}
