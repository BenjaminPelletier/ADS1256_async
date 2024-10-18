// This example demonstrates basic communication with the ADS1256 without the use of a library.
// If the examples using the library aren't working, this sketch can help verify that the
// ADS1256 and connections from the Arduino are working properly.
//
// To do this, the sketch writes a value to the DRATE register on the ADS1256 and then reads it
// back.  If the ADS1256 is unresponsive, the values 0x0 or 0xFF are expected.
//
// If no ADC_PIN_RESET is specified (set to 0), the user may need to manually reset the ADS1256
// (perhaps by power-cycling it) for this sketch to work properly.

#include <SPI.h>

// Assumes ADS1256 is connected to SPI pins for SCLK, MOSI, and MISO
const uint8_t ADC_PIN_DRDY = 4;
const uint8_t ADC_PIN_CS = 22;
const uint8_t ADC_PIN_RESET = 0;  // Set to pin of RST on ADS1256, or 0 to skip performing a reset (reset to be performed manually by user)
const uint8_t ADC_PIN_SYNC = 0;  // Set to pin of SYNC/PDWN on ADS1256 to be driven HIGH, or 0 to skip driving this pin HIGH

SPISettings spi_settings = SPISettings(1920000, MSBFIRST, SPI_MODE1);
//SPI_MODE1 = output edge: rising, data capture: falling; clock polarity: 0, clock phase: 1.

void setup(void)
{
  delay(2000);

  Serial.begin(115200);
  Serial.println("ADS1256_async: ads1256_no_library");

  SPI.begin();
}

void init_adc() {
  // Set up pins
  pinMode(ADC_PIN_DRDY, INPUT);	
	pinMode(ADC_PIN_CS, OUTPUT);
	digitalWrite(ADC_PIN_CS, HIGH);

  if (ADC_PIN_SYNC) {
    pinMode(ADC_PIN_SYNC, OUTPUT);
    digitalWrite(ADC_PIN_SYNC, HIGH);
  }

  if (ADC_PIN_RESET) {
    // Initiate reset
    pinMode(ADC_PIN_RESET, OUTPUT);
    digitalWrite(ADC_PIN_RESET, LOW);
    delayMicroseconds(5);
    digitalWrite(ADC_PIN_RESET, HIGH); //RESET is set to high
    delayMicroseconds(10);
  }
	
  // Wait for ADS1256 to boot
  while (digitalRead(ADC_PIN_DRDY) == HIGH) {}

  // Write value to DRATE register
  digitalWrite(ADC_PIN_CS, LOW);
  SPI.beginTransaction(spi_settings);
  SPI.transfer(0x53);  // WREG DRATE
  SPI.transfer(0x00);  // 1 register
  SPI.transfer(0x92);  // Set these register bits
  SPI.endTransaction();
  delayMicroseconds(2);
  digitalWrite(ADC_PIN_CS, HIGH);

  // Wait for settings to update
  while (digitalRead(ADC_PIN_DRDY) == HIGH) {}
	
  // Read value from DRATE register
  digitalWrite(ADC_PIN_CS, LOW);
  SPI.beginTransaction(spi_settings);
  SPI.transfer(0x13);  // RREG DRATE
  SPI.transfer(0x00);  // 1 register
  delayMicroseconds(7);
  uint8_t v = SPI.transfer(0xFF);  // Retrieve register bits
  SPI.endTransaction();
  delayMicroseconds(2);
  digitalWrite(ADC_PIN_CS, HIGH);

  // Report observations
  if (v == 0x92) {
    Serial.println("Successfully verified DRATE register contents are as written");
  } else {
    Serial.print("Wrote 0x92 to DRATE register, but read 0x");
    Serial.print(v, HEX);
    Serial.println(" from the DRATE register instead");
  }
}

void loop(void)
{
  Serial.println("Initializing and interrogating ADS1256");
  init_adc();
  delay(5000);
}
