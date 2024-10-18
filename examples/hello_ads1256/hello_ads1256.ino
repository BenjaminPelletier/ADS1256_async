#include <ADS1256_async.h>
#include <ADS1256_constants.h>

// Pin that should be toggled while capturing data
// Note that some ESP32s do not have the LED_BUILTIN symbol defined in this way
// #define LED_PIN (LED_BUILTIN)
#define LED_PIN (2)
#define BLINK_PERIOD_MS (50)

// Assumes ADS1256 is connected to SPI pins for SCLK, MOSI, and MISO
const uint8_t ADC_PIN_DRDY = 4;
const uint8_t ADC_PIN_CS = 22;
const uint8_t ADC_PIN_RESET = 18;  // This can be either the dedicated RST pin, or the SCLK pin (based on ADC_RESET_MODE below)
const ADS1256ResetMode ADC_RESET_MODE = ADS1256ResetMode::ClockPin;

ADS1256<1> adc(ADC_PIN_DRDY, ADC_PIN_CS, ADC_PIN_RESET, ADC_RESET_MODE);

void setup() {
  delay(2000);

  Serial.begin(115200);
  Serial.println("ADS1256_async: hello_ads1256");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Define desired configuration settings for ADS1256
  // Note that settings with matching default values do not need to be set explicitly
  adc.auto_calibration = true;
  adc.data_rate = DataRate::SPS2;

  // Define multiplexer configurations to cycle through when capturing data
  adc.muxes[0] = mux_of(0);  // AIN0

  while (true) {
    // Initialize ADS1256 (including applying our desired settings)
    Serial.println("Initializing ADS1256...");
    if (adc.blockingInit() != ADS1256Error::None) {
      Serial.print("  Error initializing ADS1256");
      delay(3000);
      continue;
    }

    // Start data capture
    if (adc.beginCapture() != ADS1256Error::None) {
      Serial.println("  Error beginning data capture");
      delay(3000);
      continue;
    }

    break;
  }

  Serial.println("Initialized.");
}

unsigned long last_blink;
bool led_high;

void loop() {
  // Service the ADS1256 state machine
  adc.update();

  // New data being available is indicated with new_data
  if (adc.new_data != NO_NEW_DATA) {
    Serial.println(adc.values[adc.new_data]);
    adc.new_data = NO_NEW_DATA;
  }

  // Blink the LED at a faster rate than the ADS1256 is being sampled
  if (millis() >= last_blink + BLINK_PERIOD_MS) {
    led_high = !led_high;
    digitalWrite(LED_PIN, led_high ? HIGH : LOW);
    while (last_blink < millis()) {
      last_blink += BLINK_PERIOD_MS;
    }
  }
}
