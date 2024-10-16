#include <ADS1256_async.h>
#include <ADS1256_constants.h>
#include <ADS1256_diagnostics.h>

// Assumes ADS1256 is connected to SPI pins for SCLK, MOSI, and MISO
const uint8_t ADC_DRDY = 4;
const uint8_t ADC_RESET = 18;  // This can be either the dedicated RST pin, or the SCLK pin (based on ADC_RESET_MODE below)
const uint8_t ADC_SYNC = NO_PIN;
const uint8_t ADC_CS = 2;
const ADS1256ResetMode ADC_RESET_MODE = ADS1256ResetMode::ClockPin;

#define N_CHANNELS 5
ADS1256<N_CHANNELS> adc_r(SPI, ADC_CS, ADC_DRDY, ADC_RESET, ADC_SYNC, ADC_RESET_MODE);

unsigned long last_print;

void setup() {
  delay(2000);

  Serial.begin(115200);
  Serial.println("ADS1256_async: init_and_capture");

  // Define desired configuration settings for ADS1256
  // Note that settings with matching default values do not need to be set explicitly
  adc_r.auto_calibration = true;
  adc_r.buffer = true;
  adc_r.clock_out = ClockOut::Off;
  adc_r.data_rate = DataRate::SPS30000;
  adc_r.gain = Gain::X1;
  adc_r.lsb_first = false;
  adc_r.sensor_detect = SDCS::Off;

  // Define multiplexer configurations to cycle through when capturing data
  adc_r.muxes[0] = mux_of(0);  // AIN0
  adc_r.muxes[1] = mux_of(0, 1);  // AIN1-AIN0
  adc_r.muxes[2] = mux_of(2, 3);  // AIN3-AIN2
  adc_r.muxes[3] = mux_of(4, 5);  // AIN5-AIN4
  adc_r.muxes[4] = mux_of(6, 7);  // AIN7-AIN6

  Serial.println("Initializing ADS1256...");

  // Begin asynchronous reset
  unsigned long t0 = micros();
  ADS1256Error result = adc_r.beginReset();
  if (result != ADS1256Error::None) {
    Serial.print("Unable to begin ADS1256 reset: ");
    Serial.println(name_of(result));
    while (true) {}
  }

  // Wait for reset to complete
  last_print = millis();
  while (adc_r.state() != ADS1256State::Idle) {
    if (millis() > last_print + 1000) {
      Serial.print("  Still resetting ADS1256 (");
      Serial.print(name_of(adc_r.state()));
      Serial.println(")...");
      last_print = millis();
    }
    adc_r.update();
  }
  unsigned long dt = micros() - t0;
  Serial.print("  Complete in ");
  Serial.print(dt);
  Serial.println("us");

  // Write our desired settings
  Serial.println("Writing ADS1256 settings...");
  t0 = micros();
  result = adc_r.beginWriteSettings();
  if (result != ADS1256Error::None) {
    Serial.print("Unable to begin writing ADS1256 settings: ");
    Serial.println(name_of(result));
    while (true) {}
  }

  // Wait for settings-write to complete (includes auto calibration)
  last_print = millis();
  while (adc_r.state() != ADS1256State::Idle) {
    if (millis() > last_print + 1000) {
      Serial.print("  Still writing ADS1256 settings(");
      Serial.print(name_of(adc_r.state()));
      Serial.println(")...");
      last_print = millis();
    }
    adc_r.update();
  }
  dt = micros() - t0;
  Serial.print("  Complete in ");
  Serial.print(dt);
  Serial.println("us");

  // Verify that the settings were written correctly by reading them back
  result = adc_r.readSettings(true);
  if (result != ADS1256Error::None) {
    Serial.print("Unable to verify ADS1256 settings: ");
    Serial.println(name_of(result));
    while (true) {}
  }

  Serial.println("Initialized.");

  // Display configuration information
  Serial.println("ADS1256 configuration:");
  print_configuration(adc_r);
}

void loop() {
  delay(5000);
  Serial.println("=== Starting capture iteration ===");

  // Perform a full initialization in one blocking step
  // This is an alternative to the asynchronous initialization demonstrated in setup()
  Serial.print("blockingInit ");
  unsigned long t0 = micros();
  ADS1256Error result = adc_r.blockingInit();
  unsigned long dt = micros() - t0;
  if (result != ADS1256Error::None) {
    Serial.println(name_of(result));
    return;
  }
  Serial.print("success in ");
  Serial.print(dt);
  Serial.println(" us");

  // Kick off continuous asynchronous data capture
  Serial.println("Capturing...");
  result = adc_r.beginCapture();
  if (result != ADS1256Error::None) {
    Serial.print("  Error beginning capture: ");
    Serial.println(name_of(result));
    return;
  }

  // Capture continuously for 3 seconds
  const unsigned long DURATION_MS = 3000;
  unsigned long t1 = millis() + DURATION_MS;
  uint32_t n[N_CHANNELS] = {0, 0, 0, 0};
  while (millis() < t1) {
    adc_r.update();
    if (adc_r.new_data != NO_NEW_DATA) {
      n[adc_r.new_data]++;
      adc_r.new_data = NO_NEW_DATA;
    }
    // Note that other activities can be performed here (in between calls to update)
  }

  // Show information about the data captured
  Serial.print("Samples captured:");
  uint32_t n_total = 0;
  for (uint8_t c = 0; c < N_CHANNELS; c++) {
    Serial.print(" ");
    Serial.print(n[c]);
    n_total += n[c];
  }
  Serial.print(" (");
  Serial.print(n_total);
  Serial.print(" total, ");
  Serial.print((float)n_total * 1000 / DURATION_MS);
  Serial.println(" samples/sec)");
  Serial.print("Most recent values:");
  for (uint8_t c = 0; c < N_CHANNELS; c++) {
    Serial.print(" ");
    Serial.print(adc_r.values[c]);
  }
  Serial.println();
}
