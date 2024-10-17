#ifndef ADS1256_ASYNC_H
#define ADS1256_ASYNC_H

#include <Arduino.h>
#include <SPI.h>

#include "ADS1256_constants.h"

#define NO_PIN (255)
#define NO_MUX (255)
#define NO_NEW_DATA (255)
#define IRRELEVANT (0xFF)
#define DEFAULT_TIMEOUT_MS (10)

enum class ADS1256ResetMode : uint8_t {
	Undefined,
	ControlPin,
	ClockPin,
};

enum class ADS1256State : uint8_t {
	Uninitialized,
	Resetting,
	WritingSettings,
	Idle,
	Capturing,
	FinishingCapture,
};

enum class ADS1256Error : uint8_t {
	None,
	SettingsOutOfSync,
	NotReadyToWriteSettings,
	NotReadyToReadSettings,
	ResetMethodNotValid,
	TimeoutWhileResetting,
	TimeoutWhileWritingSettings,
	NotReadyToBeginCapture,
	CannotEndWhenNotCapturing,
	CanOnlyWriteSettingsWhenIdle,
	CanOnlyReadSettingsWhenIdle,
	CanOnlyBeginCaptureWhenIdle,
};

template<uint8_t nCycledChannels>
class ADS1256 {
  public:
	ADS1256(
		SPIClass& spi,
		const uint8_t pin_cs,
		const uint8_t pin_drdy,
		const uint8_t pin_reset,
		const uint8_t pin_sync,
		const ADS1256ResetMode reset_mode
	) : spi_(spi),
	    pin_cs_(pin_cs),
		pin_drdy_(pin_drdy),
		pin_reset_(pin_reset),
		pin_sync_(pin_sync),
		reset_mode_(reset_mode)
	{}
	
	ADS1256(
		const uint8_t pin_cs,
		const uint8_t pin_drdy,
		const uint8_t pin_reset,
		const uint8_t pin_sync,
		const ADS1256ResetMode reset_mode
	) : ADS1256(SPI, pin_cs, pin_drdy, pin_reset, pin_sync, reset_mode)
	{}
	
	ADS1256(
		const uint8_t pin_cs,
		const uint8_t pin_drdy,
		const uint8_t pin_reset,
		const ADS1256ResetMode reset_mode
	) : ADS1256(pin_cs, pin_drdy, pin_reset, NO_PIN, reset_mode)
	{}
	
	ADS1256(
		const uint8_t pin_cs,
		const uint8_t pin_drdy
	) : ADS1256(pin_cs, pin_drdy, NO_PIN, NO_PIN, ADS1256ResetMode::Undefined)
	{}
	
	// Default SPI settings may be overridden
	// SPI_MODE1 = output edge: rising, data capture: falling; clock polarity: 0, clock phase: 1.
	SPISettings spi_settings = SPISettings(1920000, MSBFIRST, SPI_MODE1);
	
	bool lsb_first = false;
	bool auto_calibration = false;
	bool buffer = false;
	ClockOut clock_out = ClockOut::Off;
	SDCS sensor_detect = SDCS::Off;
	Gain gain = Gain::X1;
	DataRate data_rate = DataRate::SPS2;
	
	bool settings_out_of_sync = false;
	
	uint8_t muxes[nCycledChannels];
	uint8_t next_mux = 0;
	int32_t values[nCycledChannels];
	uint8_t new_data = NO_NEW_DATA;
	
	inline ADS1256State state() {
		return state_;
	}
	
	void update();
	
	void writeRegisters(Register first_register, uint8_t n_registers, const uint8_t* values);
	
	void readRegisters(Register first_register, uint8_t n_registers, uint8_t* values);
	
	ADS1256Error beginReset();
	
	ADS1256Error beginWriteSettings(int16_t timeout_ms = DEFAULT_TIMEOUT_MS);
	
	ADS1256Error readSettings(bool update_local_settings, int16_t timeout_ms = DEFAULT_TIMEOUT_MS);
	
	ADS1256Error blockingInit(int16_t timeout_ms = 1000);
	
	ADS1256Error beginCapture(int16_t timeout_ms = DEFAULT_TIMEOUT_MS);
	
	void continueCapture();
	
	ADS1256Error endCapture();
	
  private:
	SPIClass& spi_;
	ADS1256ResetMode reset_mode_;
	
	uint8_t pin_cs_;
	uint8_t pin_drdy_;
	uint8_t pin_reset_;
	uint8_t pin_sync_;
	
	ADS1256State state_;
	
	uint8_t current_mux_ = NO_MUX;
	
	inline uint8_t getStatusRegisterValue() {
		return (lsb_first ? STATUS_ORDER_LSB : STATUS_ORDER_MSB) |
			(auto_calibration ? STATUS_ACAL_ENABLED : STATUS_ACAL_DISABLED) |
			(buffer ? STATUS_BUFFER_ENABLED : STATUS_BUFFER_DISABLED);
	}
	
	inline uint8_t getMuxRegisterValue() {
		return muxes[current_mux_ == NO_MUX ? next_mux : current_mux_];
	}
	
	inline uint8_t getControlRegisterValue() {
		return (uint8_t)clock_out |
			(uint8_t)sensor_detect |
			(uint8_t)gain;
	}
	
	// Note: assumed clock frequency of 7.68 MHz for delay_* below
	inline void delay_t6() {
		delayMicroseconds(7); // t6: at least 50 clock periods
	}
	inline void delay_t10() {
		delayMicroseconds(2); // t10: at least 8 clock periods
	}
	inline void delay_t11_short() {
		delayMicroseconds(1);  // t11 (RREG, WREG, RDATA): at least 4 clock periods
	}
	inline void delay_t11_long() {
		delayMicroseconds(4);  // t11 (RDATAC, SYNC): at least 24 clock periods
	}
	inline void delay_t12() {
		delayMicroseconds(52);  // t12: 300-500 clock periods
	}
	inline void delay_t13() {
		delayMicroseconds(1);  // t13: at least 5 clock periods
	}
	inline void delay_t14() {
		delayMicroseconds(85);  // t14: 550-750 clock periods
	}
	inline void delay_t15() {
		delayMicroseconds(150);  // t15: 1050-1250 clock periods
	}
};


template<uint8_t nCycledChannels>
void ADS1256<nCycledChannels>::update() {
	switch (state_) {
		case ADS1256State::Resetting:
			if (digitalRead(pin_drdy_) == LOW) {
				state_ = ADS1256State::Idle;
			}
			break;
		case ADS1256State::WritingSettings:
			if (digitalRead(pin_drdy_) == LOW) {
				state_ = ADS1256State::Idle;
			}
			break;
		case ADS1256State::Capturing:
		case ADS1256State::FinishingCapture:
			if (digitalRead(pin_drdy_) == LOW) {
				continueCapture();
			}
		default:
			// Nothing to do
			break;
	}
}

template<uint8_t nCycledChannels>
ADS1256Error ADS1256<nCycledChannels>::beginReset() {
	// Set up pins
	pinMode(pin_drdy_, INPUT);	
	pinMode(pin_reset_, OUTPUT);
	pinMode(pin_sync_, OUTPUT);
	pinMode(pin_cs_, OUTPUT);
	digitalWrite(pin_sync_, HIGH);
	digitalWrite(pin_cs_, HIGH);

	if (reset_mode_ == ADS1256ResetMode::ControlPin && pin_reset_ != NO_PIN) {
		// Initiate ADS1256 reset via control pin
		digitalWrite(pin_reset_, LOW);
		delayMicroseconds(5);
		digitalWrite(pin_reset_, HIGH);
	} else if (reset_mode_ == ADS1256ResetMode::ClockPin && pin_reset_ != NO_PIN) {
		// Initiate ADS1256 reset via SCLK signaling
		spi_.end();  // Make sure we can control the pin (ok if begin has not yet been called)
		uint8_t sclk = pin_reset_;
		digitalWrite(sclk, HIGH);
		delay_t12();
		digitalWrite(sclk, LOW);
		delay_t13();
		digitalWrite(sclk, HIGH);
		delay_t14();
		digitalWrite(sclk, LOW);
		delay_t13();
		digitalWrite(sclk, HIGH);
		delay_t15();
		digitalWrite(sclk, LOW);
	} else {
		return ADS1256Error::ResetMethodNotValid;
	}
	
	// Begin SPI (repeated calls are ok if something else begins SPI as well)
	spi_.begin();
	
	state_ = ADS1256State::Resetting;
	return ADS1256Error::None;
}

template<uint8_t nCycledChannels>
void ADS1256<nCycledChannels>::writeRegisters(Register first_register, uint8_t n_registers, const uint8_t* values) {
  digitalWrite(pin_cs_, LOW);
  spi_.beginTransaction(spi_settings);
  spi_.transfer(CMD_WREG | (uint8_t)first_register);
  spi_.transfer(n_registers - 1);
  for (uint8_t r = 0; r < n_registers; r++) {
	spi_.transfer(values[r]);
  }
  spi_.endTransaction();
  delay_t10();
  digitalWrite(pin_cs_, HIGH);
}

template<uint8_t nCycledChannels>
void ADS1256<nCycledChannels>::readRegisters(Register first_register, uint8_t n_registers, uint8_t* values) {
  digitalWrite(pin_cs_, LOW);
  spi_.beginTransaction(spi_settings);
  spi_.transfer(CMD_RREG | (uint8_t)first_register);
  spi_.transfer(n_registers - 1);
  delay_t6();
  for (uint8_t r = 0; r < n_registers; r++) {
	values[r] = spi_.transfer(IRRELEVANT);
  }
  spi_.endTransaction();
  delay_t10();
  digitalWrite(pin_cs_, HIGH);
}

template<uint8_t nCycledChannels>
ADS1256Error ADS1256<nCycledChannels>::beginWriteSettings(int16_t timeout_ms) {
	if (state_ != ADS1256State::Idle) {
		return ADS1256Error::CanOnlyWriteSettingsWhenIdle;
	}
	unsigned long t1 = millis() + timeout_ms;
	while (digitalRead(pin_drdy_) == HIGH) {
		if (millis() > t1) {
			return ADS1256Error::NotReadyToWriteSettings;
		}
	}

	// Write settings to registers
	const uint8_t N_SETTINGS_REGISTERS = 4;
	uint8_t values[N_SETTINGS_REGISTERS] = {
		getStatusRegisterValue(),
		getMuxRegisterValue(),
		getControlRegisterValue(),
		(uint8_t)data_rate,
	};
	writeRegisters(Register::STATUS, N_SETTINGS_REGISTERS, values);

	state_ = ADS1256State::WritingSettings;
	return ADS1256Error::None;
}

template<uint8_t nCycledChannels>
ADS1256Error ADS1256<nCycledChannels>::readSettings(bool update_local_settings, int16_t timeout_ms) {
		if (state_ != ADS1256State::Idle) {
		return ADS1256Error::CanOnlyReadSettingsWhenIdle;
	}
	unsigned long t1 = millis() + timeout_ms;
	while (digitalRead(pin_drdy_) == HIGH) {
		if (millis() > t1) {
			return ADS1256Error::NotReadyToReadSettings;
		}
	}

	// Read register values
	uint8_t values[4];
	readRegisters(Register::STATUS, 4, values);
	
	if (update_local_settings) {
		// Translate register values into field data
		lsb_first = values[0] & STATUS_ORDER_LSB;
		auto_calibration = values[0] & STATUS_ACAL_ENABLED;
		buffer = values[0] & STATUS_BUFFER_ENABLED;
		muxes[0] = values[1];
		clock_out = (ClockOut)(values[2] & ADCON_CLK_MASK);
		sensor_detect = (SDCS)(values[2] & ADCON_SDCS_MASK);
		gain = (Gain)(values[2] & ADCON_PGA_MASK);
		data_rate = (DataRate)values[3];
	} else {
		// Check if values match local settings
		ADS1256Error result = ADS1256Error::None;
		if ((values[0] & STATUS_WRITEMASK) != getStatusRegisterValue() ||
			values[1] != getMuxRegisterValue() ||
			(values[2] & ADCON_WRITEMASK) != getControlRegisterValue() ||
			values[3] != (uint8_t)data_rate
		   ) {
			return ADS1256Error::SettingsOutOfSync;
		}
	}
	
	return ADS1256Error::None;
}

template<uint8_t nCycledChannels>
ADS1256Error ADS1256<nCycledChannels>::blockingInit(int16_t timeout_ms) {
	ADS1256Error result;
	unsigned long t0 = millis();
	result = beginReset();
	if (result != ADS1256Error::None) {
		return result;
	}

	while (state_ != ADS1256State::Idle) {
		if (millis() - t0 > timeout_ms) {
			return ADS1256Error::TimeoutWhileResetting;
		}
		update();
	}

	result = beginWriteSettings(t0 + timeout_ms - millis());
	if (result != ADS1256Error::None) {
		return result;
	}

	while (state_ != ADS1256State::Idle) {
		if (millis() - t0 > timeout_ms) {
			return ADS1256Error::TimeoutWhileWritingSettings;
		}
		update();
	}

	result = readSettings(true, t0 + timeout_ms - millis());
	if (result != ADS1256Error::None) {
		return result;
	}
	
	return ADS1256Error::None;
}

template<uint8_t nCycledChannels>
ADS1256Error ADS1256<nCycledChannels>::beginCapture(int16_t timeout_ms) {
	if (state_ != ADS1256State::Idle) {
		return ADS1256Error::CanOnlyBeginCaptureWhenIdle;
	}
	unsigned long t1 = millis() + timeout_ms;
	while (digitalRead(pin_drdy_) == HIGH) {
		if (millis() > t1) {
			return ADS1256Error::NotReadyToBeginCapture;
		}
	}
	state_ = ADS1256State::Capturing;
	continueCapture();
	return ADS1256Error::None;
}

template<uint8_t nCycledChannels>
void ADS1256<nCycledChannels>::continueCapture() {
	digitalWrite(pin_cs_, LOW);
	spi_.beginTransaction(spi_settings);
	
	uint8_t this_mux = current_mux_;
	if (state_ == ADS1256State::Capturing) {
		// Retarget mulitplexer and begin the next conversion
		spi_.transfer(CMD_WREG | REG_MUX);
		spi_.transfer(0);  // Write 1 register
		spi_.transfer(muxes[next_mux]);
		delay_t11_short();
		
		spi_.transfer(CMD_SYNC);
		delay_t11_long();
		
		spi_.transfer(CMD_WAKEUP);
		current_mux_ = next_mux;
		next_mux++;
		if (next_mux >= nCycledChannels) {
			next_mux = 0;
		}
	} else if (state_ == ADS1256State::FinishingCapture) {
		// Do not begin a new conversion
		current_mux_ = NO_MUX;
		state_ = ADS1256State::Idle;
	}
	
	if (this_mux != NO_MUX) {
		// Read the measurement from the previous converstion
		spi_.transfer(CMD_RDATA);
		delay_t6();
	
		values[this_mux] = (int32_t)spi_.transfer(IRRELEVANT) << 16;
		values[this_mux] |= (int32_t)spi_.transfer(IRRELEVANT) << 8;
		values[this_mux] |= spi_.transfer(IRRELEVANT);
		if (values[this_mux] & (1 << 23)) {
			// Extend two's complement into 32 bits (from 24)
			values[this_mux] |= 0xFF000000;
		}
		new_data = this_mux;
	}
	
	spi_.endTransaction();
	
	delay_t10();
	digitalWrite(pin_cs_, HIGH);
}

template<uint8_t nCycledChannels>
ADS1256Error ADS1256<nCycledChannels>::endCapture() {
	if (state_ != ADS1256State::Capturing) {
		return ADS1256Error::CannotEndWhenNotCapturing;
	}
	state_ = ADS1256::FinishingCapture;
	return ADS1256Error::None;
}

#endif
