#ifndef ADS1256_DIAGNOSTICS
#define ADS1256_DIAGNOSTICS

#include "ADS1256_async.h"

String name_of(ADS1256State state) {
	switch (state) {
		case ADS1256State::Uninitialized:
			return "Uninitialized";
		case ADS1256State::Resetting:
			return "Resetting";
		case ADS1256State::WritingSettings:
			return "WritingSettings";
		case ADS1256State::Idle:
			return "Idle";
		case ADS1256State::Capturing:
			return "Capturing";
		case ADS1256State::FinishingCapture:
			return "FinishingCapture";
		default:
			return "unknown";
	}
}

String name_of(ADS1256Error error) {
	switch(error) {
		case ADS1256Error::None:
			return "None";
		case ADS1256Error::SettingsOutOfSync:
			return "SettingsOutOfSync";
		case ADS1256Error::NotReadyToWriteSettings:
			return "NotReadyToWriteSettings";
		case ADS1256Error::NotReadyToReadSettings:
			return "NotReadyToReadSettings";
		case ADS1256Error::ResetMethodNotValid:
			return "ResetMethodNotValid";
		case ADS1256Error::TimeoutWhileResetting:
			return "TimeoutWhileResetting";
		case ADS1256Error::TimeoutWhileWritingSettings:
			return "TimeoutWhileWritingSettings";
		case ADS1256Error::NotReadyToBeginCapture:
			return "NotReadyToBeginCapture";
		case ADS1256Error::CannotEndWhenNotCapturing:
			return "CannotEndWhenNotCapturing";
		case ADS1256Error::CanOnlyWriteSettingsWhenIdle:
			return "CanOnlyWriteSettingsWhenIdle";
		case ADS1256Error::CanOnlyReadSettingsWhenIdle:
			return "CanOnlyReadSettingsWhenIdle";
		case ADS1256Error::CanOnlyBeginCaptureWhenIdle:
			return "CanOnlyBeginCaptureWhenIdle";
		default:
			return "unknown";
	}
}

String name_of_mux(uint8_t mux) {
	uint8_t n = mux & 0b00001111;
	uint8_t p = mux >> 4;
	if (n > MUX_AINCOM || p > MUX_AINCOM) {
		return "unknown";
	} else if (n == MUX_AINCOM) {
		return "AIN" + String(p);
	} else if (p == MUX_AINCOM) {
		return "-AIN" + String(n);
	} else if (n == MUX_AINCOM && p == MUX_AINCOM) {
		return "nothing";
	} else {
		return "AIN" + String(p) + "-AIN" + String(n);
	}
}

String name_of(ClockOut clock_out) {
	switch(clock_out) {
		case ClockOut::Off:
			return "Off";
		case ClockOut::FClk:
			return "FCLK";
		case ClockOut::FClkDiv2:
			return "FCLK/2";
		case ClockOut::FClkDiv4:
			return "FCLK/4";
		default:
			return "unknown";
	}
}

String name_of(DataRate data_rate) {
	switch(data_rate) {
		case DataRate::SPS30000:
			return "30000 samples/sec";
		case DataRate::SPS15000:
			return "15000 samples/sec";
		case DataRate::SPS7500:
			return "7500 samples/sec";
		case DataRate::SPS3750:
			return "3750 samples/sec";
		case DataRate::SPS2000:
			return "2000 samples/sec";
		case DataRate::SPS1000:
			return "1000 samples/sec";
		case DataRate::SPS500:
			return "500 samples/sec";
		case DataRate::SPS100:
			return "100 samples/sec";
		case DataRate::SPS60:
			return "60 samples/sec";
		case DataRate::SPS50:
			return "50 samples/sec";
		case DataRate::SPS30:
			return "30 samples/sec";
		case DataRate::SPS25:
			return "25 samples/sec";
		case DataRate::SPS15:
			return "15 samples/sec";
		case DataRate::SPS10:
			return "10 samples/sec";
		case DataRate::SPS5:
			return "5 samples/sec";
		case DataRate::SPS2:
			return "2 samples/sec";
		default:
			return "unknown";
	}
}

String name_of(Gain gain) {
	switch(gain) {
		case Gain::X1:
			return "1x";
		case Gain::X2:
			return "2x";
		case Gain::X4:
			return "4x";
		case Gain::X8:
			return "8x";
		case Gain::X16:
			return "16x";
		case Gain::X32:
			return "32x";
		case Gain::X64:
			return "64x";
		default:
			return "unknown";
	}
}

String name_of(SDCS sdcs) {
	switch(sdcs) {
		case SDCS::Off:
			return "Off";
		case SDCS::SDC05uA:
			return "0.5 uA";
		case SDCS::SDC2uA:
			return "2 uA";
		case SDCS::SDC10uA:
			return "10 uA";
	}
}

template<uint8_t nCycledChannels>
void print_configuration(ADS1256<nCycledChannels>& adc, Stream& serial) {
  serial.print("  Auto calibration ");
  serial.println(adc.auto_calibration ? "enabled" : "disabled");
  serial.print("  Analog input buffer ");
  serial.println(adc.buffer ? "enabled" : "disabled");
  serial.print("  Clock output: ");
  serial.println(name_of(adc.clock_out));
  serial.print("  Data rate: ");
  serial.println(name_of(adc.data_rate));
  serial.print("  Gain: ");
  serial.println(name_of(adc.gain));
  serial.print(adc.lsb_first ? "  Least" : "  Most");
  serial.println(" significant bit first");
  serial.print("  Sensor detect current sources: ");
  serial.println(name_of(adc.sensor_detect));
}

template<uint8_t nCycledChannels>
void print_configuration(ADS1256<nCycledChannels>& adc) {
	print_configuration(adc, Serial);
}

#endif
