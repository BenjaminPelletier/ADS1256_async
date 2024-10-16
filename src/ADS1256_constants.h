#ifndef ADS1256_CONSTANTS_H
#define ADS1256_CONSTANTS_H

// Commands
#define CMD_WAKEUP (0b00000000)
#define CMD_RDATA (0b00000001)
#define CMD_RDATAC (0b00000011)
#define CMD_SDATAC (0b00001111)
#define CMD_RREG (0b00010000)
#define CMD_WREG (0b01010000)
#define CMD_SELFCAL (0b11110000)
#define CMD_SELFOCAL (0b11110001)
#define CMD_SELFGCAL (0b11110010)
#define CMD_SYSOCAL (0b11110011)
#define CMD_SYSGCAL (0b11110100)
#define CMD_SYNC (0b11111100)
#define CMD_STANDBY (0b11111101)
#define CMD_RESET (0b11111110)

enum class Command : uint8_t {
	WAKEUP = CMD_WAKEUP,
	RDATA = CMD_RDATA,
	RDATAC = CMD_RDATAC,
	SDATAC = CMD_SDATAC,
	RREG = CMD_RREG,
	WREG = CMD_WREG,
	SELFCAL = CMD_SELFCAL,
	SELFOCAL = CMD_SELFOCAL,
	SELFGCAL = CMD_SELFGCAL,
	SYSOCAL = CMD_SYSOCAL,
	SYSGCAL = CMD_SYSGCAL,
	SYNC = CMD_SYNC,
	STANDBY = CMD_STANDBY,
	RESET = CMD_RESET,
};

// Registers
#define REG_STATUS (0x00)
#define REG_MUX (0x01)
#define REG_ADCON (0x02)
#define REG_DRATE (0x03)
#define REG_IO (0x04)
#define REG_OFC0 (0x05)
#define REG_OFC1 (0x06)
#define REG_OFC2 (0x07)
#define REG_FSC0 (0x08)
#define REG_FSC1 (0x09)
#define REG_FSC2 (0x0A)

enum class Register : uint8_t {
	STATUS = REG_STATUS,
	MUX = REG_MUX,
	ADCON = REG_ADCON,
	DRATE = REG_DRATE,
	IO = REG_IO,
	OFC0 = REG_OFC0,
	OFC1 = REG_OFC1,
	OFC2 = REG_OFC2,
	FSC0 = REG_FSC0,
	FSC1 = REG_FSC1,
	FSC2 = REG_FSC2,
};

// Status register
#define STATUS_ID (4)

#define STATUS_ORDER (3)
#define STATUS_ORDER_MSB (0 << STATUS_ORDER)
#define STATUS_ORDER_LSB (1 << STATUS_ORDER)

#define STATUS_ACAL (2)
#define STATUS_ACAL_DISABLED (0 << STATUS_ACAL)
#define STATUS_ACAL_ENABLED (1 << STATUS_ACAL)

#define STATUS_BUFEN (1)
#define STATUS_BUFFER_DISABLED (0 << STATUS_BUFEN)
#define STATUS_BUFFER_ENABLED (1 << STATUS_BUFEN)

#define STATUS_NDRDY (0)

#define STATUS_WRITEMASK ((1 << STATUS_ORDER) | (1 << STATUS_ACAL) | (1 << STATUS_BUFEN))

// Multiplexer register
#define MUX_PSEL (4)
#define MUX_NSEL (0)

#define MUX_AIN0 (0b0000)
#define MUX_AIN1 (0b0001)
#define MUX_AIN2 (0b0010)
#define MUX_AIN3 (0b0011)
#define MUX_AIN4 (0b0100)
#define MUX_AIN5 (0b0101)
#define MUX_AIN6 (0b0110)
#define MUX_AIN7 (0b0111)
#define MUX_AINCOM (0b1000)

inline
uint8_t mux_of(uint8_t negative_input, uint8_t positive_input) {
	return (positive_input << MUX_PSEL) | negative_input;
}

inline
uint8_t mux_of(uint8_t positive_input) {
	return mux_of(MUX_AINCOM, positive_input);
}

// A/D control register
#define ADCON_CLK (5)
#define ADCON_CLK_OFF (0b00 << ADCON_CLK)
#define ADCON_CLK_FCLK (0b01 << ADCON_CLK)
#define ADCON_CLK_FCLK2 (0b10 << ADCON_CLK)
#define ADCON_CLK_FCLK4 (0b11 << ADCON_CLK)
#define ADCON_CLK_MASK (0b11 << ADCON_CLK)

enum class ClockOut : uint8_t {
	Off = ADCON_CLK_OFF,
	FClk = ADCON_CLK_FCLK,
	FClkDiv2 = ADCON_CLK_FCLK2,
	FClkDiv4 = ADCON_CLK_FCLK4,
};

#define ADCON_SDCS (3)
#define ADCON_SDCS_OFF (0b00 << ADCON_SDCS)
#define ADCON_SDCS_05UA (0b01 << ADCON_SDCS)
#define ADCON_SDCS_2UA (0b10 << ADCON_SDCS)
#define ADCON_SDCS_10UA (0b11 << ADCON_SDCS)
#define ADCON_SDCS_MASK (0b11 << ADCON_SDCS)

enum class SDCS : uint8_t {
	Off = ADCON_SDCS_OFF,
	SDC05uA = ADCON_SDCS_05UA,
	SDC2uA = ADCON_SDCS_2UA,
	SDC10uA = ADCON_SDCS_10UA,
};

#define ADCON_PGA (0)
#define ADCON_PGA_1X (0b000 << ADCON_PGA)
#define ADCON_PGA_2X (0b001 << ADCON_PGA)
#define ADCON_PGA_4X (0b010 << ADCON_PGA)
#define ADCON_PGA_8X (0b011 << ADCON_PGA)
#define ADCON_PGA_16X (0b100 << ADCON_PGA)
#define ADCON_PGA_32X (0b101 << ADCON_PGA)
#define ADCON_PGA_64X (0b110 << ADCON_PGA)
#define ADCON_PGA_MASK (0b111 << ADCON_PGA)

enum class Gain : uint8_t {
	X1 = ADCON_PGA_1X,
	X2 = ADCON_PGA_2X,
	X4 = ADCON_PGA_4X,
	X8 = ADCON_PGA_8X,
	X16 = ADCON_PGA_16X,
	X32 = ADCON_PGA_32X,
	X64 = ADCON_PGA_64X,
};

#define ADCON_WRITEMASK ((0b111 << ADCON_PGA) | (0b11 << ADCON_SDCS) | (0b11 << ADCON_CLK))

// A/D data rate register
#define DRATE_30000SPS (0b11110000)
#define DRATE_15000SPS (0b11100000)
#define DRATE_7500SPS (0b11010000)
#define DRATE_3750SPS (0b11000000)
#define DRATE_2000SPS (0b10110000)
#define DRATE_1000SPS (0b10100001)
#define DRATE_500SPS (0b10010010)
#define DRATE_100SPS (0b10000010)
#define DRATE_60SPS (0b01110010)
#define DRATE_50SPS (0b01100011)
#define DRATE_30SPS (0b01010011)
#define DRATE_25SPS (0b01000011)
#define DRATE_15SPS (0b00110011)
#define DRATE_10SPS (0b00100011)
#define DRATE_5SPS (0b00010011)
#define DRATE_2SPS (0b00000011)

enum class DataRate : uint8_t {
	SPS30000 = DRATE_30000SPS,
	SPS15000 = DRATE_15000SPS,
	SPS7500 = DRATE_7500SPS,
	SPS3750 = DRATE_3750SPS,
	SPS2000 = DRATE_2000SPS,
	SPS1000 = DRATE_1000SPS,
	SPS500 = DRATE_500SPS,
	SPS100 = DRATE_100SPS,
	SPS60 = DRATE_60SPS,
	SPS50 = DRATE_50SPS,
	SPS30 = DRATE_30SPS,
	SPS25 = DRATE_25SPS,
	SPS15 = DRATE_15SPS,
	SPS10 = DRATE_10SPS,
	SPS5 = DRATE_5SPS,
	SPS2 = DRATE_2SPS,
};

#endif
