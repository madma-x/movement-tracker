#include "PAA5163.h"
#include "PAA5163_registers.h"
#include "main.h"

#include "micros.h"

#include <stdio.h>
#include <stdbool.h>

#define SPI_TIMEOUT HAL_MAX_DELAY
#define OBSERVATION_INIT_RETRY 10

#define IN_TO_MM 25.4

// ================= Hardware abstraction ================= //

static inline void _select(paa5163_t* p){
	HAL_GPIO_WritePin(p->NCS_Port, p->NCS_Pin, GPIO_PIN_RESET);
}

static inline void _deselect(paa5163_t* p){
	HAL_GPIO_WritePin(p->NCS_Port, p->NCS_Pin, GPIO_PIN_SET);
}

static uint8_t _paaRead(paa5163_t* p, paa5163_registers_t addr){
	uint8_t buf = addr & 0x7F;
	uint8_t ret = 0;

	_select(p);
	delay_us(1); // should be 120 ns
	HAL_SPI_Transmit(p->spi, &buf, 1, SPI_TIMEOUT);
	delay_us(2); // should be 2 us
	HAL_SPI_Receive(p->spi, &ret, 1, SPI_TIMEOUT);
	delay_us(1); // should be 120 ns
	_deselect(p);
	delay_us(5); // tsww or tswr, taking no risk

	return ret;
}

static void _paaWrite(paa5163_t* p, paa5163_registers_t addr, uint8_t data){
	uint8_t buf[2] = {addr | 0x80, data};

	_select(p);
	delay_us(1); // should be 120 ns
	HAL_SPI_Transmit(p->spi, buf, 2, SPI_TIMEOUT);
	delay_us(1); // should be 120 ns
	_deselect(p);
	delay_us(5); // tsww or tswr, taking no risk
}

// ================= Low level ================= //

/* Default orientation :
 * 		x-
 *		|
 * y- --+-- y+
 * 		|
 * 		x+
 */
static void _paaSetOrientation(paa5163_t* p, bool swap_axies, bool invert_x, bool invert_y){ // swaping is done before flipping
	uint8_t orient_reg = 0x00 | ((swap_axies & 0x01) << 0) | ((invert_x & 0x01) << 2) | ((invert_y & 0x01) << 1);

	_paaWrite(p, orientation, orient_reg);
}

static void _paaSetResolution(paa5163_t* p, uint16_t res_cpi){ // between 100 and 20 000, by increments of 100
	if(res_cpi < 100) res_cpi = 100;
	if(res_cpi > 20000) res_cpi = 20000;

	uint8_t reg = (res_cpi/100) -1;

	p->resolution = (reg + 1) * 100; // compensate any rounding done by calculation of reg

	_paaWrite(p, resolution_y_lower, reg);
	_paaWrite(p, resolution_y_upper, 0x00);
	_paaWrite(p, resolution_x_lower, reg);
	_paaWrite(p, resolution_x_upper, 0x00);

	_paaWrite(p, set_resolution, 0x01);
}

static bool _paaMotion(paa5163_t* p){
	return (_paaRead(p, motion) & 0x80);
}

static void _paaPerfOpti(paa5163_t* p){ 	// Check section 7.1.2 of PAA5160's datasheet : Performance Optimization Setting
	_paaWrite(p, 0x3A, 0x5A); // 1
	_paaWrite(p, 0x7F, 0x00); // 2
	_paaWrite(p, 0x40, 0x80); // 3
	_paaWrite(p, 0x7F, 0x14); // 4
	_paaWrite(p, 0x4D, 0x00); // 5
	_paaWrite(p, 0x53, 0x0D); // 6
	_paaWrite(p, 0x4B, 0x20); // 7
	_paaWrite(p, 0x42, 0xBC); // 8
	_paaWrite(p, 0x43, 0x74); // 9
	_paaWrite(p, 0x58, 0x4C); // 10
	_paaWrite(p, 0x79, 0x00); // 11
	_paaWrite(p, 0x7F, 0x0E); // 12
	_paaWrite(p, 0x54, 0x04); // 13
	_paaWrite(p, 0x7F, 0x0E); // 14
	_paaWrite(p, 0x55, 0x0D); // 15
	_paaWrite(p, 0x58, 0xD5); // 16
	_paaWrite(p, 0x56, 0xFB); // 17
	_paaWrite(p, 0x57, 0xEB); // 18
	_paaWrite(p, 0x7F, 0x15); // 19
	if((_paaRead(p, 0x58) & 0x80) == 0) { // 20
		_paaWrite(p, 0x58, 0x04);
		_paaWrite(p, 0x57, 0x80);
	} else { // (if = 1)
		_paaWrite(p, 0x58, 0x84);
		_paaWrite(p, 0x57, 0x00);
	}
	_paaWrite(p, 0x7F, 0x07); // 21
	_paaWrite(p, 0x40, 0x43); // 22
	_paaWrite(p, 0x7F, 0x13); // 23
	_paaWrite(p, 0x49, 0x20); // 24
	_paaWrite(p, 0x7F, 0x14); // 25
	_paaWrite(p, 0x54, 0x02); // 26
	_paaWrite(p, 0x7F, 0x15); // 27
	_paaWrite(p, 0x60, 0x00); // 28
	_paaWrite(p, 0x7F, 0x06); // 29
	_paaWrite(p, 0x74, 0x50); // 30
	_paaWrite(p, 0x7B, 0x02); // 31
	_paaWrite(p, 0x7F, 0x00); // 32

	_paaWrite(p, 0x64, 0x74); // 33
	_paaWrite(p, 0x65, 0x03); // 34
	_paaWrite(p, 0x72, 0x0E); // 35
	_paaWrite(p, 0x73, 0x00); // 36
	_paaWrite(p, 0x7F, 0x14); // 37
	_paaWrite(p, 0x61, 0x3E); // 38
	_paaWrite(p, 0x62, 0x1E); // 39
	_paaWrite(p, 0x63, 0x1E); // 40
	_paaWrite(p, 0x7F, 0x15); // 41
	_paaWrite(p, 0x69, 0x1E); // 42
	_paaWrite(p, 0x7F, 0x07); // 43
	_paaWrite(p, 0x40, 0x40); // 44
	_paaWrite(p, 0x7F, 0x00); // 45
	_paaWrite(p, 0x61, 0x00); // 46
	_paaWrite(p, 0x7F, 0x15); // 47
	_paaWrite(p, 0x63, 0x00); // 48
	_paaWrite(p, 0x62, 0x00); // 49
	_paaWrite(p, 0x7F, 0x00); // 50
	_paaWrite(p, 0x61, 0xAD); // 51
	_paaWrite(p, 0x7F, 0x15); // 52
	_paaWrite(p, 0x5D, 0x2C); // 53
	_paaWrite(p, 0x5E, 0xC4); // 54
	HAL_Delay(100); // 55
	_paaWrite(p, 0x5D, 0x04); // 56
	_paaWrite(p, 0x5E, 0xEC); // 57
	_paaWrite(p, 0x7F, 0x05); // 58
	_paaWrite(p, 0x42, 0x48); // 59
	_paaWrite(p, 0x43, 0xE7); // 60
	_paaWrite(p, 0x7F, 0x06); // 61
	_paaWrite(p, 0x71, 0x03); // 62
	_paaWrite(p, 0x7F, 0x09); // 63
	_paaWrite(p, 0x60, 0x1C); // 64
	_paaWrite(p, 0x61, 0x1E); // 65
	_paaWrite(p, 0x62, 0x02); // 66
	_paaWrite(p, 0x63, 0x04); // 67
	_paaWrite(p, 0x64, 0x1E); // 68
	_paaWrite(p, 0x65, 0x1F); // 69
	_paaWrite(p, 0x66, 0x01); // 70
	_paaWrite(p, 0x67, 0x02); // 71

	_paaWrite(p, 0x68, 0x02); // 72
	_paaWrite(p, 0x69, 0x01); // 73
	_paaWrite(p, 0x6A, 0x1F); // 74
	_paaWrite(p, 0x6B, 0x1E); // 75
	_paaWrite(p, 0x6C, 0x04); // 76
	_paaWrite(p, 0x6D, 0x02); // 77
	_paaWrite(p, 0x6E, 0x1E); // 78
	_paaWrite(p, 0x6F, 0x1C); // 79
	_paaWrite(p, 0x7F, 0x05); // 80
	_paaWrite(p, 0x45, 0x94); // 81
	_paaWrite(p, 0x45, 0x14); // 82
	_paaWrite(p, 0x44, 0x45); // 83
	_paaWrite(p, 0x45, 0x17); // 84
	_paaWrite(p, 0x7F, 0x09); // 85
	_paaWrite(p, 0x47, 0x4F); // 86
	_paaWrite(p, 0x4F, 0x00); // 87
	_paaWrite(p, 0x52, 0x04); // 88
	_paaWrite(p, 0x7F, 0x0C); // 89
	_paaWrite(p, 0x4E, 0x00); // 90
	_paaWrite(p, 0x5B, 0x00); // 91
	_paaWrite(p, 0x7F, 0x0D); // 92
	_paaWrite(p, 0x71, 0x92); // 93
	_paaWrite(p, 0x70, 0x07); // 94
	_paaWrite(p, 0x73, 0x92); // 95
	_paaWrite(p, 0x72, 0x07); // 96
	_paaWrite(p, 0x7F, 0x00); // 97
	_paaWrite(p, 0x5B, 0x20); // 98
	_paaWrite(p, 0x48, 0x13); // 99
	_paaWrite(p, 0x49, 0x00); // 100
	_paaWrite(p, 0x4A, 0x13); // 101
	_paaWrite(p, 0x4B, 0x00); // 102
	_paaWrite(p, 0x47, 0x01); // 103
	_paaWrite(p, 0x54, 0x55); // 104
	_paaWrite(p, 0x5A, 0x50); // 105
	_paaWrite(p, 0x66, 0x03); // 106
	_paaWrite(p, 0x67, 0x00); // 107
	_paaWrite(p, 0x7F, 0x07); // 108
	_paaWrite(p, 0x40, 0x43); // 109

	_paaWrite(p, 0x7F, 0x05); // 110
	_paaWrite(p, 0x4D, 0x00); // 111
	_paaWrite(p, 0x6D, 0x96); // 112
	_paaWrite(p, 0x55, 0x62); // 113
	_paaWrite(p, 0x59, 0x21); // 114
	_paaWrite(p, 0x5F, 0xD8); // 115
	_paaWrite(p, 0x6A, 0x22); // 116
	_paaWrite(p, 0x7F, 0x07); // 117
	_paaWrite(p, 0x42, 0x30); // 118
	_paaWrite(p, 0x43, 0x00); // 119
	_paaWrite(p, 0x7F, 0x06); // 120
	_paaWrite(p, 0x4C, 0x01); // 121
	_paaWrite(p, 0x54, 0x02); // 122
	_paaWrite(p, 0x62, 0x01); // 123
	_paaWrite(p, 0x7F, 0x09); // 124
	_paaWrite(p, 0x41, 0x01); // 125
	_paaWrite(p, 0x4F, 0x00); // 126
	_paaWrite(p, 0x7F, 0x0A); // 127
	_paaWrite(p, 0x4C, 0x18); // 128
	_paaWrite(p, 0x51, 0x8F); // 129
	_paaWrite(p, 0x7F, 0x07); // 130
	_paaWrite(p, 0x40, 0x40); // 131
	_paaWrite(p, 0x7F, 0x00); // 132
	_paaWrite(p, 0x40, 0x80); // 133
	_paaWrite(p, 0x7F, 0x05); // 134
	_paaWrite(p, 0x4D, 0x01); // 135
	_paaWrite(p, 0x7F, 0x06); // 136
	_paaWrite(p, 0x54, 0x01); // 137
	_paaWrite(p, 0x62, 0x01); // 138
	_paaWrite(p, 0x7F, 0x09); // 139
	_paaWrite(p, 0x40, 0x03); // 140
	_paaWrite(p, 0x44, 0x08); // 141
	_paaWrite(p, 0x4F, 0x08); // 142
	_paaWrite(p, 0x7F, 0x0A); // 143
	_paaWrite(p, 0x51, 0x8E); // 144
	_paaWrite(p, 0x7F, 0x00); // 145
	_paaWrite(p, 0x66, 0x11); // 146
	_paaWrite(p, 0x67, 0x08); // 147


}

// ================= High level ================= //

paa_err_t paaInit(paa5163_t* p){
	// 1. Power on, wait 50ms
	HAL_Delay(50);

	// 2. Drive NCS high to reset SPI
	_deselect(p);

	DWT_Init();

	// 3. Assert NRST low (>20us), then de-assert to reset
	HAL_GPIO_WritePin(p->NRST_Port, p->NRST_Pin, GPIO_PIN_RESET);
	delay_us(25); // >20us
	HAL_GPIO_WritePin(p->NRST_Port, p->NRST_Pin, GPIO_PIN_SET);

	// 4. Wait 2ms
	HAL_Delay(2);

	// 5. Clear observation register
	_paaWrite(p, observation, 0x00);

	// 6. Read observation register (expect 0xB7 or 0xBF)
	uint8_t i = 0; uint8_t observ_read;
	do {
		i++;
		observ_read = _paaRead(p, observation);
		if(i >= OBSERVATION_INIT_RETRY) return paa_observ;
	} while(observ_read != 0xB7 && observ_read != 0xBF);

	// 7. Read registers 0x02, 0x03, 0x04, 0x05, 0x06 to clear motion bit and buffers
	_paaRead(p, 0x02); // motion
	_paaRead(p, 0x03); // delta_x_l
	_paaRead(p, 0x04); // delta_x_h
	_paaRead(p, 0x05); // delta_y_l
	_paaRead(p, 0x06); // delta_y_h

	// Communication check (after clearing obs)
	if(_paaRead(p, product_id) != ((~_paaRead(p, inverse_product_id)) & 0xFF)){
		return paa_coms;
	}

	// Performance optimization (optional, after init)
	_paaPerfOpti(p);

	// Init done! Now we can set values
	if(p->resolution == 0) p->resolution = DEFAULT_RESOLUTION;
	_paaSetResolution(p, p->resolution);
	_paaSetOrientation(p, p->axis_swap, p->invert_x, p->invert_y);
	p->initialized = 1;
	return paa_ok;
}

// TODO : use the INT pin instead of reading the motion register (to be faster), if the motion register clears itself when reading x or y
void paaReadMotion(paa5163_t* p){
	if(!p->initialized) return;

	uint8_t* dx8 = (uint8_t*) &(p->dx_cpi);
	uint8_t* dy8 = (uint8_t*) &(p->dy_cpi);

	if(_paaMotion(p)){
		uint8_t dx_l = _paaRead(p, delta_x_l);
		uint8_t dx_h = _paaRead(p, delta_x_h);
		uint8_t dy_l = _paaRead(p, delta_y_l);
		uint8_t dy_h = _paaRead(p, delta_y_h);

		dx8[0] = dx_l;
		dx8[1] = dx_h;
		dy8[0] = dy_l;
		dy8[1] = dy_h;

		p->x_cpi += p->dx_cpi;
		p->y_cpi += p->dy_cpi;

		p->x = ((float) p->x_cpi * (float) IN_TO_MM) / (float) p->resolution;
		p->y = ((float) p->y_cpi * (float) IN_TO_MM) / (float) p->resolution;

		// Debug: print raw register values
		#ifdef DEBUG_UART
		printf("RAW dx_l=0x%02X dx_h=0x%02X dy_l=0x%02X dy_h=0x%02X\n", dx_l, dx_h, dy_l, dy_h);
		#endif
	}
}

float paaGetX(paa5163_t* p){
	return p->x;
}

float paaGetY(paa5163_t* p){
	return p->y;
}
