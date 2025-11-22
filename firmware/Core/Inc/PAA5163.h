#ifndef INC_PAA5163_H_
#define INC_PAA5163_H_

#include "main.h"

#include <stdbool.h>

#define DEFAULT_RESOLUTION 20000

typedef enum {
	paa_ok = 0,

	paa_err = 1, // unknown error

	// Init errors :
	paa_coms = 10, // could not communicate with the chip
	paa_observ = 11, // could not read the observation register properly (according to values given by the datasheet)
} paa_err_t;

typedef struct {
	SPI_HandleTypeDef* spi;

	GPIO_TypeDef *NCS_Port;
	uint16_t NCS_Pin;
	GPIO_TypeDef *NRST_Port;
	uint16_t NRST_Pin;

	uint16_t resolution; // cpi, between 100 and 20 000, by increments of 100. Value goes to DEFAULT_RESOLUTION if left to 0

	/* Default orientation :
	 * 		x-
	 *		|
	 * y- --+-- y+
	 * 		|
	 * 		x+
	 */
	bool axis_swap;
	bool invert_x;
	bool invert_y;

	int16_t dx_cpi; // last readings
	int16_t dy_cpi;

	int32_t x_cpi; // sum of all readings
	int32_t y_cpi;

	float dx; // counts per mm (sum of all readings)
	float dy;

	bool initialized;
} paa5163_t;

paa_err_t paaInit(paa5163_t* p);

void paaReadMotion(paa5163_t* p);

float paaGetX(paa5163_t* p);
float paaGetY(paa5163_t* p);

#endif /* INC_PAA5163_H_ */
