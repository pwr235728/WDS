
#include "nrf24_hal.h"

extern SPI_HandleTypeDef hspi2;

// Configure the GPIO lines of the nRF24L01 transceiver
// note: IRQ pin must be configured separately
void nRF24_GPIO_Init(void) {

	nRF24_CSN_H();
	// Configure CE pin
	nRF24_CE_L();
}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
uint8_t nRF24_LL_RW(uint8_t data) {
	 // Wait until TX buffer is empty

	uint8_t rx;

	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) { }

	HAL_SPI_TransmitReceive(&hspi2, &data, &rx, 1, 1000);


	// Return received byte
	return rx;
}
