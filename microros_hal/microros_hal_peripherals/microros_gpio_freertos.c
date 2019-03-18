
/********************************************************************
 * The arguments, are the number of the pin and mode to use:
 * - Digital Input
 * - Digital Output
 * - Analog (Only freertos at this moment)
 */

#include "microros_gpio_hal.c"

#ifdef freertos

void freertos_gpio_config(uint8_t pin, uint16_t mode){
	//Previous the use of this function, you need to bring up the port that you're looking for to use

	//First we check if the pins are in the available range
	if(pin<0 || pin >19){
		//Add some kind of message system to advice that the pin is already in use for SPI/I2C or UART
		return -1;
	}
	//TODO:Add peripheral configuration pins
	/*if(SPI && (pin == 13 || pin == 12 || pin == 11) ){
		//Error: Triying to use SPI pins when they are configured
	}
	else if(I2C && (pin == 0 || pin == 1 ) ){

	}*/
	//Check if the pin that you want to use and I2C or SPI pin and this it's in use
	//TODO

	//Struct to init the config of the pins.
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Mode = mode;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	switch(pin) {
	//Only valid for Olimex Board
		case 0 :
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		break;

		case 1 :
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		break;

		case 2 :
		GPIO_InitStruct.Pin = GPIO_PIN_2;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
		break;

		case 3 :
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
		break;

		case 4 :
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
		break;

		case 5 :
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
		break;

		case 6 :
		GPIO_InitStruct.Pin = GPIO_PIN_7;
		HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
		break;

		case 7 :
		GPIO_InitStruct.Pin = GPIO_PIN_8;
		HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
		break;

		case 8 :
		GPIO_InitStruct.Pin = GPIO_PIN_12;
		HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
		break;

		case 9 :
		GPIO_InitStruct.Pin = GPIO_PIN_15;
		HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
		break;

		case 10 :
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		break;

		case 11 :
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		break;

		case 12 :
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		break;

		case 13 :
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		break;
		//Add analog pins
	}
}

/****************************************************
 * This function is to write a digital value to arduino header pins
 * Arguments:
 * - Pin: 0 to 13
 * - Value: HIGH or LOW
 */

void freertos_digital_write(uint8_t pin, uint8_t value){

	switch(pin){
		case 0 :
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, value);
		break;
		case 1 :
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, value);
		break;
		case 2 :
			 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, value);
		break;
		case 3 :
			 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, value);
		break;
		case 4 :
			 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, value);
		break;
		case 5 :
			 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, value);
		break;
		case 6 :
			 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, value);
		break;
		case 7 :
			 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, value);
		break;
		case 8 :
			 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, value);
		break;
		case 9 :
			 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, value);
		break;
		case 10 :
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, value);
		break;
		case 11 :
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, value);
		break;
		case 12 :
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, value);
		break;
		case 13 :
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, value);
		break;
	}
}

//TODO : GPIO READ
//TODO : GPIO INterruption
//TODO : GPIO Analog

# endif
