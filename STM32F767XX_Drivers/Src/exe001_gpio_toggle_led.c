/**
 * File: exe001_gpio_toggle_led.c
 *
 * Last reviewed and updated: 2025/06/26
 * Author: VanTungDinh
 */

#include <stdio.h>
#include <stdint.h>
#include <stm32f767xx.h>


void TimeDelay(uint8_t Div_x) {
	for (uint32_t i = 0; i <= 500000 / Div_x; i += 1) {}
}


int main(void) {
	GPIO_Handle_t Button, BlueLed, CheckPin;

	Button.pGPIOx = GPIOC;
	Button.GPIO_Config.GPIO_PinMode = GPIO_MODE_INPUT;
	Button.GPIO_Config.GPIO_PinNumber = GPIO_PIN_13;
	Button.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	BlueLed.pGPIOx = GPIOB;
	BlueLed.GPIO_Config.GPIO_PinMode = GPIO_MODE_OUTPUT;
	BlueLed.GPIO_Config.GPIO_PinNumber = GPIO_PIN_7;
	BlueLed.GPIO_Config.GPIO_PinOutSpeed = GPIO_OUT_SPEED_VERYHIGH;
	BlueLed.GPIO_Config.GPIO_PinOutType = GPIO_OUT_TYPE_PUSHPULL;
	BlueLed.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	CheckPin.pGPIOx = GPIOA;
	CheckPin.GPIO_Config.GPIO_PinMode = GPIO_MODE_OUTPUT;
	CheckPin.GPIO_Config.GPIO_PinNumber = GPIO_PIN_5;
	CheckPin.GPIO_Config.GPIO_PinOutSpeed = GPIO_OUT_SPEED_VERYHIGH;
	CheckPin.GPIO_Config.GPIO_PinOutType = GPIO_OUT_TYPE_PUSHPULL;
	CheckPin.GPIO_Config.GPIO_PinPuPdControl = GPIO_NOPULL;

	GPIO_Init(&Button);
	GPIO_Init(&BlueLed);
	GPIO_Init(&CheckPin);

	while (1) {
		if (GPIO_ReadInputPin(GPIOC, GPIO_PIN_13) == HIGH) {
			TimeDelay(4);
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_7);
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		}
	}

	return 0;
}
