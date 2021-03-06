#include "LED.h"

void LED_Init() {
	LED1_Init();
	LED2_Init();
	LED3_Init();
	LED4_Init();
}

void LED1_Init() {
	GPIO_DIR_Write(LED1_PORT,LED1_MASK,OUTPUT);
}

void LED2_Init() {
	GPIO_DIR_Write(LED2_PORT,LED2_MASK,OUTPUT);
}

void LED3_Init() {
	GPIO_DIR_Write(LED3_PORT,LED3_MASK,OUTPUT);
}

void LED4_Init() {
	GPIO_DIR_Write(LED4_PORT,LED4_MASK,OUTPUT);
}

void LED_ON() {
	LED1_On();
	LED2_On();
	LED3_On();
	LED4_On();
}

void LED1_On() {
	GPIO_PIN_Write(LED1_PORT,LED1_MASK,HIGH);
}

void LED2_On() {
	GPIO_PIN_Write(LED2_PORT,LED2_MASK,HIGH);
}

void LED3_On() {
	GPIO_PIN_Write(LED3_PORT,LED3_MASK,HIGH);
}

void LED4_On() {
	GPIO_PIN_Write(LED4_PORT,LED4_MASK,HIGH);
}

void LED_OFF() {
	LED1_Off();
	LED2_Off();
	LED3_Off();
	LED4_Off();
}

void LED1_Off() {
	GPIO_PIN_Write(LED1_PORT,LED1_MASK,LOW);
}

void LED2_Off() {
	GPIO_PIN_Write(LED2_PORT,LED2_MASK,LOW);
}

void LED3_Off() {
	GPIO_PIN_Write(LED3_PORT,LED3_MASK,LOW);
}

void LED4_Off() {
	GPIO_PIN_Write(LED4_PORT,LED4_MASK,LOW);
}



//EMBEDDED LEDS
void EMBEDLED_Init() {
	GPIO_DIR_Write(EMBEDLED1_PORT,EMBEDLED1_MASK,OUTPUT);
	GPIO_DIR_Write(EMBEDLED2_PORT,EMBEDLED2_MASK,OUTPUT);
	GPIO_DIR_Write(EMBEDLED3_PORT,EMBEDLED3_MASK,OUTPUT);
	GPIO_DIR_Write(EMBEDLED4_PORT,EMBEDLED4_MASK,OUTPUT);
}



void EMBEDLED1_On() {
	GPIO_PIN_Write(EMBEDLED1_PORT,EMBEDLED1_MASK,LOW);
}

void EMBEDLED2_On() {
	GPIO_PIN_Write(EMBEDLED2_PORT,EMBEDLED2_MASK,LOW);
}

void EMBEDLED3_On() {
	GPIO_PIN_Write(EMBEDLED3_PORT,EMBEDLED3_MASK,HIGH);
}

void EMBEDLED4_On() {
	GPIO_PIN_Write(EMBEDLED4_PORT,EMBEDLED4_MASK,HIGH);
}
void EMBEDLED1_Off() {
	GPIO_PIN_Write(EMBEDLED1_PORT,EMBEDLED1_MASK,HIGH);
}

void EMBEDLED2_Off() {
	GPIO_PIN_Write(EMBEDLED2_PORT,EMBEDLED2_MASK,HIGH);
}

void EMBEDLED3_Off() {
	GPIO_PIN_Write(EMBEDLED3_PORT,EMBEDLED3_MASK,LOW);
}

void EMBEDLED4_Off() {
	GPIO_PIN_Write(EMBEDLED4_PORT,EMBEDLED4_MASK,LOW);
}

void EMBEDLED_ALL_On() {
	EMBEDLED1_On();
	EMBEDLED2_On();
	EMBEDLED3_On();
	EMBEDLED4_On();
}
void EMBEDLED_ALL_Off() {
	EMBEDLED1_Off();
	EMBEDLED2_Off();
	EMBEDLED3_Off();
	EMBEDLED4_Off();
}


