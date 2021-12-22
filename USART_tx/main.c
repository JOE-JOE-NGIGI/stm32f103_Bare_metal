#include "stm32f10x.h"                  // Device header
void delay_dummy(void);
void uart_init(void);
void usart_tx(int ch);




//initialize USART
void uart_init() {
	
	
	//configure related GPIOs clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	GPIOA->CRH |= (0x0BUL << 4); //Tx pin = PA9
	
	
	//enable USART clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	
	//initialize baudrate
	USART1->BRR = 64000000L/9600L; //divide clock rate by default baud rate
	
	//enable tx
	USART1->CR1 |= (1<<3); //
	
	//enable rx
	USART1->CR1 |= (1<<2);
	
	//enable uart bit
	USART1->CR1 |= (1<<13);
	
	
}

void usart_tx(int ch){
	while(!( USART1->SR & (1<<7))); //TXE
	USART1->DR = (ch & 0xFF);
}

void delay_dummy(){
	unsigned int i=0;
	for(i=0;i<0xFFF;i++);
}
int main()
{
	uart_init();
	delay_dummy();
	
	while(1){
	usart_tx('H');
	usart_tx('e');
	usart_tx('l');
	usart_tx('l');
	usart_tx('o');
	}

}
