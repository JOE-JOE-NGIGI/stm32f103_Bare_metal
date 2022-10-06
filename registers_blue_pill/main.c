#include "stm32f10x.h"                  // Device header
//macro for writing 0 and 1 to the bit

#define BitSet(arg,posn) ((arg) | (1L << (posn)))
#define BitClr(arg,posn), ((arg) & ~(1L << (posn)))

int main () 
	{
		RCC->APB2ENR |= (1<<4); //enable bit 4 of the APB2ENR register(Enable Clock)
	
		//Configure Port C pin 13 as Push-Pull output
		GPIOC->CRH &= ~((1<<23) | (1<<22) | (1<<20));
		GPIOC->CRH |= (1<<21);
		
		while(1) {
			GPIOC->ODR ^= (1<<13); //Toggle bit 13
			
			
		}
	
	
	}
	
	
