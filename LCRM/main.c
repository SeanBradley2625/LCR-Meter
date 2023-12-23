#include "baseFunctions.h"

uint32_t capDischargeResistor=10000;

void findCapacitorValue(){
	SetCapacitorValue(findTimeConstant()*1000/capDischargeResistor);	
}

int main (void) {
	INIT();
	SetLCDMode(0);
	findCapacitorValue();
	while(1);
}
