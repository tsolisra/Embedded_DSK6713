/*
 * main.c
 */
char getDIP();

int main(void) {

	volatile char *pointer;
	pointer = (volatile char*) 0x90080000;
	//*pointer = 0xF1;

	char data;
	char holder;
	for(;;){
		data = getDIP();
		holder = *pointer & 0xF0;
		*pointer = holder | data;

	}


	return 0;
}


char getDIP(){
	volatile char *pointer;
	pointer = (volatile char*) 0x90080000;
	char data = *pointer;
	data = data >> 4;
	return data;
}
