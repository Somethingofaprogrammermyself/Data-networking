#include "main.h"

#include "Time_Delays.h"
#include "Clk_Config.h"
#include "LCD_Display.h"

#include <stdio.h>
#include <string.h>

#include "Small_7.h"
#include "Arial_9.h"
#include "Arial_12.h"
#include "Arial_24.h"

/*
This program allows the user to read the temperature from the mbed application shield temperature sensor,
write it to the EEPROM, and read it back again.

The following joystick presses perform the listed functions, after performing a function it flashes a success
message on the LCD.
Centre: Read temperature from sensor
Right: Write temperature to EEPROM
Left: Read temperature from EEPROM
*/

//Temperature Sensor I2C Address
#define TEMPADR 0x90

//EEPROM I2C Address
#define EEPROMADR 0xA0

//GPIO
void configure_gpio(void);

//Joystick Configuration
void joystick_configure(void);
uint32_t joystick_up(void); //Not used.
uint32_t joystick_down(void); //Not used.
uint32_t joystick_left(void);
uint32_t joystick_right(void);
uint32_t joystick_centre(void);

	uint16_t pack[30]; // creating the array for packet
  int macdest = 43690  ; //initialise values to use in array
	int macsrc =  48059 ;
	int packet = 1;//packet is initialised to start at case 1
	int flck = 1 ; 
	

//I2C
void i2c_1_configure(void);

//Temperature
uint16_t read_temperature(void);

//EEPROM
void eeprom_write(uint16_t pack[]);
uint16_t eeprom_read(void);

uint32_t crccalc(uint16_t pack[]);
		
int main(void){
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

	
	for (uint16_t x = 0; x < 3; x++){ //add mac dest to packet
		pack[x] = macdest;
	}
	
	for (uint16_t x = 3; x < 6; x++){ //add macsrc to packet
		pack[x] = macsrc;
	}
	
	pack[6] = 46; //add length to packet
	
	for (uint16_t x = 7; x < 30; x++){ //initialise payload in packet
		pack[x] = 0;
	}
	
	
	
	
	//Init
        SystemClock_Config();/* Configure the system clock to 84.0 MHz */
	SysTick_Config_MCE2(us);	

	//Configure LCD
	Configure_LCD_Pins();
	Configure_SPI1();
	Activate_SPI1();
	Clear_Screen();
	Initialise_LCD_Controller();
	set_font((unsigned char*) Arial_12);
		
	//Configure GPIO
	configure_gpio();
	joystick_configure();
	
	i2c_1_configure(); //Configure I2C and set up the GPIO pins it uses
	
	uint16_t sample = 0; //int to store temperature in
	
	char outputString[18]; //Buffer to store text in for LCD
	
	
	put_string(0,0," Mac Dest            ");    //first thing printed is mac dest
	sprintf(outputString,"%x%x%x",pack[0], pack[1], pack[2]);
	put_string(0,15,outputString);	
	//Main Loop
  while (1){
	  
	if (packet != flck) {
			 
			switch (packet) {  //switch case for every joystick movement
				case 1 : 
					Clear_Screen();
						put_string(0,0," Mac Dest            "); 
						sprintf(outputString,"%x%x%x",pack[0], pack[1], pack[2]);
						put_string(0,15,outputString);
						
						break;
				case 2 : 
				 Clear_Screen();
						put_string(0,0," Mac SRC            "); 
						sprintf(outputString,"%x%x%x",pack[3], pack[4], pack[5]);
						put_string(0,15,outputString);
						
						break;
				case 3 : 
					Clear_Screen();
						put_string(0,0," Length            "); 
						sprintf(outputString,"%x",pack[6]);
						put_string(0,15,outputString);
						
						break;
				case 4 :
					
					Clear_Screen();
						put_string(0,0," Payload            "); 
						sprintf(outputString,"%5f",pack[7]*0.125);
						put_string(0,15,outputString);
					
						break;
				
				
			} 
			flck = packet;
		}
		
			
				
		if(joystick_centre()){			
			pack[7] = read_temperature(); //Reads temperature sensor
			
			put_string(0,0,"             "); //Report success temperature read
			put_string(0,0,"Sampled");
			put_string(0,15,"             ");
			LL_mDelay(50000);
			
			sprintf(outputString, "Temp: %f", pack[7]*0.125); //Print to LCD
			put_string(0,0,outputString);
			LL_mDelay(50000);
		} else if(joystick_right()){
			eeprom_write(pack); //Write to EEPROM
			
			put_string(0,0,"             "); //Report successful write
			put_string(0,0,"Written");
			put_string(0,15,"             ");
			LL_mDelay(50000);
			
			sprintf(outputString, "Temp: %f", sample*0.125); //Print to LCD
			put_string(0,0,outputString);
		} else if(joystick_left()){
			sample = eeprom_read(); //Read from EEPROM
			
			put_string(0,0,"             "); //Report success read
			put_string(0,0,"Retrieved");
			put_string(0,15,"             ");
			LL_mDelay(50000);
			
			sprintf(outputString, "Temp: %f", sample*0.125); //Print to LCD
			put_string(0,0,outputString);
		} else if(joystick_up()) { //setting limits for joystick up and down movement for packet.
			if (packet>1) { 
				packet--;}
			LL_mDelay(50000);
		}
		else if(joystick_down()) {
			if (packet<4){ 
				packet++;}
			LL_mDelay(50000);
		}

		
			
			
			
			
			
							
		
  
}
}



void configure_gpio(void){
	//Configures the GPIO pins by enabling the peripherial clocks on the ports uses by the shield
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); 
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC); 
}	

void i2c_1_configure(void){
	//Enable I2C1 Clock
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  
	// Configure SCL as: Alternate function, High Speed, Open Drain, Pull Up
        LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
        LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);
	
        // Configure SDA as: Alternate, High Speed, Open Drain, Pull Up
        LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
        LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
  
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  
        LL_I2C_Disable(I2C1);
        LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);
        LL_I2C_ConfigSpeed(I2C1, 84000000, 400000, LL_I2C_DUTYCYCLE_2);
        LL_I2C_Enable(I2C1);
}	

void joystick_configure(void){
	//This function configures all the GPIO pins that are connected to the joystick on the mbed shield
	//(not all joystick pins are used)
	
	LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT); 		//set PA4 as Input
	LL_GPIO_SetPinPull (GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO); 		//set PA4 as NO pull
	
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); 		//set PB0 as Input
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); 		//set PB0 as NO pull
	
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT); 		//set PC1 as Input
	LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_NO); 		//set PC1 as NO pull
	
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); 		//set PC0 as Input
	LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); 		//set PC0 as NO pull
	
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT); 		//set PB5 as Input
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO); 		//set PB5 as NO pull
}

uint32_t joystick_up(void) {
	//Returns 1 if the joystick is pressed up, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4));
}

uint32_t joystick_down(void) {
	//Returns 1 if the joystick is pressed down, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_0));
}

uint32_t joystick_left(void) {
	//Returns 1 if the joystick is pressed left, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_1));
}

uint32_t joystick_right(void) {
	//Returns 1 if the joystick is pressed right, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_0));
}
uint32_t joystick_centre(void) {
	//Returns 1 if the joystick is pressed in the centre, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_5));
}

void eeprom_write(uint16_t pack[]){
	//Writes two bytes to the EEPROM
	int y = 0;
	
	LL_I2C_GenerateStartCondition(I2C1); //START1
        while(!LL_I2C_IsActiveFlag_SB(I2C1));

        LL_I2C_TransmitData8(I2C1, EEPROMADR); //CONTROL BYTE (ADDRESS + WRITE)
        while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);

        LL_I2C_TransmitData8(I2C1, 0x00); //ADDRESS HIGH BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

        LL_I2C_TransmitData8(I2C1, 0x00); //ADDRESS LOW BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));
				
	
	for (y=0; y<16; y++) {
		
        LL_I2C_TransmitData8(I2C1, (char)(pack[y] >> 8)); //DATA HIGH BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, (char)(pack[y] & 0x00FF)); //DATA LOW BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	}
        LL_I2C_GenerateStopCondition(I2C1); //STOP1
				
				
				LL_mDelay(50000);
				 	LL_I2C_GenerateStartCondition(I2C1); //START2
        while(!LL_I2C_IsActiveFlag_SB(I2C1));


        LL_I2C_TransmitData8(I2C1, EEPROMADR); //CONTROL BYTE (ADDRESS + WRITE)
				while(LL_I2C_IsActiveFlag_AF (I2C1)){
				LL_I2C_ClearFlag_AF (I2C1);
					
				}

        LL_I2C_TransmitData8(I2C1, 0x20); //ADDRESS HIGH BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

        LL_I2C_TransmitData8(I2C1, 0x20); //ADDRESS LOW BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	
				for (y=16; y<30; y++) {
					
        LL_I2C_TransmitData8(I2C1, (char)(pack[y] >> 8)); //DATA HIGH BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, (char)(pack[y] & 0x00FF)); //DATA LOW BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));
				}
        LL_I2C_GenerateStopCondition(I2C1); //STOP2
}

uint16_t read_temperature(void){
	//Reads the 11 bit temperature value from the 2 byte temperature register
	
	uint16_t temperature = 0;
  
	LL_I2C_GenerateStartCondition(I2C1); //START
        while(!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, TEMPADR); //CONTROL BYTE (ADDRESS + READ)
        while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);

        LL_I2C_TransmitData8(I2C1, 0x00); //Set pointer register to temperature register
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_GenerateStartCondition(I2C1); //RE-START
        while(!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, TEMPADR+1); //ADDRESS + READ
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);

        LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK); //ACK INCOMING DATA
        while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
        temperature = LL_I2C_ReceiveData8(I2C1);  //TEMPERATURE HIGH BYTE
	temperature = temperature << 8;

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK); //NACK INCOMING DATA
	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
        temperature += LL_I2C_ReceiveData8(I2C1);  //TEMPERATURE LOW BYTE

        LL_I2C_GenerateStopCondition(I2C1);       //STOP

	return temperature >> 5; //Bit shift temperature right, since it's stored in the upper part of the 16 bits, originally.
}

uint16_t eeprom_read(void){
	//Reads two bytes from the EEPROM
	uint16_t data[30];
	int j=0;
	
	LL_I2C_GenerateStartCondition(I2C1); //START
        while(!LL_I2C_IsActiveFlag_SB(I2C1));

        LL_I2C_TransmitData8(I2C1, EEPROMADR); //CONTROL BYTE (ADDRESS + WRITE)
        while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);

        LL_I2C_TransmitData8(I2C1, 0x00); //ADDRESS HIGH BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, 0x00); //ADDRESS LOW BYTE
        while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_GenerateStartCondition(I2C1); //RE-START
        while(!LL_I2C_IsActiveFlag_SB(I2C1));

        LL_I2C_TransmitData8(I2C1, EEPROMADR+1); //ADDRESS + READ
        while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
        LL_I2C_ClearFlag_ADDR(I2C1);
				
				for (j=0; j<30; j++) {
				
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK); //ACK INCOMING DATA

        while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
        data[j] = LL_I2C_ReceiveData8(I2C1); //DATA HIGH BYTE
	data[j] = data[j] << 8;
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK); //ACK INCOMING DATA

	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
        data[j] += LL_I2C_ReceiveData8(I2C1);  //DATA LOW BYTE
				}
	LL_I2C_GenerateStopCondition(I2C1); //STOP
	return data[7];
}
