/*
EEET2481 - FINAL ASSIGNMENT
Group: 8
Hoang Thai Kiet - s3855250, Nguyen Huynh Hai Long - 3877624 
Exercise 2: BATTLESHIP GAME
Reference1: https://danchouzhou.blogspot.com/2018/01/nuvoton-nuc200-nuc220fifouart.html 
Reference2: https://bbs.21ic.com/icview-2952910-1-1.html
Reference3: http://forum.nuvoton.com/viewtopic.php?t=8422 
*/

//------------------------------------------- main.c CODE STARTS -------------------------------------------------------------------------------------
#include <stdio.h>
#include "NUC100Series.h"

// Macro define
#define HXT_STATUS 1<<0
#define PLL_STATUS 1<<2
#define PLLCON_FB_DV_VAL 10
#define CPUCLOCKDIVIDE 1
#define TMR0_COUNTS 1000
#define BUZZER_BEEP_TIME 10
#define BUZZER_BEEP_DELAY 2000000
#define BOUNCING_DELAY 180000					// for button

// Declare funtion
void enable_TMR0(void);
void System_Config(void);
void KeyPadEnable(void);
uint8_t KeyPadScanning(void);
void Game_Algorithm(void);			// Specify the shoot and coordinate
void Game_Display(void);				// content of game displayed
void Buzzer_Beep(int Beep_time);

// Declare function and var UART0
void UART0_Config(void);
void UART0_sendChar(int ch);
//char UART0_getChar();
void UART02_IRQHandler(void);

// Declare function for SPI3 and LCD
void SPI3_Config(void);
void LCD_start(void);
void LCD_command(unsigned char temp);
void LCD_data(unsigned char temp);
void LCD_clear(void);
void LCD_SetAddress(uint8_t PageAddr, uint8_t ColumnAddr);

// Map coordinates and key matrix
volatile int cord[2]={0,0};				// Map coordinate X and Y
volatile int shot = 0;						// ship shot
volatile int current_Cord = 0;		// Cuurent cordinate X and Y
uint8_t KeyPressed = 0;						// identify the pressed Key Matrix
volatile Seg7_XY = 0;							// Select which X or Y cordinate for segment U11

// Define map row and col
volatile int row = 0;
volatile int col = 0;
// Define score and its key
volatile int score = 0;
volatile char score_txt[] = "00";
volatile int score_key = 0;		// indicate led5 flash 3 times when hit
// Interrupt variables
volatile int ReadMap = 0;			// UART interrupt key
volatile int Buzzer_on = 0;		// to activate Buzzer_Beep() 
volatile int TimePassed = 0;			// for tmr interrupt
// data received from map
volatile char ReceivedByte;

// FSM: loading screen -> Welcome screen -> loading content -> play -> over 
enum game_state {loading, welcome, game_loading, gameplay, gameover};
enum game_state state;

// map 8x8
int map[8][8]={
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0}
};

//Gloabl Array to display on 7segment for NUC140 MCU
int pattern[] = {
               //   gedbaf_dot_c
                  0b10000010,  //Number 0          // ---a----
                  0b11101110,  //Number 1          // |      |
                  0b00000111,  //Number 2          // f      b
                  0b01000110,  //Number 3          // |      |
                  0b01101010,  //Number 4          // ---g----
                  0b01010010,  //Number 5          // |      |
                  0b00010010,  //Number 6          // e      c
                  0b11100110,  //Number 7          // |      |
                  0b00000010,  //Number 8          // ---d----
                  0b01000010,   //Number 9
                  0b11111111   //Blank LED 
                };  

int main(void){
	//---------------------------------
	// System initialization
	//---------------------------------
	System_Config();
	SPI3_Config();
	UART0_Config();
	//---------------------------------
	// LCD initialization
	//---------------------------------
	LCD_start();
	LCD_clear();
	
	// GPIO configuration
	// BUZZER - interrupt handling routine
	PB->PMD &= ~(0x03<<22);	
	PB->PMD |= (0x01<<22);
	
	// LED5 toggle
	PC->PMD &= ~(0x03<<24);			// clear bits
	PC->PMD |= (0x01<<24);			// output pc12
	
	// GPIO GP.15 button interrupt
	PB->PMD &= (~(0x03 << 30)); // input mode
	PB->IMD &= (~(1 << 15)); 		// Edge trigger interrupt
	PB->IEN |= (1 << 15); 			// interrupt enabled by falling edge
	
	//NVIC interrupt configuration for GPIO-B15 interrupt source
	NVIC->ISER[0] |= 1 << 3;
	NVIC->IP[0] &= (~(3 << 30));			// IP[0] PRI3
	
	// Debounce configuration
	PB->DBEN |= (1<<15);								// int1 debounce
	GPIO->DBNCECON &= ~(0xF << 0);
	GPIO->DBNCECON |= (0b0111 << 0); 		// sample interrupt input once per 128 clocks: 12MHz/128 = 93.75kHz
	GPIO->DBNCECON |= (1<<4); 					// Debounce counter clock source is the internal 10kHz low speed oscillator 
	
	//--------------------------------
	//GPIO for key matrix
	//--------------------------------
	KeyPadEnable();
	
	//Configure GPIO for 7segment
	//Set mode for PC4 to PC7 - output push-pull
	GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);
	GPIO_SetMode(PC, BIT5, GPIO_MODE_OUTPUT);
	GPIO_SetMode(PC, BIT6, GPIO_MODE_OUTPUT);
	GPIO_SetMode(PC, BIT7, GPIO_MODE_OUTPUT);		
	//Set mode for PE0 to PE7 - output push-pull
	GPIO_SetMode(PE, BIT0, GPIO_MODE_OUTPUT);		
	GPIO_SetMode(PE, BIT1, GPIO_MODE_OUTPUT);		
	GPIO_SetMode(PE, BIT2, GPIO_MODE_OUTPUT);		
	GPIO_SetMode(PE, BIT3, GPIO_MODE_OUTPUT);		
	GPIO_SetMode(PE, BIT4, GPIO_MODE_OUTPUT);		
	GPIO_SetMode(PE, BIT5, GPIO_MODE_OUTPUT);		
	GPIO_SetMode(PE, BIT6, GPIO_MODE_OUTPUT);		
	GPIO_SetMode(PE, BIT7, GPIO_MODE_OUTPUT);		
	
	while (1){
		PC->DOUT &= ~(1<<14);				// LED8 - program execution
		Game_Algorithm();						// initiate the game algorithm
	}
}

//---------------------------------------------------------------------------------------------------
//------------------------------------------ SYSTEM CPU CLOCK SETUP-----------------------------------
// CPU clock system configuration
void System_Config(void){
	SYS_UnlockReg();				// Unlock protected bits
	
	// enable clock source
	CLK->PWRCON |= (1<<0);
	while (!(CLK->CLKSTATUS & HXT_STATUS));
	
	// PLL configuration starts
	CLK->PLLCON &= ~(1<<19);		//0: PLL input is HXT source
	CLK->PLLCON &= ~(1<<16);		// normal mode
	CLK->PLLCON &= ~(0x01FF<<0);
	CLK->PLLCON |= 48;
	CLK->PLLCON &= ~(1<<18);		// PLLOUT
	while (!(CLK->CLKSTATUS & PLL_STATUS));
	// PLL configuration ends
	
	// CPU clock source selection
	CLK->CLKSEL0 &= (~(0x07 << 0));
	CLK->CLKSEL0 |= (0x02 << 0);    
	//clock frequency division
	CLK->CLKDIV &= (~0x0F << 0);
	
	enable_TMR0();		// Enable timer0
	
	//UART0 Clock selection and configuration
	CLK->CLKSEL1 |= (0b11 << 24); // UART0 clock source is 22.1184 MHz
	CLK->CLKDIV &= ~(0xF << 8); 	// clock divider is 1
	CLK->APBCLK |= (1 << 16); 		// enable UART0 clock
	
	// Enable SPI3
	CLK->APBCLK |= (1<<15);
	
	SYS_LockReg();
}

//----------------------------------------------------------------------------------------------------------
//-------------------------------------------- TIMER0 SETUP---------------------------------------------------
// Timer0 configuration
void enable_TMR0(void){
	// CONFIG CLOCK, OPERATING MODE, AND VALUE FOR TIMER0 STARTS------
	// Configure clock timer0
	CLK->CLKSEL1 &= ~(0x111 << 8);		// clear bits 
	CLK->CLKSEL1 |= (0b010 << 8);			// select clock source HCLK - 12MHz
	CLK->APBCLK |= (1 << 2);					// enable timer0
	// Set prescaler
	TIMER0->TCSR &= ~(0xFF << 0);			// reset timer0
	// Opearting mode
	TIMER0->TCSR |= (1 << 26);				// Reset Timer0
	TIMER0->TCSR &= ~(0b11 << 27);		// one-shot mode
	TIMER0->TCSR |= (0b01 << 27);			// timer operate at periodic mode
	TIMER0->TCSR &= ~(1 << 24);			  // no effect
	//TIMER0->TCSR |= (1 << 16);				// TDR is updated continuously while counter is counting
	// Value
	TIMER0->TCMPR = TMR0_COUNTS;			// 
	TIMER0->TCSR |= (1 << 29);				// Enable timer interrupt flag TIF	
	TIMER0->TCSR |= (1 << 30);				// Start counting
	
	//Set Timer0 in NVIC Set-Enable Control Register (NVIC_ISER)
	NVIC->ISER[0] |= 1 << 8;
	// CONFIG CLOCK, OPERATING MODE, AND VALUE FOR TIMER0 ENDS-------
}

// Timer0 interrupt - for time transition each 7Segment LED to avoid bouncing
void TMR0_IRQHandler(void){
	TimePassed = 1;						// Time taken/time delay to scan LEDs transition 
	TIMER0->TISR |= (1<<0);		// generate interrupt
}

//--------------------------------------------------------------------------------------------------
//------------------------------------------ BUTTON PB.15 INTERRUPT ---------------------------------- 
// PB.15 interrupt 
void EINT1_IRQHandler(void) 
{
	switch (state)	
	{
		case welcome://start game
			LCD_clear();	
			CLK_SysTickDelay(20000);
			state = game_loading;	
			break;
		
		case gameplay://shot
			// If shoots lands onto the ship coordinates
			if(map[cord[1]-1][cord[0]-1]==1) 		// Access to coordinate of "1"
				{
					map[cord[1]-1][cord[0]-1]=2;		// Assign value to 2 when hit 
					score++;
				}
			if (shot<16) shot++;
			state = game_loading;			// Upload the map
			break;
			
		case gameover://restart
			LCD_clear();	
			state = loading;				// Return to first state - loading state
			break;	
	}  
	PB->ISRC |= (1 << 15);			// Generate interrupt
}


//-----------------------------------------------------------------------------------------------------
//--------------------------------------------UART0 SETUP---------------------------------------------
// UART0 congiguration
void UART0_Config(void){
	// UART0 pin configuration. PB.1 pin is for UART0 TX
	PB->PMD &= ~(0b11 << 2);
	PB->PMD |= (0b01 << 2); 		// PB.1 is output pin
	SYS->GPB_MFP |= (1 << 1); 	// GPB_MFP[1] = 1 -> PB.1 is UART0 TX pin
	SYS->GPB_MFP |= (1 << 0); 	// GPB_MFP[0] = 1 -> PB.0 is UART0 RX pin
	PB->PMD &= ~(0b11 << 0); 		// Set Pin Mode for GPB.0 - (RX - Input)
	
	// UART0 operation configuration
	UART0->LCR |= (0b11 << 0); 	// 8 data bit
	UART0->LCR &= ~(1 << 2); 		// one stop bit	
	UART0->LCR &= ~(1 << 3); 		// no parity bit
	UART0->FCR |= (1 << 1); 		// clear RX FIFO
	UART0->FCR |= (1 << 2); 		// clear TX FIFO
	UART0->FCR &= ~(0xF << 16); // FIFO Trigger Level is 1 byte]
	
	//Baud rate config: BRD/A = 1, DIV_X_EN=0
	//--> Mode 0, Baud rate = UART_CLK/[16*(A+2)] = 22.1184 MHz/[16*(142+2)]= 9600 bps
	UART0->BAUD &= ~(0b11 << 28); // mode 0	
	UART0->BAUD &= ~(0xFFFF << 0);
	UART0->BAUD |= 142;	
	
	// UART0 interrupt
	UART0->IER |= (1 << 0); 		// enable interrupt
	UART0->FCR |= (0000 << 4); 	// RX interrupt trigger level
	
	// UART interrupt priority	
	NVIC->ISER[0] |= 1<<12;
	NVIC->IP[1] &= (~(0b11<<22));		// IP[1] - PRI2
}

// Sending map file from module - TX
void UART0_sendChar(int ch){
	while (UART0->FSR & (0x01<<23));	// wait til TX FIFO is not full
	UART0->DATA = ch;
	if (ch == '\n'){				// ch read new line
		while (UART0->FSR & (0x01<<23));
		UART0->DATA = '\r';		// carriage return, next line
	}
}

// UART0 interrupt - RX
void UART02_IRQHandler(void){
	// Receive the input data from TX 
	ReceivedByte = UART0->DATA;		
	UART0_sendChar(ReceivedByte);
		
	// Reconstruct the map from the TX read data
	if (ReceivedByte == '0' || ReceivedByte == '1'){
		if (ReceivedByte == '0') map[row][col] = 0;
		if (ReceivedByte == '1') map[row][col] = 1;	
		// Create the map from read file
		if (col == 7)
		{
			col = 0;
			if (row == 7)
			{
				// Stop RX
				ReadMap = 1;		// set flag to 1
				row = 0;
			} else row++;
		} else col++;
	}
}

//---------------------------------------------------------------------------------------------
//--------------------------------------- GAME ALGORITHM SETUP ------------------------------------
// Set up the game algorith from each state to shooting
void Game_Algorithm(void){
	switch(state){
		
		//---------------------- loading state ------------------------------------------------
		case loading:			// welcome screen - start game - Reset every var
			for (int i=0; i<8;i++){
				for (int j=0; j<8; j++){
					if (map[i][j]==2){				// Reset all index value of shot-landed coordinate X - 1
						map[i][j]=1;
					} 
				}
			} 
			// Reset all values when game restarts
			row = 0;
			col = 0;
			shot = 0;
			score = 0;
			score_key = 0;
			Buzzer_on = 0;
			cord[0] = 0;
			cord[0] = 0;
			
			// Display context
			printS_5x7(1, 30, "Loading game. Please wait");
			CLK_SysTickDelay(3000000);
			LCD_clear();
			
			state = welcome;		// state transition
			break;
			
		//---------------------- Welcome state ------------------------------------------------
		case welcome:					// Display welcome screen
			printS_5x7(2, 0, "  EEET2481 Final ASM Q2");
			printS_5x7(0, 16, "      BATTLESHIP GAME     ");
			printS_5x7(0, 24, "--------------------------");
			printS_5x7(0, 45, "    Press PB15 to begin!  ");
			if (ReadMap > 0) 
				printS_5x7(0, 32, "  Map Loaded Successfully.");
			break;
			
		//---------------------- Game loading state --------------------------------------------------
		case game_loading:		// Loading the map
			LCD_clear();
			int indexRow = 2;
			int indexCol = 0;
		
			// Display the 8x8 map from the file
			for (int i=0; i<8;i++){
				for (int j=0; j<8;j++)
				{
					// Generate the map battleship
					if (map[i][j]!=2) printS_5x7(indexRow, indexCol, "-");	// Display the coordinate of "0" as "-"
					else printS_5x7(indexRow, indexCol, "X");						// Display the coordinate of "1" as "X"
					indexRow = indexRow+8;
				}
				indexCol = indexCol+8;
				indexRow = 2;
			}
			// Print score
			sprintf(score_txt, "%d", score);
			printS_5x7(72, 0, "Score");
			printS_5x7(82, 8, score_txt);
			state = gameplay;				// state transition
			break;
		
		//------------------------ Gameplay state --------------------------------------------------
		case gameplay:			// game play
			// Main State code 
			Game_Display();
			KeyPressed = KeyPadScanning();		// check key
			
			//Print out the current selected cordination X or Y for U11
			printS_5x7(105, 0, "COR");
			if (Seg7_XY == 1) printS_5x7(110, 8, "Y");
			else printS_5x7(110, 8, "X");
			
				
			if(KeyPressed == 9)
			{
				//Change coordination whenever K9 is pressed
				if (Seg7_XY == 1) Seg7_XY = 0;
				else Seg7_XY = 1;
					
				if(current_Cord == 0) current_Cord=1; 
				else current_Cord=0;
				CLK_SysTickDelay(BOUNCING_DELAY);
				
			} else { 
				if (KeyPressed != 0) 
				{
					// Update the coordinate when Key matrix pressed
					cord[current_Cord] = KeyPressed;
					CLK_SysTickDelay(BOUNCING_DELAY);
				}
			}
			KeyPressed = 0;
				
			//If shoot more than 15 times
			if (shot > 15) 
			{
				LCD_clear();
				state = gameover;		// state transition to gameover
			}
				
			//If shoot all 5 ships
			if (score ==10) 
			{
				LCD_clear();
				state = gameover; // state transition to gameover
			}
				
			// Identify if shoot hit partially of a ship
			if (score > score_key)
			{
				for(int i = 0; i < 6; i++)
				{
					PC->DOUT ^= 1 << 12;				// LED5 flash 3 time
					CLK_SysTickDelay(500000);
				}
				score_key = score;		// Update the score_key
			} 
			break;
			
		//-------------------------- Gameover state -----------------------------------------------
		case gameover:		// end game 
			//end_game code here
			// Turn off all 7Seg and LED5
			PC->DOUT &= ~(1<<7);    //0: U11
			PC->DOUT &= ~(1<<6);		//0: U12
			PC->DOUT &= ~(1<<5);		//0: U13
			PC->DOUT &= ~(1<<4);		//0: U14

			char FinalScore[] = "00";
			if (score == 10) {
				printS_5x7(0, 5, "     ALL SHIPS SUNK!  ");
				printS_5x7(0, 14, "         YOU WIN.      ");
			
			} else {
				printS_5x7(0, 5, "     RAN OUT OF SHOTS!  ");
				printS_5x7(0, 14, "        YOU LOSE.      ");
			}
			
			sprintf(FinalScore, "%d", score);	
			printS_5x7(0, 27, "Final Score:");
			printS_5x7(0 + 5*12, 27, FinalScore);
		
			printS_5x7(0, 35, "--------------------------");
			printS_5x7(0, 43, "   Press PB15 to restart!  ");
				
			if(Buzzer_on == 0)
			{
				Buzzer_Beep(BUZZER_BEEP_TIME);
				Buzzer_on = 1;				// Turn off Buzzer_on key
			}
			CLK_SysTickDelay(20000);
			break;
		
		default: break;
	}
}

//-----------------------------------------------------------------------------------------------------
//------------------------------------------- LED GAME DISPLAY ----------------------------------------
// Setup the U11 and LED5 for game algorithm
void Game_Display(void){
	//Select the 7segment one by one and display cordinate and number of shot
	while (TimePassed != 1);
	TimePassed = 0;
	// Display the X or Y 7 Segment coordinate
	if(Seg7_XY == 1) PE->DOUT = pattern[cord[1]];		// Y
	else PE->DOUT = pattern[cord[0]];								// X
	
	// 7 Segments
	PC->DOUT |= (1<<7);     //1: U11
	PC->DOUT &= ~(1<<6);		//0: U12
	PC->DOUT &= ~(1<<5);		//0: U13
	PC->DOUT &= ~(1<<4);		//0: U14	
		 

	if(shot != 0)
	{
		if(shot/10 == 1)
		{
			while (TimePassed != 1);		
			TimePassed = 0;
			//Select the 7segment U13
			PE->DOUT = pattern[1];
			PC->DOUT &= ~ (1<<7);   //0: U11
			PC->DOUT &= ~(1<<6);		//0: U12
			PC->DOUT |=(1<<5);			//1: U13
			PC->DOUT &= ~(1<<4);		//0: U14
		}
					
		while (TimePassed != 1);		
		TimePassed = 0;;
		//Select the 7segment U14
		PE->DOUT = pattern[shot%10];
		PC->DOUT &= ~ (1<<7);    	//0: U11
		PC->DOUT &= ~(1<<6);			//0: U12
		PC->DOUT &= ~(1<<5);			//0: U13
		PC->DOUT |=(1<<4);				//1: U14
		while (TimePassed != 1);
		TimePassed = 0;
	}
}

//------------------------------------------------------------------------------------------------------
//------------------------------------------- KEY MATRIX SETUP --------------------------------------------
// Setup GPIO mode for key matrix
void KeyPadEnable(void) {
	GPIO_SetMode(PA, BIT0, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT1, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT2, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT3, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT4, GPIO_MODE_QUASI);
	GPIO_SetMode(PA, BIT5, GPIO_MODE_QUASI);
}

// Function scan key pressed
uint8_t KeyPadScanning(void) {
	// Col 1
	PA0 = 1; PA1 = 1; PA2 = 0; PA3 = 1; PA4 = 1; PA5 = 1;
	if (PA3 == 0) return 1;
	if (PA4 == 0) return 4;
	if (PA5 == 0) return 7;
	// Col 2
	PA0 = 1; PA1 = 0; PA2 = 1; PA3 = 1; PA4 = 1; PA5 = 1;
	if (PA3 == 0) return 2;
	if (PA4 == 0) return 5;
	if (PA5 == 0) return 8;
	// Col 3
	PA0 = 0; PA1 = 1; PA2 = 1; PA3 = 1; PA4 = 1; PA5 = 1;
	if (PA3 == 0) return 3;
	if (PA4 == 0) return 6;
	if (PA5 == 0) return 9;
	return 0;
}

//-------------------------------------------------------------------------------------------------------
//--------------------------------------------- LCD SETUP --------------------------------------------------
// SPI3 Config
void SPI3_Config(void){
	SYS->GPD_MFP |= 1 << 11; 	//1: PD11 is configured for alternative function
	SYS->GPD_MFP |= 1 << 9; 	//1: PD9 is configured for alternative function
	SYS->GPD_MFP |= 1 << 8; 	//1: PD8 is configured for alternative function

	SPI3->CNTRL &= ~(1 << 23); 	//0: disable variable clock feature
	SPI3->CNTRL &= ~(1 << 22);	//0: disable two bits transfer mode
	SPI3->CNTRL &= ~(1 << 18); 	//0: select Master mode
	SPI3->CNTRL &= ~(1 << 17); 	//0: disable SPI interrupt
	SPI3->CNTRL |= 1 << 11; 		//1: SPI clock idle high
	SPI3->CNTRL &= ~(1 << 10); 	//0: MSB is sent first
	SPI3->CNTRL &= ~(3 << 8); 	//00: one transmit/receive word will be executed in one data transfer

	SPI3->CNTRL &= ~(31 << 3); //Transmit/Receive bit length
	SPI3->CNTRL |= 9 << 3;     //9: 9 bits transmitted/received per data transfer

	SPI3->CNTRL |= (1 << 2);  //1: Transmit at negative edge of SPI CLK
	SPI3->DIVIDER = 0; 				// SPI clock divider. SPI clock = HCLK / ((DIVIDER+1)*2). HCLK = 50 MHz
}

void LCD_start(void)
{
	LCD_command(0xE2); // Set system reset
	LCD_command(0xA1); // Set Frame rate 100 fps
	LCD_command(0xEB); // Set LCD bias ratio E8~EB for 6~9 (min~max)
	LCD_command(0x81); // Set V BIAS potentiometer
	LCD_command(0xA0); // Set V BIAS potentiometer: A0 ()
	LCD_command(0xC0);
	LCD_command(0xAF); // Set Display Enable
}

void LCD_command(unsigned char temp)
{
	SPI3->SSR |= 1 << 0;
	SPI3->TX[0] = temp;
	SPI3->CNTRL |= 1 << 0;
	while (SPI3->CNTRL & (1 << 0));
	SPI3->SSR &= ~(1 << 0);
}

void LCD_data(unsigned char temp)
{
	SPI3->SSR |= 1 << 0;
	SPI3->TX[0] = 0x0100 + temp;
	SPI3->CNTRL |= 1 << 0;
	while (SPI3->CNTRL & (1 << 0));
	SPI3->SSR &= ~(1 << 0);
}

void LCD_clear(void)
{
	int16_t i;
	LCD_SetAddress(0x0, 0x0);
	for (i = 0; i < 132 * 8; i++)
	{
		LCD_data(0x00);
	}
}

void LCD_SetAddress(uint8_t PageAddr, uint8_t ColumnAddr)
{
	LCD_command(0xB0 | PageAddr);
	LCD_command(0x10 | (ColumnAddr >> 4) & 0xF);
	LCD_command(0x00 | (ColumnAddr & 0xF));
}

//-----------------------------------------------------------------------------------------------------
//----------------------------------------- BUZZER FUNCTION SETUP--------------------------------------
// Buzzer function
void Buzzer_Beep(int Beep_time){
	for (int i=0; i<Beep_time; i++){
		PB->DOUT ^= (1<<11);
		CLK_SysTickDelay(BUZZER_BEEP_DELAY);
	} 
}

//------------------------------------------- main.c CODE ENDS ---------------------------------------------------------------------------------------
