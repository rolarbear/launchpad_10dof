//
//*****************************************************************************
//
// master_slave_loopback.c - Example demonstrating a simple I2C message
// transmission and reception.
//
// Copyright (c) 2010-2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 9453 of the Stellaris Firmware Development Package.
//
//*****************************************************************************
#define PART_LM4F120H5QR TRUE
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "utils/uartstdio.c"
#include "driverlib/pin_map.h"

#ifdef DEBUG
void __error__(char *pcFilename, unsigned long ulLine) {
}
#endif

#define BMP085_ADDRESS				0x77  // address of pressure sensor
#define ADXL345_ADDRESS				0x53  // address of accelerometer
#define HMC5883L_ADDRESS			0x1E  // address of compass sensor
#define L3G4200D_ADDRESS			0x68  // address of gyroscope
#define ACCEL_VALUES				0x32  // start of the 6 memory locations that hold xyz accelerometer data
#define DATA_FORMAT					0x31  // accelerometer
#define FIFO_REG					0x38  // accelerometer
#define PRESS_SEN_READ_ADDRESS		0xF4  // address for reading values on BMP085
#define PRESS_TEMP_VALUE			0x2E  // value to write into PRESS_SEN_READ_ADDRESS to get uncompensated temperature value
#define UP_REGISTER					0xF6  // address for reading uncompensated values
#define DUMMY						0x00
#define NUM_SSI_DATA				4
#define HIGH						0xFF
#define LOW							0x00

//for holding values of all sensors
unsigned long gyroX, gyroY, gyroZ, compX, compY, compZ;

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
int xVal, yVal, zVal, pressVal;

//print with the uart?
tBoolean printUart;

unsigned long ulDataRx[NUM_SSI_DATA];	//changed from unsigned long
int ulDataTx[NUM_SSI_DATA];				//changed from unsigned long
int ulindex;							//changed from unsigned long
int ulDummyRx;							//changed from unsigned long

short flash_buff_mark = 0;
volatile short nasty[33];

void delay(unsigned long time) {
	SysCtlDelay(time);
}

void flash_clk(void) {
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, HIGH);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, LOW);
}

void InitConsole(void) {
	//
	// Enable GPIO port A which is used for UART0 pins.
	// TODO: change this to whichever GPIO port you are using.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Configure the pin muxing for UART0 functions on port A0 and A1.
	// This step is not necessary if your part does not support pin muxing.
	// TODO: change this to select the port/pin you are using.
	//

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

	//
	// Select the alternate (UART) function for these pins.
	// TODO: change this to select the port/pin you are using.
	//
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioInit(0);
}

void InitBtConsole(void) {
	//
	// Enable GPIO port A which is used for UART0 pins.
	// TODO: change this to whichever GPIO port you are using.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//
	// Configure the pin muxing for UART0 functions on port A0 and A1.
	// This step is not necessary if your part does not support pin muxing.
	// TODO: change this to select the port/pin you are using.
	//

	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);

	//
	// Select the alternate (UART) function for these pins.
	// TODO: change this to select the port/pin you are using.
	//

	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	IntEnable(INT_UART1);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioInit(1);

	UARTprintf("\n\nBT CONSOLE\n");
}

void flashErase(void) {

//	ssiClk();
	delay(500);
	unsigned long ulDataTx[NUM_SSI_DATA];
	unsigned long ulindex;

	ulDataTx[0] = 0x50;
	ulDataTx[1] = 0x94;
	ulDataTx[2] = 0x80;
	ulDataTx[3] = 0x9A;

	//while(SSIDataGetNonBlocking(SSI0_BASE,&ulDataRx[0])){}

	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, LOW);

	for (ulindex = 0; ulindex < 4; ulindex++) {
		//
		// Send the data using the "blocking" put function.  This function
		// will wait until there is room in the send FIFO before returning.
		// This allows you to assure that all the data you send makes it into
		// the send FIFO.
		//
		SSIDataPut(SSI0_BASE, ulDataTx[ulindex]);
		SSIDataGet(SSI0_BASE, &ulDummyRx);
	}
	while (SSIBusy(SSI0_BASE)) {
	}
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, HIGH);

}

void ssi(void) {
	// The SSI0 peripheral must be enabled for use.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	//
	// For this example SSI0 is used with PortA[5:2].  The actual port and pins
	// used may be different on your part, consult the data sheet for more
	// information.  GPIO port A needs to be enabled so these pins can be used.
	// TODO: change this to whichever GPIO port you are using.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//
	// Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
	// This step is not necessary if your part does not support pin muxing.
	// TODO: change this to select the port/pin you are using.
	//
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);
	GPIOPinConfigure(GPIO_PB2_T3CCP0);

	//
	// Configure the GPIO settings for the SSI pins.  This function also gives
	// control of these pins to the SSI hardware.  Consult the data sheet to
	// see which functions are allocated per pin.
	// The pins are assigned as follows:
	//      PA5 - SSI0Tx
	//      PA4 - SSI0Rx
	//      PA3 - SSI0Fss
	//      PA2 - SSI0CLK
	// TODO: change this to select the port/pin you are using.
	//
	GPIOPinTypeSSI(GPIO_PORTA_BASE,
			GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);

	//
	// Configure and enable the SSI port for SPI master mode.  Use SSI0,
	// system clock supply, idle clock level low and active low clock in
	// freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
	// For SPI mode, you can set the polarity of the SSI clock when the SSI
	// unit is idle.  You can also configure what clock edge you want to
	// capture data on.  Please reference the datasheet for more information on
	// the different SPI modes.
	//

	//TODO: check ssi protocol

	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3,
			SSI_MODE_MASTER, 1000000, 8);

	//
	// Enable the SSI0 module.
	//
	SSIEnable(SSI0_BASE);

	//
	// Read any residual data from the SSI port.  This makes sure the receive
	// FIFOs are empty, so we don't read any unwanted junk.  This is done here
	// because the SPI SSI mode is full-duplex, which allows you to send and
	// receive at the same time.  The SSIDataGetNonBlocking function returns
	// "true" when data was returned, and "false" when no data was returned.
	// The "non-blocking" function checks if there is any data in the receive
	// FIFO and does not "hang" if there isn't.
	//

	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, HIGH);
	/*
	 GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);
	 SSIDataPut(SSI0_BASE, 0x00);
	 SSIDataGet(SSI0_BASE,&ulDummyRx);
	 GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_3, 0x01);
	 while(SSIBusy(SSI0_BASE))
	 {
	 }
	 */
}

void i2c(void) {

//setup i2c for sensors
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

//
	I2CMasterEnable(I2C1_MASTER_BASE);

	GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	GPIOPinConfigure(GPIO_PA6_I2C1SCL); //Setup Mux

	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

//   //Sets Clock Rate of I2C - 100kbps

	I2CMasterInitExpClk(I2C1_MASTER_BASE, SysCtlClockGet(), false);
	delay(10000);
}

void sensorWrite(unsigned long device, unsigned long memLocation,
		unsigned long memValue) {
	unsigned long memoryRead;
	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device, false);

	I2CMasterDataPut(I2C1_MASTER_BASE, memLocation);
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}

	I2CMasterDataPut(I2C1_MASTER_BASE, memValue);
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}

	I2CMasterDataPut(I2C1_MASTER_BASE, memLocation);
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}

	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, device, true);

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}
	memoryRead = I2CMasterDataGet(I2C1_MASTER_BASE);

	if (printUart == true) {
		UARTprintf("Register %2d (should be %2d): %2d\n", memLocation, memValue,
				memoryRead);
	}
}

void accelRead(void) {

	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, ADXL345_ADDRESS, false);

	I2CMasterDataPut(I2C1_MASTER_BASE, ACCEL_VALUES);
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}

	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, ADXL345_ADDRESS, true);

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}
	xVal = I2CMasterDataGet(I2C1_MASTER_BASE);

	xVal = xVal << 8;

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}
	xVal |= I2CMasterDataGet(I2C1_MASTER_BASE);

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}
	yVal = I2CMasterDataGet(I2C1_MASTER_BASE);

	yVal = yVal << 8;

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}
	yVal |= I2CMasterDataGet(I2C1_MASTER_BASE);

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}
	zVal = I2CMasterDataGet(I2C1_MASTER_BASE);

	zVal = zVal << 8;

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}
	zVal |= I2CMasterDataGet(I2C1_MASTER_BASE);
}

void pressRead(void) {
	
	/*
	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, BMP085_ADDRESS, false);

	I2CMasterDataPut(I2C1_MASTER_BASE, PRESS_REGISTER_ADDRESS);
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}

	I2CMasterDataPut(I2C1_MASTER_BASE, PRESS_CONTROL_VALUE);
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}

	delay(135);

	I2CMasterDataPut(I2C1_MASTER_BASE, UP_REGISTER);
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}

	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, BMP085_ADDRESS, true);

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}
	pressVal = I2CMasterDataGet(I2C1_MASTER_BASE);
	pressVal = pressVal << 8;
	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, BMP085_ADDRESS, true);

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}
	pressVal |= I2CMasterDataGet(I2C1_MASTER_BASE);
	*/
	pressVal = bmp085ReadInt(PRESS_CONTROL_VALUE);
	
}

void idRead(void) {
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, LOW);
	SSIDataPut(SSI0_BASE, 0x9F);
	SSIDataGet(SSI0_BASE, &ulDummyRx);
	while (SSIBusy(SSI0_BASE)) {
	}
	for (ulindex = 0; ulindex < NUM_SSI_DATA; ulindex++) {
		SSIDataPut(SSI0_BASE, DUMMY);
		SSIDataGet(SSI0_BASE, &ulDataRx[ulindex]);
		ulDataRx[ulindex] &= 0x00FF;
		UARTprintf("'%d' ", ulDataRx[ulindex]);
	}
	while (SSIBusy(SSI0_BASE)) {
	}
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, HIGH);
}
void dummyWrite(int num) {
	int i;
	for (i = 0; i < num; i++) {
		SSIDataPut(SSI0_BASE, DUMMY);
		SSIDataGet(SSI0_BASE, &ulDummyRx);
	}
}
void memRead(void) {
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, LOW);
	SSIDataPut(SSI0_BASE, 0x0B);
	SSIDataGet(SSI0_BASE, &ulDummyRx);
	SSIDataPut(SSI0_BASE, 0x00);
	SSIDataGet(SSI0_BASE, &ulDummyRx);
	dummyWrite(3);
	while (SSIBusy(SSI0_BASE)) {
	}
	for (ulindex = 0; ulindex < 8; ulindex++) {
		SSIDataPut(SSI0_BASE, DUMMY);
		SSIDataGet(SSI0_BASE, &ulDataRx[ulindex]);
		UARTprintf("'%d' ", ulDataRx[ulindex]);
	}
	while (SSIBusy(SSI0_BASE)) {
	}
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, HIGH);
}

void memWrite(void) {
	char i;
	short x, y;

	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, LOW);
	SSIDataPut(SSI0_BASE, 0x84);
	SSIDataGet(SSI0_BASE, &ulDummyRx);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, LOW);
	for (i = 0; i < 14; i++) {
		flash_clk();
	}
	x = 0;
	for (i = 0; i < 10; i++) {
		if (x & 0x200)
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, HIGH); //SI high
		else
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, LOW); //SI low

		flash_clk();

		x <<= 1;
	}
	for (x = 0; x < 33; x++) {
		y = nasty[x];

		for (i = 0; i < 16; i++) {
			if (y & 0x8000)
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, HIGH); //SI high
			else
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, LOW); //SI low

			flash_clk();

			y <<= 1;
		}
	}
}

int bmp085ReadInt(unsigned char address) {
	unsigned char msb, lsb;

	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, BMP085_ADDRESS, false);

	I2CMasterDataPut(I2C1_MASTER_BASE, PRESS_REGISTER_ADDRESS);
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while (I2CMasterBusy(I2C1_MASTER_BASE))	{
	}

	//delay(135);

	I2CMasterDataPut(I2C1_MASTER_BASE, address);
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(I2C1_MASTER_BASE))	{
	}

	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, BMP085_ADDRESS,true);

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(I2CMasterBusy(I2C1_MASTER_BASE)){
	}
	msb = I2CMasterDataGet(I2C1_MASTER_BASE);
	I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, BMP085_ADDRESS, true);

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while (I2CMasterBusy(I2C1_MASTER_BASE)) {
	}
	lsb = I2CMasterDataGet(I2C1_MASTER_BASE);

	return ((msb<<8) | (lsb));
}

void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

void sensorPrint(void) {
	UARTprintf("Accelerometer:  ");
//	UARTprintf(" X: %4d%4d ", dataX[1],dataX[0]);
//	UARTprintf(" Y: %4d%4d ", dataY[1],dataY[0]);
//	UARTprintf(" Z: %4d%4d ", dataZ[1],dataZ[0]);
	UARTprintf("X: %8d ", xVal);
	UARTprintf("Y: %8d ", yVal);
	UARTprintf("Z: %8d ", zVal);
	UARTprintf(" Pressure: %8d\r", pressVal);
}

int main(void) {
	unsigned char menuSel;
	unsigned char printSel;

	SysCtlClockSet(
			SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN
					| SYSCTL_XTAL_16MHZ);

	InitConsole();
	//InitBtConsole();

	while (1) {
		UARTprintf("\n\nHOWDY! What would you like to do?\n");
		UARTprintf("1. Read first 8 mem locations\n");
		UARTprintf("2. Read Flash ID\n");
		UARTprintf("3. Display Sensor Data\n");
		UARTprintf("4. Write Memory Locations\n");
		UARTprintf("5. Erase Flash\n");
//	ssi();
//	i2c();

		menuSel = UARTgetc();

		if (menuSel == '1') {
			ssi();
			memRead();
		}

		if (menuSel == '2') {
			ssi();
			delay(100);
			idRead();
		}

		if (menuSel == '3') {
			i2c();
			delay(100);
			UARTprintf("Print out readings? (0 for no, 1 for yes)\n");
			printSel = UARTgetc();

			if (printSel == '1') {
				printUart = true;
			} else {
				printUart = false;
			}

			sensorWrite(ADXL345_ADDRESS, 0x2D, 0x08);
			sensorWrite(ADXL345_ADDRESS, DATA_FORMAT, 0x0B);
			sensorWrite(ADXL345_ADDRESS, FIFO_REG, 0x9F);

			while (1) {
				accelRead(ADXL345_ADDRESS);
				pressRead(BMP085_ADDRESS);
				if (printUart == true) {
					sensorPrint();
				}
				delay(50);
			}

		}
		if (menuSel == '4') {
			ssi();
			delay(100);
			memWrite();
			//int i;
			//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, LOW);
			//for(i = 0; i<20;i++)
			//{
			//	flash_clk();
			//}
			//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, HIGH);
		}
		if (menuSel == '5') {
			ssi();
			flashErase();
		}
	}

}
