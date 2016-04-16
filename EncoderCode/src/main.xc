/*
 * Intel Pulse Train Synth.xc
 *
 *  Created on: Sep 14, 2015
 *      Author: Colin
 */
#include <platform.h>
#include <xs1.h>
#include <timer.h>
#include <print.h>
#include <string.h>
#include <stdio.h>
//#include <spi.h>

#define TRUE  1
#define FALSE 0




#define LED_OFF 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3
#define LED_PETAL_RGB 4
#define LED_PETAL_RG  5
#define LED_PETAL_RB  6
#define LED_PETAL_GB  7
#define LED_SLICE_RGB 8
#define LED_SLICE_RG  9
#define LED_SLICE_RB  10
#define LED_SLICE_GB  11

//LED Mode bit flags
#define LED_PETAL_UPDATE 1
#define LED_SLICE_UPDATE 0
#define LED_R_ON 2
#define LED_G_ON 4
#define LED_B_ON 8


// Define I/O pins for FT221x USB to SPI serial converter.
out port PIN_VCP_CS = XS1_PORT_1D; // X0D10 //= XS1_PORT_1C; // X0D10
out port PIN_VCP_CLK = XS1_PORT_1C; // X0D11 //= XS1_PORT_1D; // X0D11

//RedPulse Pin 1 -> XD0 -> P1A0


out port PIN_ADC_CS = XS1_PORT_1A; // X0D10 //= XS1_PORT_1C; // X0D10


//GreenPulse Pin 2 -> XD2 -> P4A0
//BluePulse Pin 10 -> XD3 -> P4A1
//SPI_CS_ADC Pin 15 -> XD9 -> P4A3
port PORT_RGB = XS1_PORT_4A;

//#define GreenPulseMask 1 //0001 (A0)
//#define BluePulseMask 2 //0010 (A1)
//#define ADCMask 8 //1000 (A3)

//out port PIN_VCP_CS = XS1_PORT_1C; // X0D11 //= XS1_PORT_1D; // X0D11
//out port PIN_VCP_CLK = XS1_PORT_1D; // X0D10 //= XS1_PORT_1C; // X0D10


port PIN_VCP_MIOSI = XS1_PORT_1E; // X0D12
in port PIN_VCP_MISO = XS1_PORT_1F; // X0D13

in port PINS_ENCODER = XS1_PORT_4C; // X0D14
//in  port PIN_ENCODER_A = XS1_PORT_; // X0D15
//in  port PIN_ENCODER_B = XS1_PORT_; // X0D20

//out port SYNC_OUT_A = XS1_PORT_1A; // X0D00
//out port SYNC_OUT_B = XS1_PORT_1B; // X0D01

out port SYNC_OUT_A = XS1_PORT_1B; // X0D01
//out port SYNC_OUT_B = XS1_PORT_1A; // X0D00


double curTilt;

interface iStatus
{

	void
	countsAtIndex(int counts);
}; // status


interface commands
{
	void setPetalOffset(int offset);
	void setFrameLen(int len);
	void setFrameCount(int count);
	void setPetalCount(int count);
	void enablePwm(int enable);
	void flushBuffer();
	void ledMode(char mode);
	int getRPM();
};


void setRGB_Value(char val)
{
	switch(val)
	{
		case LED_OFF:
			PORT_RGB <: 0;
			break;
		case LED_RED:
			PORT_RGB <: 2;
			break;
		case LED_GREEN:
			PORT_RGB <: 1;
			break;
		case LED_BLUE:
			PORT_RGB <: 4;
			break;
	}
}

int incRgbState(int curCount, int ledMode)
{
	int count = ++curCount;
	if(count > 3)
		count = 1;

	//Skip a number if it's not enabled (used for bi-color
	if(count == 1 && ((ledMode & LED_R_ON) == 0))
	{
		count = 2;
	}
	else if(count == 2 && ((ledMode & LED_G_ON) == 0))
	{
		count = 3;
	}
	else if(count == 3 && ((ledMode & LED_B_ON) == 0))
	{
		count = 1;
	}

	return count;
}

int getNumLedsOn(int ledMode)
{
	int numLeds = 0;

	if((ledMode & LED_R_ON) == LED_R_ON)
		numLeds++;

	if((ledMode & LED_G_ON) == LED_G_ON)
			numLeds++;

	if((ledMode & LED_B_ON) == LED_B_ON)
			numLeds++;


	return numLeds;
}


void
VirtualCommPort(client interface commands cmd);

void
PulseTrainSynth(client interface iStatus status, server interface commands cmd)
{
	PORT_RGB <: 0;

	int encoder, encoder_previous;

	// All numbers in counts (40,000 per rev). 133k cps, 7.5us @ 200rpm
		int first_petal_start = 9200; //1900  //index pulse
		int petal_count = 4;
		int encoder_cpr = 40000; // 10,000 CPR * 4 (quadrature).
		int petal_offset[4];
		int frame_length = 40; // 10,000 per petal, %50, 5,000 / number of frames, = 100
		int frame_count_per_petal = 250;
		int sync_pulse_length = frame_length / 2;
		//int sync_pulse_length = 25;


		int counts_per_petal = (int)(((double)frame_length) * ((double)frame_count_per_petal) * 1);
		int cpr_jump_per_petal = encoder_cpr / petal_count;

		int count;

		int absolute_count = 0;
		int current_count = encoder_cpr - first_petal_start;


		int enablePwm = 1;
		char resetNeeded = 0;
		char reZeroed = 0;

		char ledMode = 0;
		char rgbState = 0;
		int sliceCounter = 0;
		int petalCounter = 0;
		char ledOn = 0;
		char sliceOn = 0;

		int tempEnPwm = enablePwm;
		int tempFrameLen = frame_length;
		int tempCountPerPetal = frame_count_per_petal;
		int tempPetalCount = petal_count;
		int tempPetalStart = first_petal_start;

		int i;

		timer curTimer;
		unsigned int curTime = 0;
		unsigned int lastTime = 0;
		unsigned long rpm = 0;


		while(1)
		{
			select
			{


				case PINS_ENCODER when pinsneq(encoder) :> encoder:
						curTimer :> curTime;
						if(lastTime != 0 && curTime > lastTime)
						{
							rpm = (curTime - lastTime)*4;
							rpm = 600000/rpm;
						}
						lastTime = curTime;

					/*
				if(!(encoder_previous & 0x02) && (encoder & 0x02) )
				{
					if((encoder & 0x04) )
					{
						absolute_count++;
						current_count++;
					}
					else
					{
						//absolute_count--;
						//current_count--;
					}
				}
				*/


				//On an encoder change:
				if(((encoder_previous & 0x06)) != (encoder & 0x06))
				{
					absolute_count++;
					current_count++;

					if(current_count >= encoder_cpr)
					{
						current_count -= encoder_cpr;

						if(resetNeeded == 1)
						{
							resetNeeded = 0;
							enablePwm = tempEnPwm;

							//New code for resetting the RGB state on refresh
							rgbState = 0;
							sliceCounter = 0;
							petalCounter = 1;
						}
					}
				}

				//On an encoder zero crossing
				if((!(encoder_previous & 0x01)) && (encoder & 0x01))
				{
					absolute_count = 0;
					current_count = encoder_cpr - first_petal_start;
					reZeroed = 1;
				}

				if(reZeroed && current_count < 10) //On or few counts past the adjusted zero mark
				{
					reZeroed = 0;
					sliceCounter = 0;
					petalCounter = 1;

					//Wait until the zero crossing before we reset to this RGB mode
					if(ledMode > 0 && rgbState == 0)
					{
						rgbState = incRgbState(rgbState, ledMode);
					}
				}

				if((current_count % frame_length) < sync_pulse_length && enablePwm == 1)
				{
					SYNC_OUT_A <: 1;

					if(sliceOn==0)
					{
						sliceOn = 1;
						sliceCounter++;

						//Slice Based RGB
						if(((ledMode & LED_PETAL_UPDATE) == LED_SLICE_UPDATE) && rgbState != 0)
						{
							/*
							rgbState++;

							if(rgbState > 3)
								rgbState = 1;
							*/
							rgbState = incRgbState(rgbState, ledMode);

							setRGB_Value(rgbState);
						}

						if(sliceCounter >= frame_count_per_petal)
						{
							sliceCounter -= frame_count_per_petal;
							petalCounter++;

							//Petal Based RGB
							if(((ledMode & LED_PETAL_UPDATE) == LED_PETAL_UPDATE) && rgbState != 0)
							{
								/*
								rgbState++;

								if(rgbState > 3)
									rgbState = 1;
								*/
								rgbState = incRgbState(rgbState, ledMode);
								setRGB_Value(rgbState);
							}

							if(petalCounter > petal_count)
							{
								petalCounter = 1;
							}
						}
					}
				}
				else
				{
					SYNC_OUT_A <: 0;
					if(sliceOn)
					{
						sliceOn = 0;
						//sliceCounter++;
					}

					/*
					if(sliceCounter == frame_count_per_petal)
					{
						sliceCounter = 0;
						ledOn = 0;
					}
					*/

				}



				/*

				// If we've swept through a whole revolution (all the petals) then
				// reset the current count to the actual number of counts since the
				// last index pulse to maintain syncronization.
				if(current_count >= first_petal_start + encoder_cpr)
				{
					//current_count = 0;
					current_count = absolute_count;


					//Reset all values at the start of the rotation
					if(resetNeeded == 1)
					{
						resetNeeded = 0;
						enablePwm = tempEnPwm;

						//frame_length = tempFrameLen;
						//frame_count_per_petal = tempCountPerPetal;
						//petal_count = tempPetalCount;
						//first_petal_start = tempPetalStart;

						//counts_per_petal = (int)(((double)frame_length) * ((double)frame_count_per_petal) * 1);
						//cpr_jump_per_petal = encoder_cpr / petal_count;

					}
				}

				*/

				/*

				count = current_count;

				// Deterime if we're at the first petal yet...
				//count -= first_petal_start;


				// and do nothing if we're not.
				//if(current_count > first_petal_start)
				if(count > 0 && !firstRun)
				{
					// Then see if we're within a petal.
					if(count % cpr_jump_per_petal < counts_per_petal)
					{




						//if(cur_petal_counts - half_off_petal_counts < frame_length)
						//SYNC_OUT_B <: 1;

						// Now see if we're close enough (within sync pulse length) to the
						// start of a frame to enable the sync pluses.
						if((count % frame_length) < sync_pulse_length && enablePwm == 1)
						{
							SYNC_OUT_A <: 1;
							sliceOn = 1;
						}
						else
						{
							SYNC_OUT_A <: 0;

							if(sliceOn)
							{
								sliceOn = 0;
								sliceCounter++;
							}

							if(sliceCounter == frame_count_per_petal)
							{
								sliceCounter = 0;
								ledOn = 0;
							}

						}
					}
					else
					{
						ledOn = 0;
						// Not in a petal
						//SYNC_OUT_B <: 0;
					}
				} // if

				*/


				encoder_previous = encoder;
				break;


			//Perform a second check after the encoder to perform any additional changes
				/*
				case curTimer when timerafter(lastTime+1E6) :> curTime:
						lastTime = curTime;
						rpm = 0;
				   break;
				*/

				//These handle the commands sent via the serial port
				case cmd.setPetalOffset(int offset):
						first_petal_start = offset;
					break;

				case cmd.setFrameLen(int len):
						frame_length = len;
						break;

				case cmd.setFrameCount(int count):
						frame_count_per_petal = count;
						counts_per_petal = (int)(((double)frame_length) * ((double)frame_count_per_petal) * 1);
						cpr_jump_per_petal = encoder_cpr / petal_count;
						break;

				case cmd.setPetalCount(int count):
						petal_count = count;
						counts_per_petal = (int)(((double)frame_length) * ((double)frame_count_per_petal) * 1);
						cpr_jump_per_petal = encoder_cpr / petal_count;
						break;

						/*
							case cmd.setPetalOffset(int offset):
								tempPetalStart = offset;

								resetNeeded = 1;
								break;

							case cmd.setFrameLen(int len):
								tempFrameLen = len;
								resetNeeded = 1;
								break;

							case cmd.setFrameCount(int count):
									tempCountPerPetal = count;
									resetNeeded = 1;
									break;

							case cmd.setPetalCount(int count):
									tempPetalCount = count;
									resetNeeded = 1;
									break;
						 */

				case cmd.enablePwm(int enable):
						//Immediately turn off PWM, but delay turning it back on
						if(enable == 0)
							enablePwm = 0;
						else
							resetNeeded = 1;

						tempEnPwm = enable;

						//firstRun = 1;
						//if(enablePwm == 0)
						break;

				//Flush double the size of the current slice count to fully clear any sequences still in the vialux
				case cmd.flushBuffer():
						//Flush out all the frames currently in the buffer... do at least a full rotation for each color LED currently on, just in case
						for(int i=0; i<frame_count_per_petal*3*getNumLedsOn(ledMode);i++)
						{
							SYNC_OUT_A <: 1;
							delay_milliseconds(1);
							SYNC_OUT_A <: 0;
							delay_milliseconds(1);
						}

						break;

				case cmd.getRPM() -> int return_val:
						return_val = (int) rpm;
						break;

				case cmd.ledMode(char mode):
					switch(mode)
					{
						case LED_OFF:
						case LED_RED:
						case LED_GREEN:
						case LED_BLUE:
							ledMode = 0;
							rgbState = 0;
							setRGB_Value(mode);
							break;

						case LED_PETAL_RGB:
							setRGB_Value(0);
							ledMode = LED_PETAL_UPDATE | LED_R_ON | LED_G_ON | LED_B_ON;
							rgbState = 0;
							break;

						case LED_PETAL_RG:
							setRGB_Value(0);
							ledMode = LED_PETAL_UPDATE | LED_R_ON | LED_G_ON;
							rgbState = 0;
							break;

						case LED_PETAL_RB:
							setRGB_Value(0);
							ledMode = LED_PETAL_UPDATE | LED_R_ON | LED_B_ON;
							rgbState = 0;
							break;

						case LED_PETAL_GB:
							setRGB_Value(0);
							ledMode = LED_PETAL_UPDATE | LED_G_ON | LED_B_ON;
							rgbState = 0;
							break;

						case LED_SLICE_RGB:
							setRGB_Value(0);
							ledMode = LED_SLICE_UPDATE | LED_R_ON | LED_G_ON | LED_B_ON;
							rgbState = 0;
							break;

						case LED_SLICE_RG:
							setRGB_Value(0);
							ledMode = LED_SLICE_UPDATE | LED_R_ON | LED_G_ON;
							rgbState = 0;
							break;

						case LED_SLICE_RB:
							setRGB_Value(0);
							ledMode = LED_SLICE_UPDATE | LED_R_ON | LED_B_ON;
							rgbState = 0;
							break;

						case LED_SLICE_GB:
							setRGB_Value(0);
							ledMode = LED_SLICE_UPDATE | LED_G_ON | LED_B_ON;
							rgbState = 0;
							break;

					}
						break;
			}

		}
	} // PulseTrainSynth()


void
Debug(server interface iStatus status)
{
	while(1)
	{
		select
		{
			case status.countsAtIndex(int counts):
			printintln(counts);
			break;

		} // select
	} // while
} // Debug()


int main()
{
	//printstrln("Hello World!");
	interface	iStatus status;
	interface commands cmd;

	par
	{
		VirtualCommPort(cmd);
		PulseTrainSynth(status, cmd);
		Debug(status);
	}

	return 0;
}

char* alias processCmd(char buffer[100], client interface commands cmd)
{

	char command[70];
	char returnMessage[100];
	int val1;

	command[0] = '\0';
	returnMessage[0] = '\0';
	sscanf(buffer, "%s %d", command, &val1);

	if(strcmp("PWM", command) == 0)
	{

		//printstr("PWM\n");

		if(val1 == 0)
		{
			cmd.enablePwm(0);
			sprintf(returnMessage, "PWM OFF\r");
		}
		else if(val1 == 1)
		{
			cmd.enablePwm(1);
			sprintf(returnMessage, "PWM ON\r");
		}
		else
		{
			sprintf(returnMessage, "ERROR PROCESSING 'PWM' COMMAND\r");
		}
	}

	else if(strcmp("OFFSET", command) == 0)
	{
		cmd.setPetalOffset(val1);
		sprintf(returnMessage, "OFFSET SET\r");
	}

	//Adjust the number of petals

		else if(strcmp("PETAL#", command) == 0)
		{
			cmd.setPetalCount(val1);
			sprintf(returnMessage, "PETAL# SET\r");
		}

		//Adjust the frame_length

		else if(strcmp("FRAMELEN", command) == 0)
		{
			cmd.setFrameLen(val1);
			sprintf(returnMessage, "FRAMELEN SET\r");
		}

		//Adjust the frame_count_per_petal

		else if(strcmp("FRAMECOUNT", command) == 0)
		{
			cmd.setFrameCount(val1);
			sprintf(returnMessage, "FRAMECOUNT SET\r");
		}

		else if(strcmp("TILT", command) == 0)
		{
			//cmd.setFrameCount(val1);
			sprintf(returnMessage, "TILT %f\r", curTilt);
		}
		else if(strcmp("FLUSH", command) == 0)
		{
			cmd.flushBuffer();
			sprintf(returnMessage, "BUFFER FLUSHED\r");
		}
		else if(strcmp("LEDMODE", command) == 0)
		{
			cmd.ledMode((unsigned char)val1);
			sprintf(returnMessage, "LED MODE CHANAGED\r");
		}

		else if(strcmp("RPM", command) == 0)
		{
			sprintf(returnMessage, "RPM %d\r", cmd.getRPM());
		}
		else
		{
			sprintf(returnMessage, "UNKNOWN COMMAND:\r%s\r", command);
		}

		return returnMessage;
}



void spiWriteReg(const unsigned char regAddr, const unsigned char* alias regData)
{
	int t = 100;
	int x;


  unsigned char SPICount;                               // Counter used to clock out the data

  unsigned char SPIData;                                // Define a data structure for the SPI data.

  PIN_VCP_CS <: 1;                                           // Make sure we start with /CS high
  PIN_VCP_CLK <: 0;                                           // and CK low
  delay_microseconds(t);

  SPIData = regAddr;                                    // Preload the data to be sent with Address
  PIN_VCP_CS <: 0;                                           // Set /CS low to start the SPI cycle 25nS
  delay_microseconds(t);
                                                        // Although SPIData could be implemented as an "int", resulting in one
                                                        // loop, the routines run faster when two loops are implemented with
                                                        // SPIData implemented as two "char"s.

  for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Address byte
  {
    if (SPIData & 0x80)                                 // Check for a 1
    	PIN_VCP_MIOSI <: 1;                                     // and set the MOSI line appropriately
    else
    	PIN_VCP_MIOSI <: 0;
    PIN_VCP_CLK <: 1;                                         // Toggle the clock line
    delay_microseconds(t);
    PIN_VCP_CLK <: 0;
    delay_microseconds(t);

    SPIData <<= 1;                                      // Rotate to get the next bit
  }


	for(x = 0; x < 100; x++)
	{
		if(regData[x] == '\0')
			break;
			// and loop back to send the next bit
																													// Repeat for the Data byte
		SPIData = regData[x];                                    // Preload the data to be sent with Data
		for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock out the Data
		{
			if (SPIData & 0x80)
				PIN_VCP_MIOSI <: 1;
			else
				PIN_VCP_MIOSI <: 0;
			PIN_VCP_CLK <: 1;
			delay_microseconds(t);
			PIN_VCP_CLK <: 0;
			delay_microseconds(t);
			SPIData <<= 1;
		}
	}

  PIN_VCP_CS <: 1;
  PIN_VCP_MIOSI <: 0;
}


unsigned int adcSpiReadReg()
{
	int t = 10;
	int sampleT = 10;

	unsigned char x;

	unsigned char dataBit;
	 unsigned char SPICount;                               // Counter used to clock out the data

	  unsigned int SPIData;


	  PIN_ADC_CS <: 1;                                           // Make sure we start with /CS high
	  delay_microseconds(t);
	  PIN_ADC_CS <: 0;
	  PIN_VCP_CLK <: 0;                                           // and CK low
	  delay_microseconds(sampleT);
	  PIN_VCP_CLK <: 1;
	  delay_microseconds(sampleT);
	  PIN_VCP_CLK <: 0;                                           // and CK low
		delay_microseconds(sampleT);
		PIN_VCP_CLK <: 1;
		delay_microseconds(sampleT);

		//Null bit
		PIN_VCP_CLK <: 0;
		delay_microseconds(t);
		//PIN_VCP_MISO :> dataBit;
		PIN_VCP_CLK <: 1;
		delay_microseconds(t);

		//SPIData |= dataBit;

		SPIData = 0;
		for (SPICount = 0; SPICount < 12; SPICount++)          // Prepare to clock in the data to be fread
		{
			SPIData <<=1;
			PIN_VCP_CLK <: 0;
			delay_microseconds(t);
			PIN_VCP_CLK <: 1;
			PIN_VCP_MISO :> dataBit;
			SPIData |= dataBit;
			delay_microseconds(t);
		}

	  //PIN_VCP_CLK <: 1;
	  //delay_microseconds(t);
    PIN_VCP_CLK <: 0;
    delay_microseconds(t);

	  PIN_VCP_MIOSI :> x;
	  PIN_VCP_MISO :> x;


	  PIN_ADC_CS <: 1;                                           // Raise CS
	  delay_microseconds(t);

	  return SPIData;                      // Finally return the read data
}


unsigned int spiReadReg (const unsigned char regAddr)
{
	int t = 100;

	unsigned char x;

	unsigned char dataBit;
	 unsigned char SPICount;                               // Counter used to clock out the data

	  unsigned int SPIData;

	  PIN_VCP_CS <: 1;                                           // Make sure we start with /CS high
	  PIN_VCP_CLK <: 0;                                           // and CK low
	  delay_microseconds(t);

	  SPIData = regAddr;                                    // Preload the data to be sent with Address & Data

	  PIN_VCP_CS <: 0;                                           // Set /CS low to start the SPI cycle
	  delay_microseconds(t);

	  SPIData <<= 1;
	  for (SPICount = 0; SPICount < 7; SPICount++)          // Prepare to clock out the Address & Data
	  {
	    PIN_VCP_CLK <: 1;

	    if (SPIData & 0x80)
				PIN_VCP_MIOSI <: 1;
			else
				PIN_VCP_MIOSI <: 0;

	    delay_microseconds(t);

	    PIN_VCP_CLK <: 0;
	    delay_microseconds(t);

	    SPIData <<= 1;
	  }                                                     // and loop back to send the next bit
	  //PIN_VCP_MIOSI <: 0;                                         // Reset the MOSI data line

	  PIN_VCP_CLK <: 1;
	  delay_microseconds(t);
    PIN_VCP_CLK <: 0;
    delay_microseconds(t);

	  PIN_VCP_MIOSI :> x;
	  PIN_VCP_MISO :> x;

	  SPIData = 0;
	  if(!x)
	  {
			for (SPICount = 0; SPICount < 8; SPICount++)          // Prepare to clock in the data to be fread
			{
				SPIData <<=1;
				PIN_VCP_CLK <: 1;
				delay_microseconds(t);
				PIN_VCP_CLK <: 0;
				PIN_VCP_MIOSI :> dataBit;
				delay_microseconds(t);
				SPIData |= dataBit;
			}
	  }

	  PIN_VCP_CS <: 1;                                           // Raise CS
	  delay_microseconds(t);

	  return ((unsigned char)SPIData);                      // Finally return the read data
}



void VirtualCommPort(client interface commands cmd)
{
	char* retMess;
	char adcMess[100];
	int adcQuery = 0; //Current sample number
	char adcWait = 0; //Used to space out samples


	char buffer[100];
	int ind=0;
	unsigned int adcVal, newAdcVal;
	float tiltAng = 0;

	curTilt = 0;

	int fullAdcVal = 0; //Collects all values as a summation prior to averaging
	int maxAdcVal = 0; //Used to discard the max
	int minAdcVal = 0; //and min values from the sample
	int curSample = 0;
	int numSamples = 8;
	double sampleMult = 360.0f/(2048.0f * numSamples);

	//int x;
	//char x;
	char y;
	//int d;
		//int t = 100;
		//uint8_t temp;
		//printstrln("");
		//PIN_VCP_CS <: 1;
		//PIN_VCP_CLK <: 0;
		//PIN_VCP_MIOSI :> x;


		printstr("ENCODER COMMAND PROMPT ONLINE\n");

		//spiWriteReg(0, "ENCODER COMMAND PROMPT ONLINE\n");

		while(1)
		{
			y = spiReadReg(1);
			if(y != 0)
			{
				buffer[ind++] = y;

				//printchar(y);

				//Process the command
				if(y == '\r' || ind == 100)
				{
					if(ind != 100)
					{
						retMess = processCmd(buffer, cmd);

						if(retMess[0] != '\0')
						{
								spiWriteReg(0, retMess);
						}
					}

					//Clear buffer
					for(ind=0; ind < 100; ind++)
					{
						buffer[ind] = 0;
					}

					//reset index
					ind = 0;
				}
			}

			//Sampling concept: take 10 samples and discard the max and min, then average out the other 8
			//The adcWait is a crude way of spacing out when we sample the adc
			if(++adcWait > 20)
			{
				//Take another sample
				curSample = adcSpiReadReg();

				//Set the max and min values after read (these will be reset to this read if the sample is 1 below)
				if(curSample > maxAdcVal)
					maxAdcVal = curSample;

				if(curSample < minAdcVal)
					minAdcVal = curSample;


				if(++adcQuery >= numSamples + 2)
				{
					//curTilt = adcSpiReadReg();

					fullAdcVal += curSample;
					//fullAdcVal = (fullAdcVal / numSamples);

					curTilt = fullAdcVal - maxAdcVal - minAdcVal; //remove the max and min values from the samples
					curTilt = curTilt/ numSamples; //average out the rest of the samples
					curTilt = 2048.0f - curTilt; //the middle of tilt sensor should be roughly zero degrees, but is 2048 ADC counts, so convert
					curTilt = ((float)(curTilt*360))/4096.0f; //Now convert from ADC counts to degrees (12 bit adc)

					fullAdcVal = 0;
					adcQuery = 0;
				}
				//First sample - as driven by the ++adcQuery == 1 on first run through or after reset
				else if(adcQuery == 1)
				{
					fullAdcVal = curSample;
					maxAdcVal = curSample;
					minAdcVal = curSample;
				}
				else
				{
					fullAdcVal += curSample;
				}

				//Reset the wait counter
				adcWait = 0;
			}

			/*
			{
				newAdcVal = adcSpiReadReg();
				if(newAdcVal != adcVal)
				{
					tiltAng = (1024 - (float)newAdcVal)*360/2048;

					//sprintf(adcMess, "ADC: %d\n", adcVal);
					sprintf(adcMess, "tilt: %f\n", tiltAng);
					//spiWriteReg(0, adcMess);
					adcQuery = 0;
				}

				newAdcVal = adcVal;
			}
			*/
		}



		//PIN_VCP_CS <: 1; delay_microseconds(t);

/*

		while(1)
		{
			//PIN_VCP_MISO :> d;
			//if(d) {
			//printstrln("Data Ready!");

			PIN_VCP_CS <: 0; delay_microseconds(t);

			// Send Bit 0
			PIN_VCP_CLK <: 1;
			PIN_VCP_MIOSI <: 0; delay_microseconds(t);
			PIN_VCP_CLK <: 0; delay_microseconds(t);
			// Send Bit 1
			PIN_VCP_CLK <: 1;
			PIN_VCP_MIOSI <: 0; delay_microseconds(t);
			PIN_VCP_CLK <: 0; delay_microseconds(t);
			// Send Bit 2
			PIN_VCP_CLK <: 1;
			PIN_VCP_MIOSI <: 0; delay_microseconds(t);
			PIN_VCP_CLK <: 0; delay_microseconds(t);
			// Send Bit 3
			PIN_VCP_CLK <: 1;
			PIN_VCP_MIOSI <: 0; delay_microseconds(t);
			PIN_VCP_CLK <: 0; delay_microseconds(t);
			// Send Bit 4
			PIN_VCP_CLK <: 1;
			PIN_VCP_MIOSI <: 0; delay_microseconds(t);
			PIN_VCP_CLK <: 0; delay_microseconds(t);
			// Send Bit 5
			PIN_VCP_CLK <: 1;
			PIN_VCP_MIOSI <: 0; delay_microseconds(t);
			PIN_VCP_CLK <: 0; delay_microseconds(t);
			// Send Bit 6, CMD[0]
			PIN_VCP_CLK <: 1;
			PIN_VCP_MIOSI <: 1; delay_microseconds(t);
			PIN_VCP_CLK <: 0; delay_microseconds(t);
			// Turn around port
			PIN_VCP_CLK <: 1; delay_microseconds(t);
			PIN_VCP_CLK <: 0; delay_microseconds(t);
			PIN_VCP_MIOSI :> x;
			PIN_VCP_MISO :> x;

			if(!x)
			{
				//printuintln(x);

				y = 0x00;

				// Read Bit
				for(int i = 0;i < 8; i++)
				{
					y <<= 1;
					PIN_VCP_CLK <: 1; delay_microseconds(t);
					PIN_VCP_CLK <: 0;
					PIN_VCP_MIOSI :> x; delay_microseconds(t);
					y |= x;
				}
				//printchar(y);
				buffer[ind++] = y;

				//Process the command
				if(y == '\r' || ind == 100)
				{
					if(ind != 100)
					processCmd(buffer, cmd);

					//Clear buffer
					for(ind=0; ind < 100; ind++)
					{
						buffer[ind] = 0;
					}

					//reset index
					ind = 0;
				}

				//}
				//printuintln(y);

				//delay_milliseconds(100);
			} // if

			PIN_VCP_CS <: 1; delay_microseconds(t);
		} // while

*/

	} //

