///////////////////////////////////////////////////////////////////////////////////////////////
// SINEIO
///////////////////////////////////////////////////////////////////////////////////////////////

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <time.h>
#include <fstream>
#include <conio.h>              // <-- added: for _kbhit()
#include <algorithm>            // <-- added: for std::min/std::max
#include "IIRdf1filt.h"
#include "../myWin826.h"
#include "../826api.h"
#include "../RealTime.h"

#define SAMPLE_RATE  1000    // Hz
#define MAX_LOOP_TIME  10    //seconds
#define RECORD_LENGTH (MAX_LOOP_TIME*SAMPLE_RATE)


// IO card configuration constants
#define IO_BOARD_NUM  0						  // no change
#define ADC_SLOT      0						  // no change

#define DAC_ZERO_OUTPUT 0x8000
#define DAC_CONFIG_GAIN S826_DAC_SPAN_10_10   // -10 to 10 spans 20 volts so...
#define DAC_VRANGE    20.0                    // This MUST correspond to the DAC_SPAN just above
#define DAC_CNT_RANGE 0xFFFF				  // 16-bit DAC
#define DAC_OFFSET_COUNTS 0x8000              // 32768 offset for a signed desired output mapped to unsigned DAC write.

#define DAC_CHANNEL   0						  // SET ME! CHECK YOUR SETUP: DAC channel output

#define ADC_CHANNEL  1                        // SET ME! CHECK YOUR SETUP: analog input channel to track


#define ADC_CNT_RANGE 0xFFFF				  // 2^16= 0xFFFF (16 bit converter)
#define ADC_GAIN     S826_ADC_GAIN_1          // -10 to 10 option
#define ADC_VRANGE   20                       // this must correspond to gain setting

#define ADC_ENABLE 1


// my matlab design for a high-pass Butterworth
const double a[] = { 1.000000000000000, -1.991114292201654,  0.991153595868935 };
const double b[] = { 0.995566972017647, -1.991133944035295,  0.995566972017647 };

int main()
{
	int  errcode = S826_ERR_OK;
	int  boardflags = S826_SystemOpen();        // open 826 driver and find all 826 boards
	int  ncount = 0;
	uint dacout;

	double voltageout = 0;

	double Amp = 0;
	double Frequency = 0;

	// encoder interface
	uint   start_count;
	uint   counts;
	int    rawcounts[RECORD_LENGTH];
	double filtcounts[RECORD_LENGTH];

	// real-time filter
	int  N = sizeof(a) / sizeof(double); // number of a's and b's. filter order plus 1.
	IIRdf1filt filter(N, a, b);          // instantiate and initialize the filter

	// file output
	int i;
	char outfileName[30];

	// instantiate our "real time" object that will pace our loop
	RealTime realTime(SAMPLE_RATE);


	// Configure data acquisition interfaces and start them running.
	X826(S826_AdcSlotConfigWrite(IO_BOARD_NUM, ADC_SLOT, ADC_CHANNEL, 0, ADC_GAIN));  // program adc timeslot attributes: slot, chan, 0us settling time. For -5V-5V use: "S826_ADC_GAIN_2"
	X826(S826_AdcSlotlistWrite(IO_BOARD_NUM, 1 << ADC_SLOT, S826_BITWRITE));  // enable adc timeslot; disable all other slots
	X826(S826_AdcEnableWrite(IO_BOARD_NUM, ADC_ENABLE));  // enable adc conversions
	X826(S826_DacRangeWrite(IO_BOARD_NUM, DAC_CHANNEL, DAC_CONFIG_GAIN, 0));  // program dac output range: -10V to +10V

	// Initialize Motor Shaft Encoder Interface
	S826_CounterModeWrite(0, 0, S826_CM_K_QUADX1); // Configure counter0 as incremental encoder interface.
	S826_CounterStateWrite(0, 0, 1);               // Start tracking encoder position.
	S826_CounterRead(0, 0, &start_count);          // Read initial encoder counts

	// Added
	printf("Enter sine Frequency in Hz (e.g., 5): ");
	scanf("%lf", &Frequency);

	printf("Enter sine Amplitude in Volts PEAK (e.g., 1.0): ");
	scanf("%lf", &Amp);

	// safety clamp on user input (prevents overflow of DAC output range and wraparound of negative voltages)
	if (Amp > 10.0) Amp = 10.0;
	if (Amp < 0.0)  Amp = 0.0;  // negative amplitude is redundant; phase sign belongs in sin()

	// commence pseudo-real-time loop.
	realTime.Start();


	while (!_kbhit() && ncount < RECORD_LENGTH)
	{
		// ------------------------------------------------------------
		// generate the commanded sine to drive the motor
		// v[n] = Amp * sin(2*pi*Frequency*t),  t = n / Fs
		// ------------------------------------------------------------
		double t = (double)ncount / (double)SAMPLE_RATE;     // seconds
		voltageout = Amp * sin(2.0 * M_PI * Frequency * t);  // volts

		// scale the desired voltage to the DAC integer output.
		dacout = (uint)(voltageout * (DAC_CNT_RANGE / DAC_VRANGE) + DAC_OFFSET_COUNTS);

		// optional clamp to valid 16-bit range (prevents wraparound if you ever exceed range)
		if (dacout > 0xFFFF) dacout = 0xFFFF;

		// output to the DAC
		X826(S826_DacDataWrite(IO_BOARD_NUM, DAC_CHANNEL, dacout, 0));

		S826_CounterRead(0, 0, &counts);          // Read current encoder counts.

		counts = counts - start_count;             // Subtracting the initial counts so we begin from 0

		rawcounts[ncount] = (int)counts;          // raw encoder signal (contains drift + sinusoid)
		filtcounts[ncount] = filter.FilterSample((float)rawcounts[ncount]); // high-pass removes drift

		realTime.Sleep();
		ncount++;
	}

	realTime.Stop(ncount); // stop real-time loop

	X826(S826_DacDataWrite(IO_BOARD_NUM, DAC_CHANNEL, DAC_ZERO_OUTPUT, 0));   // put to zero at the end
	S826_SystemClose();


	// file output
	printf("\nType in a file name (freq_amp like 50_10 for 5Hz, 1V is a good idea):\n");
	scanf("%s", outfileName);

	std::ofstream outfile(strcat(outfileName, ".txt")); // our test file to filter

	// Write raw data to first column of output file
	for (i = 0; i < ncount; i++)
	{
		// Must match required format: Raw,Filtered
		outfile << rawcounts[i] << "," << filtcounts[i] << "\n";
	}

	outfile.close();

	return 0;
}
