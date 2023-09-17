// Main parameters file for EGG V3.

#define PING_PERIOD_MIN 1         // period to transmit during sleep in minutes, integer, default=5, for sleep mode minutes
#define PING_PERIOD_SEC 1
#define PING_PERIOD_MSEC 950 // This has to be below 1000

#define N_RECORDING_MODE_2 8 // Samples sent in between each mode 2 sleep
#define TIME_BETWEEN_MS_MODE_2 2000 // Sleep time in ms between each sample period in mode 2 (down_sampling)


#define N_RECORDING_MODE_3 8 // Samples sent in between each mode 2 sleep
#define TIME_BETWEEN_S_MODE_3 1 // Sleep time in SECONDS between each sample period in mode 2 (down_sampling)
#define TIME_BETWEEN_MS_MODE_3 840 // Sleep time in SECONDS between each sample period in mode 2 (down_sampling)

//RECORDING MODE
// 0 - NO ADC
// 1 - ADC recording samples/sec
// 2 - ADC recording sample with ADC sleep sampling
// 3 - Same as mode 2 BUT SLEEP IS IN SECONDS

//Sleep
//0  - no sleep
//1 - sleep in minutes
//2 - sleep in seconds
//3 - sleep in milliseconds: NOTE SLEEP_TIME_ARRAY should be <1000 in this case. Otherwise use mode 2


#define INIT_SLEEP 1300000 //initial time to sleep

#define FIRST_INDEX_NO_REPEAT 0 //setting to not repeat first index on repeat
// NUMBER_OF_SAMPLES_ARRAY and SLEEP_TIME_MIN_ARRAY arrays should be treated as pairs for the same index,
// indicating FIRST the number of samples to be recorded THEN the time in minutes to sleep before the next samples are taken.
// Note that all values should be greater than PING_PERIOD_MIN

const uint16_t RECORDING_MODE_ARRAY[]       =   {   0, 0, 0, 0, 0   };
const uint64_t NUMBER_OF_SAMPLES_ARRAY[]    =   {   0, 0, 0, 0, 0   };
const uint16_t SLEEP_MODE_ARRAY[]           =   {   0, 0, 0, 0, 0   };
const uint64_t SLEEP_TIME_ARRAY[]           =   {   0, 0, 0, 0, 0   }; //If msec sleep is used, sleep time preferentially is a multiple of PING_PERIOD_MSEC

// Set Rate to 62.5 samples/second
#define CLOCK_ADC_SAMPLE CLOCK_OSR_16384
#define GAIN1_SET GAIN1_PGAGAIN3_4 | GAIN1_PGAGAIN2_4 | GAIN1_PGAGAIN1_4 | GAIN1_PGAGAIN0_4
#define GAIN2_SET GAIN2_PGAGAIN7_4 | GAIN2_PGAGAIN6_4 | GAIN2_PGAGAIN5_4 | GAIN2_PGAGAIN4_4


// Gain Configurations Full Scale Range (FSR) is determined by end number
// _1 +- 1.2V
// _2 +- 600mV
// _4 +- 300mV
// _8 +- 150mV
// _16 +- 75mV
// _32 +- 37.5mVs
// _64 +- 18.75mV
// _128 +- 9.375mV

//for testing -- TO BE COMMENTED OUT DURING FLASING FOR TERMINAL
//const uint64_t NUMBER_OF_SAMPLES_ARRAY[]    = {10,    20, 25, 25, 100};
//const uint64_t SLEEP_TIME_MIN_ARRAY[]       = {2,     10,   1,   0,   5};

//112500 = 30 min recording
