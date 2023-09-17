// Main parameters file for EGG V3.

#define PING_PERIOD_MIN 1
#define PING_PERIOD_SEC 1          // period to transmit during sleep in minutes, integer, default=5

// NUMBER_OF_SAMPLES_ARRAY and SLEEP_TIME_MIN_ARRAY arrays should be treated as pairs for the same index,
// indicating FIRST the number of samples to be recorded THEN the time in minutes to sleep before the next samples are taken.
// Note that all values should be greater than PING_PERIOD_MIN (or equal??)

//                                            SETUP     FIRST   SECOND  THIRD   FOURTH
const uint64_t NUMBER_OF_SAMPLES_ARRAY[]    =   {4,     4,      4,      4,      4   };
const uint64_t SLEEP_IN_SEC_ARRAY[]         =   {1,     1,      1,      1,      1   };





//for testing -- TO BE COMMENTED OUT DURING FLASING FOR TERMINAL
//const uint64_t NUMBER_OF_SAMPLES_ARRAY[]    = {10,    20, 25, 25, 100};
//const uint64_t SLEEP_TIME_MIN_ARRAY[]       = {2,     10,   1,   0,   5};

//112500 = 30 min recording
