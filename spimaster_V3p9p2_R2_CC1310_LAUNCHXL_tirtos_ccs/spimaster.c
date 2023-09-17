/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//ag test

/*
 *  ======== spimaster.c ========
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>
#include <ti/drivers/PWM.h>

#include <ti/drivers/rf/RF.h>
#include <ti/drivers/Power.h>

#include "ads131m0x.h"

#include "EGG_Experiment_Params.h"

/* Example/Board Header files */
#include "Board.h"
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include "smartrf_settings/smartrf_settings.h"

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      20  //remember to update for multi packets!

#define DEBUG_GPIO_1 1
#define ADC_CLOCK_NOT_EN 5

#define SEND_DEBUG_INFO (0)
#define SEND_DATA (1)


#define TRUE 1

/***** Prototypes *****/

/***** Variable declarations for RF*****/
RF_Object rfObject;
RF_Handle rfHandle;
uint8_t packet[PAYLOAD_LENGTH+2];
//static uint8_t packet[PAYLOAD_LENGTH+2];
static uint16_t seqNumber;
void send_databuffer(const void* buffer,int buffer_size);
int send_spi_to_adc(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t words_to_send, const bool send_response);
int init_ads131m08();
int transmit_adc_samples(const uint64_t NUM_SAMPLES);
int goto_sleep(const uint64_t standby_sleep_duration, const uint64_t standby_intervals, const bool transmit_flag);
int goto_sleep_min(const uint64_t SLEEP_IN_MIN);
int goto_sleep_sec(const uint64_t SLEEP_IN_SEC);
int migut_open_spi(void);

#define THREADSTACKSIZE (1024)

#define SPI_MSG_LENGTH  (10)
//#define MASTER_MSG      ("Hello from master, msg#: ")

#define MAX_LOOP        (10)

//static Display_Handle display;

int16_t masterRxBuffer[SPI_MSG_LENGTH];
int16_t masterTxBuffer[SPI_MSG_LENGTH];
SPI_Transaction transaction;
SPI_Handle      masterSpi;
SPI_Params      spiParams;
uint8_t data_ready = 0;

// PWM Globals
PWM_Handle pwm;
PWM_Params pwmParams;

/* Semaphore to block master until slave is ready for transfer */
sem_t masterSem;

/*
 *  ======== slaveReadyFxn ========
 *  Callback function for the GPIO interrupt on Board_SPI_SLAVE_READY.
 */
void slaveReadyFxn(uint_least8_t index)
{
    //sem_post(&masterSem);

    data_ready = 1;
    //GPIO_toggle(Board_GPIO_LED0);


}

void send_databuffer(const void* buffer, int buffer_size)
{

    RF_cmdPropTx.pktLen = buffer_size+2;

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    //uint8_t j;
    //for (j = 0; j < buffer_size/PAYLOAD_LENGTH; ++j)
    //{
        /* Create packet with incrementing sequence number and random payload */
        uint8_t *a=(((uint8_t*)buffer)/*+(PAYLOAD_LENGTH)*j*/);
        packet[0] = (uint8_t)(seqNumber >> 8);
        packet[1] = (uint8_t)(seqNumber++);

        uint8_t i;
        for (i = 2; i < buffer_size+2 /*PAYLOAD_LENGTH+2*/; i++)
        {
            packet[i]=a[i-2];
            //packet[i] = rand();
//            packet[i] = ((uint8_t*)buffer)+(PAYLOAD_LENGTH)*j+i /* check pointer math */
        }

        /* Send packet */
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
                                                   RF_PriorityNormal, NULL, 0);

        switch(terminationReason)
        {
            case RF_EventLastCmdDone:
                // A stand-alone radio operation command or the last radio
                // operation command in a chain finished.
                break;
            case RF_EventCmdCancelled:
                // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
                break;
            case RF_EventCmdAborted:
                // Abrupt command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            case RF_EventCmdStopped:
                // Graceful command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            default:
                // Uncaught error event
                while(1);
        }

        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
        switch(cmdStatus)
        {
            case PROP_DONE_OK:
                // Packet transmitted successfully
                break;
            case PROP_DONE_STOPPED:
                // received CMD_STOP while transmitting packet and finished
                // transmitting packet
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT while transmitting packet
                break;
            case PROP_ERROR_PAR:
                // Observed illegal parameter
                break;
            case PROP_ERROR_NO_SETUP:
                // Command sent without setting up the radio in a supported
                // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                break;
            case PROP_ERROR_NO_FS:
                // Command sent without the synthesizer being programmed
                break;
            case PROP_ERROR_TXUNF:
                // TX underflow observed during operation
                break;
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
                while(1);
        }

        /* Power down the radio */
        RF_yield(rfHandle);
    //}
}

int send_spi_to_adc(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t words_to_send, const bool send_response) {
    int return_value = 0;

    transaction.count = words_to_send; //9; testing changing to 10
    transaction.txBuf = (void *) tx_buffer;
    transaction.rxBuf = (void *) rx_buffer;

    //GPIO_write(DEBUG_GPIO_1, 1);
    //GPIO_write(DEBUG_GPIO_1, 0);
    bool spitransferOK;
    spitransferOK = SPI_transfer(masterSpi, &transaction);
    if (spitransferOK) {
        //do nothing
        return_value = 1;
    }
    else {
        return_value = 0;
    }

#if (SEND_DEBUG_INFO)
    send_databuffer(tx_buffer,sizeof(tx_buffer)); //TODO check if safe to change to words_to_send*2, was 20 if not working
#endif

    if (send_response) {
        send_databuffer(rx_buffer,sizeof(rx_buffer));
    }

    return return_value;

}


int migut_open_spi() {
    /* Open SPI as master (default) */
    SPI_Params_init(&spiParams);            //spiParams is a global (TODO change eventually)
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.bitRate = 4000000;
    spiParams.dataSize = 16;    //While the ADS131M0x default is 24bit packets, the CC1310 is limited to 16.
    spiParams.transferMode = SPI_MODE_BLOCKING;
    //GPIO_write(Board_SPI_MASTER_READY, 0); //AG CHECK TODO check

    masterSpi = SPI_open(0, &spiParams);    //masterSPI is a global (TODO change eventually)
    if (masterSpi == NULL) {
        //Display_printf(display, 0, 0, "Error initializing master SPI\n");
        uint8_t error_buffer[1] = {0x99};
        send_databuffer(error_buffer,sizeof(error_buffer));
        return 0;
    }
    else {
        //Display_printf(display, 0, 0, "Master SPI initialized\n");

    }

    return 1;

}

int init_ads131m08() {
    //INIT_ADS131M08 takes in a refference to the SPI Params and sets them to work with the ADS131M08 ADC,
    // and configures the registers for proper operation.

    /* Open SPI as master (default) */
    SPI_Params_init(&spiParams);            //spiParams is a global (TODO change eventually)
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.bitRate = 4000000;
    spiParams.dataSize = 16;    //While the ADS131M0x default is 24bit packets, the CC1310 is limited to 16.
    spiParams.transferMode = SPI_MODE_BLOCKING;
    //GPIO_write(Board_SPI_MASTER_READY, 0); //AG CHECK TODO check

    masterSpi = SPI_open(0, &spiParams);    //masterSPI is a global (TODO change eventually)
    if (masterSpi == NULL) {
        //Display_printf(display, 0, 0, "Error initializing master SPI\n");
        uint8_t error_buffer[1] = {0x99};
        send_databuffer(error_buffer,sizeof(error_buffer));
        return 0;
    }
    else {
        //Display_printf(display, 0, 0, "Master SPI initialized\n");
    }

    //uint16_t init24bit[10] = {0x6104, 0x0000, 0x1000, 0xff1c, 0x0033, 0x3300, 0x3333, 0x0006, 0x0000};
    uint16_t init24bitRx[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

    /* RESET */
    uint16_t reset24bit[10] = {0x0011, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
    send_spi_to_adc(reset24bit, init24bitRx, 10, SEND_DEBUG_INFO);

    /* Set word length to 16bit */
    uint16_t sixteenbit24bit[3] = {0x6100, 0x0004, 0x1000};     // {0x6100, 0x0004, 0x1000};
    uint16_t sixteenbit24bitRx[3] = {0x6100, 0x0004, 0x1000};   // 0011 0000 0687
    send_spi_to_adc(sixteenbit24bit, sixteenbit24bitRx, 3, SEND_DEBUG_INFO);

    /* Config CLOCK Register */
    // set VLP Mode (1.8mA ON), disable crystal, and OSR to 128 (8kSPS)
    //uint16_t lpm16bit[2] = {0x6180, 0xff80}; // CLOCK_CH7_EN_ENABLED | CLOCK_CH6_EN_ENABLED | CLOCK_CH5_EN_ENABLED | CLOCK_CH4_EN_ENABLED | CLOCK_CH3_EN_ENABLED | CLOCK_CH2_EN_ENABLED | CLOCK_CH1_EN_ENABLED | CLOCK_CH0_EN_ENABLED | CLOCK_OSR_128 | CLOCK_PWR_VLP
    uint16_t lpm16bit[2] = {0x6180, CLOCK_CH7_EN_ENABLED
                                    | CLOCK_CH6_EN_ENABLED
                                    | CLOCK_CH5_EN_ENABLED
                                    | CLOCK_CH4_EN_ENABLED
                                    | CLOCK_CH3_EN_ENABLED
                                    | CLOCK_CH2_EN_ENABLED
                                    | CLOCK_CH1_EN_ENABLED
                                    | CLOCK_CH0_EN_ENABLED
                                    | CLOCK_ADC_SAMPLE //128==8kSPS, 16384 == 62.5SPS
                                    | CLOCK_PWR_VLP};

    uint16_t lpm16bitRx[2] = {0x0000, 0x0000};
    send_spi_to_adc(lpm16bit, lpm16bitRx, 2, SEND_DEBUG_INFO);


    /* Config GAIN1 Register */
    // Set gain to 2x
    uint16_t gain1_16bit[2] = {0x6200, GAIN1_SET};
    uint16_t gain1_16bitRx[2] = {0x0000, 0x0000};
    send_spi_to_adc(gain1_16bit, gain1_16bitRx, 2, SEND_DEBUG_INFO);

    /* Config GAIN1 Register */
    // Set gain to 2x
    uint16_t gain2_16bit[2] = {0x6280, GAIN2_SET};
    uint16_t gain2_16bitRx[2] = {0x0000, 0x0000};
    send_spi_to_adc(gain2_16bit, gain2_16bitRx, 2, SEND_DEBUG_INFO);

    /*  Enable GPIO 3 (index 3 in gpioPinCongfigs in LAUNCHXL.c) for nDRDY Interupt */
    GPIO_setConfig(3, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);//GPIO_CFG_IN_PU
    GPIO_setCallback(3, slaveReadyFxn);
    GPIO_enableInt(3);

    return 1;

}

int transmit_adc_samples(const uint64_t NUM_SAMPLES) {
    //GET_ADC_SAMPLES reads NUM_SAMPLES from the ADC and immediatly

    uint64_t sample_counter = 0;
    while (sample_counter<NUM_SAMPLES) {
        if (data_ready == 1) {
            uint16_t null_command[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
            uint16_t mdata_buffer[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
            send_spi_to_adc(null_command, mdata_buffer, 10, SEND_DATA);        // new data saved in mdata_buffer and sent by RF
            sample_counter = sample_counter+1;
            data_ready = 0;
        }
    }

    return 1;

}

#define ADS131_PACKET_SIZE 10
#define NUM_SAMPLES_PER_PACKET 1
int transmit_multiple_adc_samples(const uint64_t NUM_SAMPLES) {
    //GET_ADC_SAMPLES reads NUM_SAMPLES from the ADC and immediatly

    // USE DEFINITION
    //const uint8_t NUM_SAMPLES_PER_PACKET = 3;   //Number of packets to save before sending


    uint64_t sample_counter = 0;

    while (sample_counter<NUM_SAMPLES) {
        // reset data buffer so not transmit old data
        uint16_t mdata_buffer[ADS131_PACKET_SIZE*NUM_SAMPLES_PER_PACKET] = {0x0000};    //buffer data is saved to
        uint16_t null_command[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};   //buffer to send over SPI; the NULL (sample) command

        // sample NUM_SAMPLES_PER_PACKET sequentially
        uint8_t inter_transmit_sample_counter = 0;  // counter for when to transmit
        while (inter_transmit_sample_counter < NUM_SAMPLES_PER_PACKET) {
            if (data_ready == 1) {
                send_spi_to_adc(null_command, mdata_buffer+inter_transmit_sample_counter*ADS131_PACKET_SIZE, ADS131_PACKET_SIZE, !SEND_DATA);        // new data saved in mdata_buffer and sent by RF
                sample_counter = sample_counter+1;
                inter_transmit_sample_counter=inter_transmit_sample_counter+1;
                data_ready = 0; //ensure last
            }
        }

        //transmit buffer
        send_databuffer(mdata_buffer,sizeof(mdata_buffer));


    }

    return 1;

}

int transmit_adc_samples_secs(const uint64_t NUM_SAMPLES) {

    uint64_t sample_counter = 0;

    while (sample_counter<NUM_SAMPLES) {
        // reset data buffer so not transmit old data
        uint16_t mdata_buffer[ADS131_PACKET_SIZE*NUM_SAMPLES_PER_PACKET] = {0x0000};    //buffer data is saved to
        uint16_t null_command[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};   //buffer to send over SPI; the NULL (sample) command

        // sample NUM_SAMPLES_PER_PACKET sequentially
        uint8_t inter_transmit_sample_counter = 0;  // counter for when to transmit
        while (inter_transmit_sample_counter < NUM_SAMPLES_PER_PACKET) {
            if (data_ready == 1) {
                send_spi_to_adc(null_command, mdata_buffer+inter_transmit_sample_counter*ADS131_PACKET_SIZE, ADS131_PACKET_SIZE, !SEND_DATA);        // new data saved in mdata_buffer and sent by RF
                sample_counter = sample_counter+1;
                inter_transmit_sample_counter=inter_transmit_sample_counter+1;
                data_ready = 0; //ensure last
            }
        }

        //only transmit buffer which is not start-up buffer
        uint16_t corrected_sample_counter = sample_counter-1;
        if(corrected_sample_counter > 2){
            send_databuffer(mdata_buffer,sizeof(mdata_buffer));
        }

    }

    return 1;

}


int transmit_adc_samples_skip(const uint64_t NUM_SAMPLES, const uint64_t TRANSMIT_FREQ) {

    uint64_t sample_counter = 0;
    uint64_t corrected_sample_counter = 0;

    while(sample_counter<NUM_SAMPLES){
    // reset data buffer so not transmit old data
            uint16_t mdata_buffer[ADS131_PACKET_SIZE*NUM_SAMPLES_PER_PACKET] = {0x0000};    //buffer data is saved to
            uint16_t null_command[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};   //buffer to send over SPI; the NULL (sample) command

            // sample NUM_SAMPLES_PER_PACKET sequentially
            uint8_t inter_transmit_sample_counter = 0;  // counter for when to transmit
            while (inter_transmit_sample_counter < NUM_SAMPLES_PER_PACKET) {
                if (data_ready == 1) {
                    if (sample_counter % TRANSMIT_FREQ == 0) {
                        send_spi_to_adc(null_command, mdata_buffer+inter_transmit_sample_counter*ADS131_PACKET_SIZE, ADS131_PACKET_SIZE, !SEND_DATA);        // new data saved in mdata_buffer and soon sent by RF
                    }
                    sample_counter = sample_counter+1;
                    inter_transmit_sample_counter=inter_transmit_sample_counter+1;
                    data_ready = 0; //ensure last
                }
            }
            //transmit buffer
            corrected_sample_counter = sample_counter-1;
            if (corrected_sample_counter % TRANSMIT_FREQ == 0) {
            send_databuffer(mdata_buffer,sizeof(mdata_buffer));
            }

        }

        return 1;
}


#define MIN_TO_SEC 60
#define SEC_TO_SEC 1
#define HOURS_TO_SEC 3600
#define TRANSMIT (1)
int goto_sleep(const uint64_t STANDBY_SLEEP_DURATION, const uint64_t STANDBY_INTERVALS, const bool TRANSMIT_FLAG) {
    // GOTO_SLEEP puts everything to sleep for STANDBY_SLEEP_DURATION, transmits a info bit, and goes back to sleep for
    // STANDBY_INTERVALS; the total sleep time is STANDBY_SLEEP_DURATION*STANDBY_INTERVALS.
    // Inputs:
    //  STANDBY_SLEEP_DURATION  seconds for the microcontroller to go to sleep between STANDBY_INTERVALS
    //  STANDBY_INTERVALS       number of times the microcontroller should wakup to transmit, and return to sleep


    /* SLEEP */
    //GPIO_write(ADC_CLOCK_NOT_EN, 1);        // disable ADC clock (controlled by raising pin ADC_CLOCK_NOT_EN)
    PWM_stop(pwm);                          // disable ADC clock
    GPIO_write(ADC_CLOCK_NOT_EN, 0);    //Set pin to low during sleep (hopefully)

    GPIO_disableInt(3);                     // just in case, disable interupt

    // Put ADC to sleep
    uint16_t standby16bit[1] = {0x0022};    // 0x0022 is standby command
    uint16_t standbybitRx[1] = {0x0000};
    send_spi_to_adc(standby16bit, standbybitRx, 1, SEND_DEBUG_INFO);
    
  	// Close SPI and force DIN low
    SPI_close(masterSpi);
    
    if (STANDBY_INTERVALS == 0) {
        //no intervals, just sleep
        if (TRANSMIT_FLAG) {
            uint64_t standby_interval_ticker = 0;
            uint64_t standby_buffer[1] = {standby_interval_ticker};
            send_databuffer(standby_buffer, sizeof(standby_buffer));    // transmit ticker value to know device alive TODO:  Check if OK to send uint64_t or if needs to be array and sizeof ok, otherwise should be =8.
        }
        sleep(STANDBY_SLEEP_DURATION);
    } else {
        // Wakup to transmit hello if required
        uint64_t standby_interval_ticker = 0;
        while (standby_interval_ticker < STANDBY_INTERVALS) {
            if (TRANSMIT_FLAG) {
                uint64_t standby_buffer[1] = {standby_interval_ticker};
                send_databuffer(standby_buffer, sizeof(standby_buffer));    // transmit ticker value to know device alive TODO:  Check if OK to send uint64_t or if needs to be array and sizeof ok, otherwise should be =8.
            }
            sleep(STANDBY_SLEEP_DURATION);                                // sleep for standby_sleep_duration (15min)
            standby_interval_ticker = standby_interval_ticker + 1;        // increment ticker
        }
    }

//     if (STANDBY_INTERVALS == 0) {
//     	return 1
//     	}

//    // Wakup to transmit hello if required
//    uint64_t standby_interval_ticker = 0;
//    while (standby_interval_ticker < STANDBY_INTERVALS) {
//        if (TRANSMIT_FLAG) {
//            uint64_t standby_buffer[1] = {standby_interval_ticker};
//            send_databuffer(standby_buffer, sizeof(standby_buffer));    // transmit ticker value to know device alive TODO:  Check if OK to send uint64_t or if needs to be array and sizeof ok, otherwise should be =8.
//        }
//        sleep(STANDBY_SLEEP_DURATION);                                // sleep for standby_sleep_duration (15min)
//        standby_interval_ticker = standby_interval_ticker + 1;        // increment ticker
//    }


    /* WAKEUP */
    //GPIO_write(ADC_CLOCK_NOT_EN, 0); //enable ADC Clock
    PWM_start(pwm);                     //enable ADC clock

    // Restart SPI
    migut_open_spi();

    /* Wakeup ADC */
    uint16_t wakeup16bit[1] = {0x0033};
    uint16_t wakeupbitRx[1] = {0x0000};
    send_spi_to_adc(wakeup16bit, wakeupbitRx, 1, SEND_DEBUG_INFO);

    GPIO_enableInt(3);  //enable DRDY interupt

    return 1;
}

// FUNCTION TO GO TO SLEEP IN USEC >> CONVERTED TO MSEC IN USLEEP()
int goto_u_sleep(const uint64_t STANDBY_SLEEP_DURATION, const uint64_t STANDBY_INTERVALS, const bool TRANSMIT_FLAG) {
    // GOTO_SLEEP puts everything to sleep for STANDBY_SLEEP_DURATION, transmits a info bit, and goes back to sleep for
    // STANDBY_INTERVALS; the total sleep time is STANDBY_SLEEP_DURATION*STANDBY_INTERVALS.
    // Inputs:
    //  STANDBY_SLEEP_DURATION  milliseconds for the microcontroller to go to sleep between STANDBY_INTERVALS
    //  STANDBY_INTERVALS       number of times the microcontroller should wakup to transmit, and return to sleep


    /* SLEEP */
    //GPIO_write(ADC_CLOCK_NOT_EN, 1);        // disable ADC clock (controlled by raising pin ADC_CLOCK_NOT_EN)
    PWM_stop(pwm);                          // disable ADC clock
    GPIO_write(ADC_CLOCK_NOT_EN, 0);    //Set pin to low during sleep (hopefully)

    GPIO_disableInt(3);                     // just in case, disable interupt

//     Put ADC to sleep
    uint16_t standby16bit[1] = {0x0022};    // 0x0022 is standby command
    uint16_t standbybitRx[1] = {0x0000};
    send_spi_to_adc(standby16bit, standbybitRx, 1, SEND_DEBUG_INFO);

//     Close SPI and force DIN low
    SPI_close(masterSpi);

    if (STANDBY_INTERVALS == 0) {
        //no intervals, just sleep
        if (TRANSMIT_FLAG) {
            uint64_t standby_interval_ticker = 0;
            uint64_t standby_buffer[1] = {standby_interval_ticker};
            send_databuffer(standby_buffer, sizeof(standby_buffer));    // transmit ticker value to know device alive TODO:  Check if OK to send uint64_t or if needs to be array and sizeof ok, otherwise should be =8.
        }
        usleep(STANDBY_SLEEP_DURATION*1000);
    } else {
        // Wakup to transmit hello if required
        uint64_t standby_interval_ticker = 0;
        while (standby_interval_ticker < STANDBY_INTERVALS) {
            if (TRANSMIT_FLAG) {
                uint64_t standby_buffer[1] = {standby_interval_ticker};
                send_databuffer(standby_buffer, sizeof(standby_buffer));    // transmit ticker value to know device alive TODO:  Check if OK to send uint64_t or if needs to be array and sizeof ok, otherwise should be =8.
            }
            usleep(STANDBY_SLEEP_DURATION*1000);                                // sleep for standby_sleep_duration (15min)
            standby_interval_ticker = standby_interval_ticker + 1;        // increment ticker
        }
    }


    /* WAKEUP */
    //GPIO_write(ADC_CLOCK_NOT_EN, 0); //enable ADC Clock
    PWM_start(pwm);                     //enable ADC clock

//     Restart SPI
    migut_open_spi();

    /* Wakeup ADC */
    uint16_t wakeup16bit[1] = {0x0033};
    uint16_t wakeupbitRx[1] = {0x0000};
    send_spi_to_adc(wakeup16bit, wakeupbitRx, 1, SEND_DEBUG_INFO);

    GPIO_enableInt(3);  //enable DRDY interupt

    return 1;
}

int goto_sleep_min(const uint64_t SLEEP_IN_MIN) {


    const uint64_t MYF_STANDBY_SLEEP_DURATION = PING_PERIOD_MIN*MIN_TO_SEC; //20s for debug
    const uint64_t SLEEP_TIME_IN_SEC = SLEEP_IN_MIN*MIN_TO_SEC;
    if (SLEEP_TIME_IN_SEC < MYF_STANDBY_SLEEP_DURATION) {
        //do not count sleep in terms of number of pings
        const uint64_t ZERO_STANDBY_INTERVOLS = 0;
        return goto_sleep(SLEEP_TIME_IN_SEC, ZERO_STANDBY_INTERVOLS, TRANSMIT);
    }
    else{
        const uint64_t MYF_STANDBY_INTERVALS = (uint64_t) (SLEEP_IN_MIN*MIN_TO_SEC/MYF_STANDBY_SLEEP_DURATION);
        return goto_sleep(MYF_STANDBY_SLEEP_DURATION, MYF_STANDBY_INTERVALS, TRANSMIT);
    }

}

int goto_sleep_sec(const uint64_t SLEEP_IN_SEC) {

    const uint64_t SEC_STANDBY_SLEEP_DURATION = PING_PERIOD_SEC;
    if (SLEEP_IN_SEC < SEC_STANDBY_SLEEP_DURATION) {
        const uint64_t ZERO_STANDBY_INTERVOLS = 0;
        return goto_sleep(SLEEP_IN_SEC, ZERO_STANDBY_INTERVOLS, TRANSMIT);
    }
    else{
        const uint64_t SEC_STANDBY_INTERVALS = (uint64_t) (SLEEP_IN_SEC/SEC_STANDBY_SLEEP_DURATION);
        return goto_sleep(SEC_STANDBY_SLEEP_DURATION, SEC_STANDBY_INTERVALS, TRANSMIT);
    }

}

int goto_sleep_msec(const uint64_t SLEEP_IN_MSEC) {
    uint64_t SLEEPTIME = SLEEP_IN_MSEC; // Sleep in msec is global variable, but sleeptime is local, we need to modify it if it is too large (>999)
    if (SLEEPTIME > 999) {
        SLEEPTIME = 999;
    }
    if (SLEEPTIME < PING_PERIOD_MSEC) {
        const uint64_t ZERO_STANDBY_INTERVOLS = 0;
        return goto_u_sleep(SLEEPTIME, ZERO_STANDBY_INTERVOLS, 0);
    }
    else{
        const uint64_t MSEC_STANDBY_INTERVALS = (uint64_t) (SLEEPTIME/PING_PERIOD_MSEC);
        return goto_u_sleep(PING_PERIOD_MSEC, MSEC_STANDBY_INTERVALS, TRANSMIT);
    }

}


/*
 *  ======== masterThread ========
 *  Master SPI sends a message to slave while simultaneously receiving a
 *  message from the slave.
 */
void *masterThread(void *arg0)
{

    /* INIT */
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH+2;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    uint8_t test_startup_buffer[1] = {0x28};
    send_databuffer(test_startup_buffer,sizeof(test_startup_buffer));     //confirm Tx OK

    Power_enablePolicy();

    //GPIO_write(ADC_CLOCK_NOT_EN, 1);    //Disable SITIME ADC CLock

    // Initialize the PWM parameters for ADC Clock
    PWM_Params_init(&pwmParams);
    pwmParams.idleLevel = PWM_IDLE_HIGH;      // Output low when PWM is not running
    pwmParams.periodUnits = PWM_PERIOD_HZ;   // Period is in Hz
    pwmParams.periodValue = 2e6;             // 2.001MHz
    pwmParams.dutyUnits = PWM_DUTY_FRACTION; // Duty is in fractional percentage
    pwmParams.dutyValue = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 50) / 100); //50% clock

    // Open the PWM instance
    pwm = PWM_open(Board_PWM2, &pwmParams);
    if (pwm == NULL) {
            // PWM_open() failed
            while (1); // TODO change to RF send help command
    }

    PWM_start(pwm);

    /* START OPERATIONS */
    extern const uint64_t NUMBER_OF_SAMPLES_ARRAY[];
    extern const uint64_t SLEEP_TIME_ARRAY[];
    extern const uint16_t RECORDING_MODE_ARRAY[];
    extern const uint16_t SLEEP_MODE_ARRAY[];


    init_ads131m08();           // init ADC

    uint16_t sst_array_index = 0;
    uint16_t first_index_flag = 0;
    const uint16_t sst_array_size = sizeof(NUMBER_OF_SAMPLES_ARRAY) / sizeof(NUMBER_OF_SAMPLES_ARRAY[0]);

    goto_sleep_sec(INIT_SLEEP);

    // Loop which will run forever
    for (sst_array_index = 0; TRUE; sst_array_index=(sst_array_index+1) % sst_array_size) {

        if (sst_array_index==0 && first_index_flag==1 && FIRST_INDEX_NO_REPEAT==1){
            //empty skip IF nothing
        } else {
            if (RECORDING_MODE_ARRAY[sst_array_index]==1) {

                transmit_multiple_adc_samples(NUMBER_OF_SAMPLES_ARRAY[sst_array_index]); //adam func

            }

            if (RECORDING_MODE_ARRAY[sst_array_index]==2) {
                uint64_t j;
                for (j = 0; j<NUMBER_OF_SAMPLES_ARRAY[sst_array_index]; j++){
                    transmit_adc_samples_secs(N_RECORDING_MODE_2); //coen func
                    goto_sleep_msec(TIME_BETWEEN_MS_MODE_2);
                    }
            }

            if (RECORDING_MODE_ARRAY[sst_array_index]==3) {
                            uint64_t j;
                            for (j = 0; j<NUMBER_OF_SAMPLES_ARRAY[sst_array_index]; j++){
                                transmit_adc_samples_secs(N_RECORDING_MODE_3); //coen func
                                goto_sleep_sec(TIME_BETWEEN_S_MODE_3);
                                if (TIME_BETWEEN_MS_MODE_3>0){
                                    goto_sleep_msec(TIME_BETWEEN_MS_MODE_3);
                                    }
                                }
                        }


            if (SLEEP_MODE_ARRAY[sst_array_index]==1) {
                    goto_sleep_min(SLEEP_TIME_ARRAY[sst_array_index]);          //sleep for X minutes (defined in SLEEP_TIME_ARRAY array in "EGG_Experiment_Params.h")
            }

            if (SLEEP_MODE_ARRAY[sst_array_index]==2) {
                    goto_sleep_sec(SLEEP_TIME_ARRAY[sst_array_index]);          //sleep for X seconds (defined in SLEEP_TIME_ARRAY array in "EGG_Experiment_Params.h")
            }

            if (SLEEP_MODE_ARRAY[sst_array_index]==3) {
                    goto_sleep_msec(SLEEP_TIME_ARRAY[sst_array_index]);         //sleep for X milliseconds (defined in SLEEP_TIME_ARRAY array in "EGG_Experiment_Params.h")
            }

        }
        if (sst_array_index == 0) {
            first_index_flag = 1; //Flip flag the first time the first element is run
        }

    }


//     Should do these at end of program, but unreachable since we want the device to die in loop
//     Commenting so no warnings during build.
//    SPI_close(masterSpi);
//
//    GPIO_disableInt(3);
//    GPIO_setConfig(3, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
//    GPIO_write(3, 0);
//
//    GPIO_write(DEBUG_GPIO_1, 1);
//    GPIO_write(DEBUG_GPIO_1, 0);
//
//
//    Display_printf(display, 0, 0, "\nDone");

//    return (NULL);
}







/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    pthread_t           thread0;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    /* Call driver init functions. */
    Display_init();
    GPIO_init();
    SPI_init();
    Power_init();
    Power_enablePolicy();
    PWM_init();

    /* Configure the LED pins */
//    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
//    GPIO_setConfig(Board_GPIO_LED1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);


    GPIO_setConfig(DEBUG_GPIO_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);


   /* Turn on user LED */
   GPIO_write(DEBUG_GPIO_1, 1);
   GPIO_write(DEBUG_GPIO_1, 0);



    /* Open the display for output */
//    display = Display_open(Display_Type_UART, NULL);
//    if (display == NULL) {
//        /* Failed to open display driver */
//        while (1);
//    }

    /* Turn on user LED */
    //GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

//    Display_printf(display, 0, 0, "Starting the SPI master example");
//    Display_printf(display, 0, 0, "This example requires external wires to be "
//        "connected to the header pins. Please see the Board.html for details.\n");

    /* Create application threads */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    /* Create master thread */
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread0, &attrs, masterThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }

    return (NULL);
}
