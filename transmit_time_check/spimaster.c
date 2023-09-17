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

#include <ti/drivers/rf/RF.h>
#include <ti/drivers/Power.h>

#include "ads131m0x.h"

/* Example/Board Header files */
#include "Board.h"
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include "smartrf_settings/smartrf_settings.h"

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      20

#define DEBUG_GPIO_1 1
#define ADC_CLOCK_NOT_EN 5

/***** Prototypes *****/

/***** Variable declarations for RF*****/
RF_Object rfObject;
RF_Handle rfHandle;
uint8_t packet[PAYLOAD_LENGTH+2];
//static uint8_t packet[PAYLOAD_LENGTH+2];
static uint16_t seqNumber;
uint16_t testbuffer[300]={0,1,2,3,10};
uint16_t testbuffer2[300]={5,11,10,10,10};
uint16_t testbuffer3[300]={9,9,9,9,9};
void send_databuffer(const void* buffer,int buffer_size);
int send_spi_to_adc(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t words_to_send);

#define THREADSTACKSIZE (1024)

#define SPI_MSG_LENGTH  (10)
//#define MASTER_MSG      ("Hello from master, msg#: ")

#define MAX_LOOP        (10)

static Display_Handle display;

int16_t masterRxBuffer[SPI_MSG_LENGTH];
int16_t masterTxBuffer[SPI_MSG_LENGTH];
SPI_Transaction transaction;
SPI_Handle      masterSpi;
SPI_Params      spiParams;
uint8_t data_ready = 0;

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

int send_spi_to_adc(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t words_to_send) {
    int return_value = 0;

    transaction.count = words_to_send; //9; testing changing to 10
    transaction.txBuf = (void *) tx_buffer;
    transaction.rxBuf = (void *) rx_buffer;

    GPIO_write(DEBUG_GPIO_1, 1);
    GPIO_write(DEBUG_GPIO_1, 0);
    bool spitransferOK;
    spitransferOK = SPI_transfer(masterSpi, &transaction);
    if (spitransferOK) {
        //do nothing
        return_value = 1;
    }
    else {
        return_value = 0;
    }

    send_databuffer(tx_buffer,words_to_send*2); //TODO check if safe to change to words_to_send*2, was 20 if not working
    send_databuffer(rx_buffer,words_to_send*2);

    return return_value;

}


/*
 *  ======== masterThread ========
 *  Master SPI sends a message to slave while simultaneously receiving a
 *  message from the slave.
 */
void *masterThread(void *arg0)
{

    /* START */
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH+2;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    Power_enablePolicy();
    GPIO_write(DEBUG_GPIO_1, 0);

  //  uint32_t    standbyDuration = 5;
 //   uint32_t    ticker = 0;

    /*
     * Repeatedly sleeps for a duration, to allow repeated entry/exit
     * from standby. The LED states are toggled on each iteration
     */
//    while(ticker < 1) {
//        /* Sleep, to let the power policy transition the device to standby */
//        GPIO_write(DEBUG_GPIO_1, 1);
//        sleep(standbyDuration);
//        ticker = ticker +1;
//
//        /* Read current output value for all pins */
//        currentOutputVal =  PIN_getPortOutputValue(hPin);
//
//        /* Toggle the LEDs, configuring all LEDs at once */
//        PIN_setPortOutputValue(hPin, ~currentOutputVal);
//        GPIO_toggle(DEBUG_GPIO_1);
//     }
//    GPIO_write(DEBUG_GPIO_1, 0);

    /* Open SPI as master (default) */
    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.bitRate = 4000000;
    spiParams.dataSize = 16;    //While the ADS131M0x default is 24bit packets, the CC1310 is limited to 16.
    spiParams.transferMode = SPI_MODE_BLOCKING;
    GPIO_write(Board_SPI_MASTER_READY, 0); //AG CHECK TODO check

    masterSpi = SPI_open(0, &spiParams);
    if (masterSpi == NULL) {
        //Display_printf(display, 0, 0, "Error initializing master SPI\n");
        while (1);
    }
    else {
        //Display_printf(display, 0, 0, "Master SPI initialized\n");
    }



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // SETUP ADC

    //uint16_t init24bit[10] = {0x6104, 0x0000, 0x1000, 0xff1c, 0x0033, 0x3300, 0x3333, 0x0006, 0x0000};
    uint16_t init24bitRx[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

    /* RESET */
    uint16_t reset24bit[10] = {0x0011, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
    send_spi_to_adc(reset24bit, init24bitRx, 10);

    /* Set word length to 16bit */
    uint16_t sixteenbit24bit[3] = {0x6100, 0x0004, 0x1000};     // {0x6100, 0x0004, 0x1000};
    uint16_t sixteenbit24bitRx[3] = {0x6100, 0x0004, 0x1000};   // 0011 0000 0687
    send_spi_to_adc(sixteenbit24bit, sixteenbit24bitRx, 3);

    /* Config CLOCK Register */
    // set VLP Mode (1.8mA ON), disable crystal, and OSR to 128 (8kSPS)
    uint16_t lpm16bit[2] = {0x6180, 0xff80}; //GOOD
    uint16_t lpm16bitRx[2] = {0x0000, 0x0000};
    send_spi_to_adc(lpm16bit, lpm16bitRx, 2);

#define GAIN_ALL_TWO

    /* Config GAIN1 Register */
    // Set gain to 2x
    uint16_t gain1_16bit[2] = {0x6200, GAIN_ALL_TWO}; //GOOD
    uint16_t gain1_16bitRx[2] = {0x0000, 0x0000};
    send_spi_to_adc(gain1_16bit, gain1_16bitRx, 2);

    /* Config GAIN1 Register */
    // Set gain to 2x
    uint16_t gain2_16bit[2] = {0x6280, GAIN_ALL_TWO}; //GOOD
    uint16_t gain2_16bitRx[2] = {0x0000, 0x0000};
    send_spi_to_adc(gain2_16bit, gain2_16bitRx, 2);

    /*  SAMPLE FOR TESTING CONTACTS AND CONFIRM GOOD DATA */

    GPIO_setConfig(3, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);//GPIO_CFG_IN_PU
    GPIO_setCallback(3, slaveReadyFxn);
    GPIO_enableInt(3);

    uint8_t sample_counter = 0;
    const uint8_t num_test_samples = 10;
    while (sample_counter<num_test_samples) {
            if (data_ready == 1) {

                uint16_t null_command[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
                uint16_t mdata_buffer[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
                send_spi_to_adc(null_command, mdata_buffer, 10);        // new data saved in mdata_buffer and sent by RF
                sample_counter = sample_counter+1;
                data_ready = 0;
            }
        }


    /* SLEEP */
    GPIO_write(ADC_CLOCK_NOT_EN, 1);        // disable ADC clock (controlled by raising pin ADC_CLOCK_NOT_EN)
    GPIO_disableInt(3);                     // just in case, disable interupt
    uint16_t standby16bit[1] = {0x0022};    // 0x0022 is standby command
    uint16_t standbybitRx[1] = {0x0000};
    send_spi_to_adc(standby16bit, standbybitRx, 1);


    /* put Micro to sleep */

#define MIN_TO_SEC 60
#define HOURS_TO_SEC 3600

//    const uint64_t hours_until_wake = 5;                    // hours after device ON for main recording loop to start
    const uint64_t standby_sleep_duration = 10;  // seconds between each "I'm awake transmission"

    const uint64_t standby_intervals_before_first_rec = 10;
    uint64_t standby_ticker = 0;
    while (standby_ticker < standby_intervals_before_first_rec) {
        uint64_t standby_buffer[1] = {standby_ticker};
        send_databuffer(standby_buffer, sizeof(standby_buffer)/8);              // transmit ticker value to know device alive TODO:  Check if OK to send uint64_t or if needs to be array and sizeof ok, otherwise should be =8.
        sleep(standby_sleep_duration);              // sleep for standby_sleep_duration (15min)
        standby_ticker = standby_ticker + 1;        // increment ticker
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PUT DEVICE IN ANIMAL
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /* WAKEUP */
    GPIO_write(DEBUG_GPIO_1, 0);
    GPIO_write(ADC_CLOCK_NOT_EN, 0);

    /* Wakeup ADC */
    uint16_t wakeup16bit[1] = {0x0033};
    uint16_t wakeupbitRx[1] = {0x0000};
    send_spi_to_adc(wakeup16bit, wakeupbitRx, 1);

    /* GET SAMPLES */

//    GPIO_setConfig(3, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);//GPIO_CFG_IN_PU
//    GPIO_setCallback(3, slaveReadyFxn);
    GPIO_enableInt(3);

    // first recording session after animal wakes up
//    uint64_t main_sample_counter = 0;
//    const uint64_t SAMPLES_BEFORE_SLEEP = 500000;
//    while (main_sample_counter < SAMPLES_BEFORE_SLEEP) {
//
//        uint16_t null_command[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
//        uint16_t mdata_buffer[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
//        send_spi_to_adc(null_command, mdata_buffer, 10);
//        main_sample_counter = main_sample_counter+1;
//        data_ready = 0;
//    }

    // main sleep-record loop until device dies
//    while (1) {
//
//        /* SLEEP */
//        // Put ADC to sleep
//        GPIO_write(ADC_CLOCK_NOT_EN, 1);        // disable ADC clock (controlled by raising pin ADC_CLOCK_NOT_EN)
//        GPIO_disableInt(3);                     // just in case, disable interupt
//        uint16_t standby16bit[1] = {0x0022};    // 0x0022 is standby command
//        uint16_t standbybitRx[1] = {0x0000};
//        send_spi_to_adc(standby16bit, standbybitRx, 1);
//
//        // Sleep uC
//        const uint64_t hours_until_wake_loop = 5;                    // hours after device ON for main recording loop to start
//        const uint64_t standby_sleep_duration = 15*MIN_TO_SEC;  // seconds between each "I'm awake transmission"
//
//        const uint64_t standby_intervals_before_loop_rec = hours_until_wake_loop*HOURS_TO_SEC/standby_sleep_duration;
//        uint64_t standby_ticker = 0;
//        while (standby_ticker < standby_intervals_before_loop_rec) {
//            uint64_t standby_buffer[1] = {standby_ticker};
//            send_databuffer(standby_buffer, sizeof(standby_buffer)/8);              // transmit ticker value to know device alive TODO:  Check if OK to send uint64_t or if needs to be array and sizeof ok, otherwise should be =8.
//            sleep(standby_sleep_duration);              // sleep for standby_sleep_duration (15min)
//            standby_ticker = standby_ticker + 1;        // increment ticker
//        }
//
//        /* RECORD */
//        // Wakeup ADC
//
//        GPIO_write(DEBUG_GPIO_1, 0);
//        GPIO_write(ADC_CLOCK_NOT_EN, 0);
//
//        /* Wakeup ADC */
//        uint16_t wakeup16bit[1] = {0x0033};
//        uint16_t wakeupbitRx[1] = {0x0000};
//        send_spi_to_adc(wakeup16bit, wakeupbitRx, 1);
//
//        /* GET SAMPLES */
//
//    //    GPIO_setConfig(3, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);//GPIO_CFG_IN_PU
//    //    GPIO_setCallback(3, slaveReadyFxn);
//        GPIO_enableInt(3);
//
//        // first recording session after animal wakes up
//        main_sample_counter = 0;
//        //const uint64_t SAMPLES_BEFORE_SLEEP = 500000;
//        while (main_sample_counter < SAMPLES_BEFORE_SLEEP) {
//
//            uint16_t null_command[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
//            uint16_t mdata_buffer[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
//            send_spi_to_adc(null_command, mdata_buffer, 10);
//            main_sample_counter = main_sample_counter+1;
//            data_ready = 0;
//        }
//
//
//
//
//
//
//    }



    SPI_close(masterSpi);

    GPIO_disableInt(3);
    GPIO_setConfig(3, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_write(3, 0);

    GPIO_write(DEBUG_GPIO_1, 1);
    GPIO_write(DEBUG_GPIO_1, 0);


    Display_printf(display, 0, 0, "\nDone");

    return (NULL);
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
