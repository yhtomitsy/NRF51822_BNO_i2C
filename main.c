/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"
#include "math.h"

/*Pins to connect shield. */
#define ARDUINO_I2C_SCL_PIN 7
#define ARDUINO_I2C_SDA_PIN 30

/*UART buffer size. */
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

/*Common addresses definition for accelereomter. */
#define MMA7660_ADDR        (0x98U >> 1)

#define MMA7660_REG_XOUT    0x00U
#define MMA7660_REG_YOUT    0x01U
#define MMA7660_REG_ZOUT    0x02U
#define MMA7660_REG_TILT    0x03U
#define MMA7660_REG_SRST    0x04U
#define MMA7660_REG_SPCNT   0x05U
#define MMA7660_REG_INTSU   0x06U
#define MMA7660_REG_MODE    0x07U
#define MMA7660_REG_SR      0x08U
#define MMA7660_REG_PDET    0x09U
#define MMA7660_REG_PD      0x0AU

#define BNO_ADDRESS               	0x4B            // Device address when SA0 Pin 17 = GND; 0x4B SA0 Pin 17 = VDD
#define QP(n)                       (1.0f / (1 << n))                   // 1 << n ==  2^-n
#define radtodeg                    (180.0f / (22/7))
#define MAX_METADATA_SIZE           9
#define MAX_PACKET_SIZE             20            

//Registers
const uint8_t CHANNEL_COMMAND = 0;
const uint8_t CHANNEL_EXECUTABLE = 1;
const uint8_t CHANNEL_CONTROL = 2;
const uint8_t CHANNEL_REPORTS = 3;
const uint8_t CHANNEL_WAKE_REPORTS = 4;
const uint8_t CHANNEL_GYRO = 5;

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD


/* Mode for MMA7660. */
#define ACTIVE_MODE 1u

/*Failure flag for reading from accelerometer. */
#define MMA7660_FAILURE_FLAG (1u << 6)

/*Tilt specific bits*/
#define TILT_TAP_MASK (1U << 5)
#define TILT_SHAKE_MASK (1U << 7)

// [max 255, otherwise "int16_t" won't be sufficient to hold the sum
//  of accelerometer samples]
#define NUMBER_OF_SAMPLES 20

/* Define version of GCC. */
#define GCC_VERSION (__GNUC__ * 10000 \
                     + __GNUC_MINOR__ * 100 \
                     + __GNUC_PATCHLEVEL__)

/**
 * @brief Structure for holding sum of samples from accelerometer.
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} sum_t;
static sum_t m_sum = {0};

/**
 * @brief Union to keep raw and converted data from accelerometer samples at one memory space.
 */
typedef union{
    uint8_t raw;
    int8_t  conv;
}elem_t;

/**
 * @brief Enum for selecting accelerometer orientation.
 */
typedef enum{
    LEFT = 1,
    RIGHT = 2,
    DOWN = 5,
    UP = 6
}accelerometer_orientation_t;

/**
 * @brief Structure for holding samples from accelerometer.
 */
typedef struct
{
    elem_t  x;
    elem_t  y;
    elem_t  z;
    uint8_t tilt;
} sample_t;

#ifdef __GNUC_PATCHLEVEL__
#if GCC_VERSION < 50505
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"           // Hack to GCC 4.9.3 bug. Can be deleted after switch on using GCC 5.0.0
#endif
#endif
/* Buffer for samples. */
static sample_t m_sample_buffer[NUMBER_OF_SAMPLES] = {0};
#ifdef __GNUC_PATCHLEVEL__
#if GCC_VERSION < 50505
#pragma GCC diagnostic pop
#endif
#endif

/* Indicates if reading operation from accelerometer has ended. */
static volatile bool m_xfer_done = true;
/* Indicates if setting mode operation has ended. */
static volatile bool m_set_mode_done = false;
/* TWI instance. */
//static const nrf_drv_twi_t m_twi_mma_7660 = NRF_DRV_TWI_INSTANCE(0);
static const nrf_drv_twi_t m_twi_bno = NRF_DRV_TWI_INSTANCE(0);

static bool initialized = false;                  					// check if IMU has been initialized
const uint8_t quat_report = 0x05;          									// defines kind of rotation vector (0x05), geomagnetic (0x09), AR/VR (0x28),
const int reporting_frequency    = 400;           					// reporting frequency in Hz  // note that serial output strongly reduces data rate
const uint8_t B0_rate = 1000000 / reporting_frequency;      // calculate LSB (byte 0)
const uint8_t B1_rate = B0_rate >> 8;                       // calculate byte 1
static uint8_t readSuccess = 0;                             // confirms quaternion has been read successfully
static float q0,q1,q2,q3;                                		// quaternions q0 = qw 1 = i; 2 = j; 3 = k;
static float h_est;                                      		// heading accurracy estimation
static uint8_t stat_;                                    		// Status (0-3)
uint8_t cargo[50] = {0}; 																	// holds in coming data
static float quatI = 0;
static float quatJ = 0;
static float quatK = 0;
static float quatReal = 0;
static bool reset = false;
static bool requestID = false;
uint8_t i2C_event = 0;

// i2C SHTP protocol
// Length LSB
// Length MSB
// Channel
// Sequence Num
uint8_t SHTP_HEADER[4] = {0};
uint8_t shtpData[MAX_PACKET_SIZE];
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
uint8_t commandSequenceNumber = 0; //Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
uint32_t metaData[MAX_METADATA_SIZE]; //There is more than 10 words in a metadata record but we'll stop at Q point 3

// function prototypes
static void set_feature_cmd_QUAT(); 												// configure quaternion output
float qToFloat_(int16_t, uint8_t);    											// convert q point data into float
static void sendPacket_IMU(uint8_t, uint8_t);               // send data to IMU
void resetIMU();                                            // reset the IMU
void requestProductID();                                    // request the product ID

/**
 * @brief Function for casting 6 bit uint to 6 bit int.
 *
 */
__STATIC_INLINE void int_to_uint(int8_t * put, uint8_t data)
{
    if (!(data & MMA7660_FAILURE_FLAG))     //6th bit is failure flag - we cannot read sample
    {
        *put = (int8_t)(data << 2) / 4;
    }
}

/**
 * @brief UART events handler.
 */
static void uart_events_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**
 * @brief UART initialization.
 */
static void uart_config(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_events_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
void MMA7660_set_mode(void)
{
    ret_code_t err_code;
    /* Writing to MMA7660_REG_MODE "1" enables the accelerometer. */
    uint8_t reg[2] = {MMA7660_REG_MODE, ACTIVE_MODE};

    err_code = nrf_drv_twi_tx(&m_twi_bno, MMA7660_ADDR, reg, sizeof(reg), false);  
    APP_ERROR_CHECK(err_code);
    
    while(m_set_mode_done == false);
}

/**
 * @brief Function for averaging samples from accelerometer.
 */
void read_data(sample_t * p_new_sample)
{
    /* Variable to count samples. */
    static uint8_t sample_idx;
    static uint8_t prev_tilt;
    
    sample_t * p_sample = &m_sample_buffer[sample_idx];
    
    /* Subtracting oldest sample. */
    m_sum.x    -= p_sample->x.conv;
    m_sum.y    -= p_sample->y.conv;
    m_sum.z    -= p_sample->z.conv;
    
    p_sample->tilt = p_new_sample->tilt;    
    
    int_to_uint(&p_sample->x.conv, p_new_sample->x.raw);
    int_to_uint(&p_sample->y.conv, p_new_sample->y.raw);
    int_to_uint(&p_sample->z.conv, p_new_sample->z.raw);
    
    /* Adding new sample. This way we always have defined number of samples. */
    m_sum.x    += p_sample->x.conv;
    m_sum.y    += p_sample->y.conv;
    m_sum.z    += p_sample->z.conv;

    ++sample_idx;
    if (sample_idx >= NUMBER_OF_SAMPLES)
    {
        sample_idx = 0;
    }

    if (sample_idx == 0 || (prev_tilt && (prev_tilt != p_sample->tilt)))
    {
        char const * orientation;
        switch ((p_sample->tilt >> 2) & 0x07)
        {
            case LEFT: 
                orientation = "LEFT";
                break;
            case RIGHT:
                orientation = "RIGHT"; 
                break;
            case DOWN:             
                orientation = "DOWN";  
                break;
            case UP:
                orientation = "UP";    
                break;
            default: 
                orientation = "?";     
                break;
        }
        printf("X: %3d, Y: %3d, Z: %3d | %s%s%s\r\n",
                m_sum.x / NUMBER_OF_SAMPLES,
                m_sum.y / NUMBER_OF_SAMPLES,
                m_sum.z / NUMBER_OF_SAMPLES,
                orientation,
                (p_sample->tilt & TILT_TAP_MASK) ? " TAP"   : "",
                (p_sample->tilt & TILT_SHAKE_MASK) ? " SHAKE" : "");
                prev_tilt = p_sample->tilt;
    }
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
    ret_code_t err_code;
    static sample_t m_sample;
    readSuccess = 0;                           // nothing read
	
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
						//i2C_event++;
            /*if ((p_event->type == NRF_DRV_TWI_EVT_DONE) && (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)){
                if(m_set_mode_done != true)
                {
                    m_set_mode_done  = true;
                    return;
                }
                m_xfer_done = false;
                /* Read 4 bytes from the specified address. 
                err_code = nrf_drv_twi_rx(&m_twi_mma_7660, MMA7660_ADDR, (uint8_t*)&m_sample, sizeof(m_sample));
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                read_data(&m_sample);
                m_xfer_done = true;
            }*/
						SEGGER_RTT_printf(0,"TWI EVT DONE: %d\r\n", p_event->xfer_desc.type);
						SEGGER_RTT_printf(0,"request ID: %d\r\n", requestID);
						SEGGER_RTT_printf(0,"reset: %d\r\n", reset);
				
						if((p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)) // receive event
						{ 
								if(!initialized)
								{
										initialized = true;  
										return;
								}
								else 
									
								err_code = nrf_drv_twi_rx(&m_twi_bno, BNO_ADDRESS, cargo, sizeof(cargo));
								SEGGER_RTT_printf(0,"Err code: %d\r\n", err_code);
								//APP_ERROR_CHECK(err_code);	
								
								
								/*if(reset == true)// read all incoming metadata
								{  	
										err_code = nrf_drv_twi_rx(&m_twi_bno, BNO_ADDRESS, cargo, 4);
										APP_ERROR_CHECK(err_code);
										i2C_event++;
										for(uint8_t i = 0; i < 23; i++)SEGGER_RTT_printf(0,"%d,", cargo[i]);
										SEGGER_RTT_printf(0,"\r\n");*
								}
								else if(requestID == true)
								{
										err_code = nrf_drv_twi_rx(&m_twi_bno, BNO_ADDRESS, cargo, sizeof(cargo));
										APP_ERROR_CHECK(err_code);
										for(uint8_t i = 0; i < 23; i++)SEGGER_RTT_printf(0,"%d,", cargo[i]);
										SEGGER_RTT_printf(0,"\r\n");*
								}*/
						}
						else{
								if(reset == true)// read all incoming metadata
								{  	
										//Calculate the number of data bytes in this packet
										int16_t dataLength = ((uint16_t)cargo[1] << 8 | cargo[0]);
										dataLength &= ~(1 << 15); //Clear the MSbit.
										SEGGER_RTT_printf(0,"%d\r\n",dataLength);	
										err_code = nrf_drv_twi_rx(&m_twi_bno, BNO_ADDRESS, (uint8_t*)&cargo, sizeof(cargo));
										if(i2C_event == 6) reset = false;
										i2C_event++;
										
										for(uint8_t i = 0; i < 23; i++)SEGGER_RTT_printf(0,"%d,", cargo[i]);
										SEGGER_RTT_printf(0,"\r\n");
										/*err_code = nrf_drv_twi_rx(&m_twi_bno, BNO_ADDRESS, (uint8_t*)&cargo, sizeof(cargo));
										APP_ERROR_CHECK(err_code);*/
										
								}
								else if(requestID == true)
								{
										int16_t dataLength = ((uint16_t)cargo[1] << 8 | cargo[0]);
										dataLength &= ~(1 << 15); //Clear the MSbit.
										SEGGER_RTT_printf(0,"%d\r\n",dataLength);
										//err_code = nrf_drv_twi_rx(&m_twi_bno, BNO_ADDRESS, (uint8_t*)&cargo, sizeof(cargo));
										//APP_ERROR_CHECK(err_code);
										for(uint8_t i = 0; i < 23; i++)SEGGER_RTT_printf(0,"%d,", cargo[i]);
										SEGGER_RTT_printf(0,"\r\n");
								}
						}
            break;
        default:
            break;        
    }   
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t twi_bno_config = {
       .scl                = 4,//ARDUINO_SCL_PIN,
       .sda                = 3, //ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&m_twi_bno, &twi_bno_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi_bno);
}

/**
 * @brief BNO IMU initialization.
 */
void initializeIMU(){
		ret_code_t err_code;
		uint8_t reg[2] = {0, 0};
		
		// check if i2C communication is sucessful
		err_code = nrf_drv_twi_tx(&m_twi_bno, BNO_ADDRESS, reg, sizeof(reg), false);  
    APP_ERROR_CHECK(err_code);
		nrf_delay_ms(10);
		
    while (!initialized){	
      //SEGGER_RTT_printf(0,".");
    }
		SEGGER_RTT_printf(0,"IMU Available!\r\n");
		
		resetIMU();
		while(reset)
		{
			nrf_delay_ms(1);
		}
		
		//nrf_delay_ms(1000);
		requestProductID();
		while(requestID)
		{
			nrf_delay_ms(1);
		}
		
    /*err_code = nrf_drv_twi_tx(&m_twi_bno, BNO_ADDRESS, reg, sizeof(reg), false);  
    APP_ERROR_CHECK(err_code);
		nrf_delay_ms(10);*/
		
		/*SEGGER_RTT_printf(0,"Initializing\r\n");
    while (!initialized){	
      //SEGGER_RTT_printf(0,".");
    }
		SEGGER_RTT_printf(0,"Done!\r\n");*/
    //Serial.println("BNO found");
    //nrf_delay_ms(100);                     //needed to accept feature command; minimum not tested
    //set_feature_cmd_QUAT();                // set the required feature report data rate  are generated  at preset report interval
		
}

/**
 * @brief setup quaternion output
 */
static void set_feature_cmd_QUAT()// quat_report determines the kind of quaternions (see data sheets)
{                                 
		ret_code_t err_code;
		uint8_t quat_setup[21] = {21,0,2,0,0xFD,quat_report,0,0,0,B0_rate,B1_rate,0,0,0,0,0,0,0,0,0,0};  
		err_code = nrf_drv_twi_tx(&m_twi_bno, BNO_ADDRESS, quat_setup, sizeof(quat_setup), false);  
		APP_ERROR_CHECK(err_code);
		SEGGER_RTT_printf(0,"Quats enabled \r\n");
    /*Wire.beginTransmission(BNO_ADDRESS);   
    Wire.write(quat_setup, sizeof(quat_setup));            
    Wire.endTransmission();*/
}


static void get_QUAT()
{  
    readSuccess = 0;       
		ret_code_t err_code;	
		uint8_t reg[2] = {0, 0};
		
		err_code = nrf_drv_twi_tx(&m_twi_bno, BNO_ADDRESS, reg, sizeof(reg), false);  
		APP_ERROR_CHECK(err_code);
		while(!readSuccess){
				// do nothing
		}
    //Wire.requestFrom(BNO_ADDRESS,23);
    /*while (Wire.available()){
        if(readSuccess == 0)readSuccess = 1;
        cargo[j] = Wire.read();
        j++;
    }*/

    //Check to see if this packet is a sensor reporting its data to us
    if((readSuccess == 1) && (cargo[9] == quat_report) && (cargo[2] == 0x03) && (cargo[4] == 0xFB)){    //  && ((cargo[10]) == next_data_seqNum ) check for report and incrementing data seqNum
        //next_data_seqNum = ++cargo[10];                                         // predict next data seqNum              
        stat_ = cargo[11] & 0x03;                                                 // bits 1:0 contain the status (0,1,2,3)  
    
        float qI = (((int16_t)cargo[14] << 8) | cargo[13] ); 
        float qJ = (((int16_t)cargo[16] << 8) | cargo[15] );
        float qK = (((int16_t)cargo[18] << 8) | cargo[17] );
        float qReal = (((int16_t)cargo[20] << 8) | cargo[19] ); 

        quatReal = qToFloat_(qReal, 14); //pow(2, 14 * -1);//QP(14); 
        quatI = qToFloat_(qI, 14); //pow(2, 14 * -1);//QP(14); 
        quatJ = qToFloat_(qJ, 14); //pow(2, 14 * -1);//QP(14); 
        quatK = qToFloat_(qK, 14); //pow(2, 14 * -1);//QP(14);                  // apply Q point (quats are already unity vector)

        //if (quat_report == 0x05){  // heading accurracy only in some reports available
        h_est = (((int16_t)cargo[22] << 8) | cargo[21] );                        // heading accurracy estimation  
        h_est *= QP(12);                                                         // apply Q point 
        h_est *= radtodeg;                                                       // convert to degrees                
        //}
    }
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float qToFloat_(int16_t fixedPointValue, uint8_t qPoint)
{
		float qFloat = fixedPointValue;
		qFloat *= pow(2, (qPoint * -1));
		return (qFloat);
}

// reset BNO080
static void sendPacket_IMU(uint8_t channelNumber, uint8_t packetLength)
{
		ret_code_t err_code;	
	
		// first four bytes are for the header
		shtpData[0] = packetLength & 0xFF;
		shtpData[1] = packetLength >> 8;
		shtpData[2] = channelNumber;
		shtpData[3] = sequenceNumber[channelNumber]++;
		
		err_code = nrf_drv_twi_tx(&m_twi_bno, BNO_ADDRESS, shtpData, sizeof(shtpData), false); 
		APP_ERROR_CHECK(err_code);
		
		/*shtpData[0] = 5 & 0xFF;
		shtpData[1] = 5 >> 8;
		shtpData[2] = 1;
		shtpData[3] = sequenceNumber[1]++;
		shtpData[4] = 1;
		
		err_code = nrf_drv_twi_tx(&m_twi_bno, BNO_ADDRESS, shtpData, sizeof(shtpData), false); 
		APP_ERROR_CHECK(err_code);*/
	
		/*i2cPort->beginTransmission(_deviceAddress);

    //Send the 4 byte packet header
    _i2cPort->write(packetLength & 0xFF); //Packet length LSB
    _i2cPort->write(packetLength >> 8); //Packet length MSB
    _i2cPort->write(channelNumber); //Channel number
    _i2cPort->write(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

    //Send the user's data packet
    for (uint8_t i = 0 ; i < dataLength ; i++)
    {
      _i2cPort->write(shtpData[i]);
    }
    if (_i2cPort->endTransmission() != 0)
    {
      return (false);
    }*/
}

void resetIMU()
{
		reset = true;
		SEGGER_RTT_printf(0,"Resetting IMU\r\n");
		shtpData[4] = 1; //Reset
		//Attempt to start communication with sensor
		sendPacket_IMU(CHANNEL_EXECUTABLE, 5); //Transmit packet on channel 1, 5 bytes (4 header bytes, 1 data byte)
		
}

void requestProductID()
{
		requestID = true;
		SEGGER_RTT_printf(0,"Requesting product ID\r\n");
		//Check communication with device
		shtpData[4] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
		shtpData[5] = 0; //Reserved

		//Transmit packet on channel 2, 6 bytes (4 header bytes, 2 data bytes)
		sendPacket_IMU(CHANNEL_CONTROL, 6);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
   //uart_config();
   // int a = __GNUC__, c = __GNUC_PATCHLEVEL__;
    SEGGER_RTT_printf(0,"\n\rTWI sensor example\r\n");
    twi_init();
		SEGGER_RTT_printf(0,"\n\rTWI intited\r\n");
		initializeIMU();
		SEGGER_RTT_printf(0,"\n\rIMU inited\r\n");
    
    //uint8_t reg = 0;
    //ret_code_t err_code;
    
    while(true)
    {
				/*get_QUAT();
				SEGGER_RTT_printf(0, "%f,", quatReal);
				SEGGER_RTT_printf(0, "%f,", quatI);
				SEGGER_RTT_printf(0, "%f,", quatJ);
				SEGGER_RTT_printf(0, "%f,", quatK);
				nrf_delay_ms(10);
				//nrf_delay_ms(100);
        /* Start transaction with a slave with the specified address. */
        /*do
        {
            __WFE();
        }while(m_xfer_done == false);
        //err_code = nrf_drv_twi_tx(&m_twi_bno, MMA7660_ADDR, &reg, sizeof(reg), true);
        APP_ERROR_CHECK(err_code);
        m_xfer_done = false;*/
    }
}

/** @} */

