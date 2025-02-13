/*
 * TuyaDriver.h
 *
 *  Created on: Sep 6, 2024
 *      Author: 26081178
 */

#ifndef SRC_APPLICATIONLAYER_TUYADRIVER_H_
#define SRC_APPLICATIONLAYER_TUYADRIVER_H_

#include "DataTypes.h"
#include "DriverLayer.h"

#define TUYA_UART_BUFFER_SIZE               256U
#define TUYA_DP_BUFFER_SIZE                 32U
#define TUYA_TX_FIFO_SIZE                   16U
#define TUYA_RX_FIFO_SIZE                   8U

#define TUYA_MINIMUM_FRAME_SIZE             7U
#define TUYA_FRAME_DATA_SIZE(data_length)   TUYA_MINIMUM_FRAME_SIZE + data_length /* Header(2) + Version(1) + Command(1) +
                                                                                     Length(2) + Data(N) + Checksum(1) */

#define DATA_POINT_COUNT                	sizeof( TuyaDataPoints_t ) / sizeof( DataPoint_t )
#define MINIMUM_DATA_POINT_SIZE         	4U
#define DATA_POINT_SIZE(data_length)    	MINIMUM_DATA_POINT_SIZE + data_length    /* Id(1) + Type(1) + Length(2) + Data(N) */


typedef enum
{
    TUYA_CMD_HEARTBEAT     			   = 0x0,
    TUYA_CMD_PRODUCT_INFO              = 0x1,
    TUYA_CMD_WORKING_MODE              = 0x2,
    TUYA_CMD_REPORT_NETWORK_STATUS 	   = 0x3,
    TUYA_CMD_RESET_WIFI                = 0x4,
    TUYA_CMD_SELECT_PAIRING_MODE 	   = 0x5,
    TUYA_CMD_SEND	  	  			   = 0x6,		/* Sent by the module */
    TUYA_CMD_REPORT_STATUS 			   = 0x7,		/* Sent by the MCU */
    TUYA_CMD_QUERY_WORKING_STATUS	   = 0x8,
    /* ----------------------------------- */
    TUYA_CMD_UNKNOWN				   = 0xFF,
} TuyaCommand_t;

typedef enum
{
    HEADER_INDEX  	   = 0,			/* 2 bytes */
    VERSION_INDEX 	   = 2,			/* 1 byte  */
    COMMAND_INDEX 	   = 3,			/* 1 byte  */
    DATA_LENGTH_INDEX  = 4,			/* 2 bytes */
    DATA_START_INDEX   = 6			/* N Bytes */
} DataByteIndex_t;

typedef enum
{
    DP_TYPE_RAW      = 0x00,
    DP_TYPE_BOOLEAN	 = 0x01,
    DP_TYPE_VALUE	 = 0x02,
    DP_TYPE_STRING	 = 0x03,
    DP_TYPE_ENUM	 = 0x04,
    DP_TYPE_BITMAP	 = 0x05
} DataPointType_t;

typedef enum
{
    DP_SIZE_BOOLEAN	  	   = 1,
    DP_SIZE_VALUE	  	   = 4,
    DP_SIZE_ENUM	  	   = 1,
    DP_SIZE_RAW_PHASE_DATA = 8,
} DataPointTypeSize_t;

typedef enum
{
    DP_INDEX_ID        = 0,
    DP_INDEX_TYPE      = 1,
    DP_INDEX_LENGTH_HB = 2,
    DP_INDEX_LENGTH_LB = 3,
    DP_INDEX_VALUE     = 4,
} DataPointIndex_t;

typedef enum
{
    DP_ID_TOTAL_FORWARD_ENERGY   = 1,
    DP_ID_WORK_STATE			 = 3,
    DP_ID_CHARGE_CURRENT_SET	 = 4,			/* Issue and report */
    DP_ID_SINGLE_PHASE_POWER	 = 5,
    DP_ID_PHASE_A				 = 6,
    DP_ID_PHASE_B				 = 7,
    DP_ID_PHASE_C				 = 8,
    DP_ID_TOTAL_POWER			 = 9,
    DP_ID_FAULT			 		 = 10,
    DP_ID_ALARM_SET_1			 = 11,			/* Issue and report */
    DP_ID_ALARM_SET_2			 = 12,			/* Issue and report */
    DP_ID_CONNECTION_STATE	     = 13,
    DP_ID_WORK_MODE			 	 = 14,			/* Issue and report */
    DP_ID_BALANCE_ENERGY		 = 15,
    DP_ID_CLEAR_ENERGY			 = 16,			/* Issue and report */
    DP_ID_ENERGY_CHARGE			 = 17,			/* Issue and report */
    DP_ID_SWITCH				 = 18,			/* Issue and report */
    DP_ID_SCHEDULE_CHARGING		 = 19,			/* Issue and report */
    DP_ID_CHARGE_CARD_ID		 = 20,			/* Issue and report */
    DP_ID_METER_ID				 = 22,
    DP_ID_SYSTEM_VERSION		 = 23,
    DP_ID_CURRENT_TEMP			 = 24,
    DP_ID_ONCE_CHARGE_ENERGY	 = 25,
    DP_ID_UPDATE_ALL_CARDS		 = 26,			/* Issue and report */
    DP_ID_ONLINE_STATE			 = 27,			/* Issue and report */
    DP_ID_DELAY_POWER_ON		 = 28,			/* Issue and report */
    DP_ID_LOCK_GUN				 = 29,			/* Issue and report */
    DP_ID_RANDOM_DELAY_SET		 = 30,			/* Issue and report */
    DP_ID_DSR_TIMER				 = 31,			/* Issue and report */
    DP_ID_IP_ADDRESS			 = 32,			/* Issue and report */
} DataPointID_t;

typedef struct
{
    uint8_t   ucID;
    uint8_t   ucType;
    uint8_t   ucLengthHigh;					/* Length High Byte */
    uint8_t   ucLengthLow;					/* Length Low Byte */
    uint8_t   ucValue[ TUYA_DP_BUFFER_SIZE ];
} DataPoint_t;

typedef struct
{
    DataPoint_t  xTotalForwardEnergy;
    DataPoint_t  xWorkState;
    DataPoint_t  xChargeCurrentSet;
    DataPoint_t  xSinglePhasePower;
    DataPoint_t  xPhaseA;
    DataPoint_t  xPhaseB;
    DataPoint_t  xPhaseC;
    DataPoint_t  xSwitch;
    DataPoint_t  xClearEnergy;
    DataPoint_t  xOnlineState;
    DataPoint_t  xLockGun;
    DataPoint_t  xCurrentTemp;
    DataPoint_t  xConnectionState;
} TuyaDataPoints_t;

typedef enum
{
    REGISTER_HEARTBEAT = 100,
    REGISTER_PRODUCT_INFORMATION,
    REGISTER_WORKING_MODE,
    REGISTER_WIFI_STATUS,
    REGISTER_WIFI_RESET,
    REGISTER_WORKING_STATUS,
    /* ------------------------ */
    REGISTER_UNKNOWN   = 0xFF
} Register_t;


typedef enum
{
    SMARTCONFIG_MODE = 0,
    AP_MODE
} PairingMode_t;

typedef enum
{
    LITTLE_ENDIAN = 0,
    BIG_ENDIAN
} Endianness_t;

typedef enum
{
    SMART_CONFIG_STATE  =  0x00,
    AP_STATE            =  0x01,
    WIFI_NOT_CONNECTED  =  0x02,
    WIFI_CONNECTED      =  0x03,
    WIFI_CONN_CLOUD     =  0x04,
    WIFI_LOW_POWER      =  0x05,
    SMART_AND_AP_STATE  =  0x06,
    /* ------------------------ */
    WIFI_STATE_UNKNOWN  =  0xff,
} WifiStatus_t;

typedef enum
{
    TUYA_BUSY, 			/* Operation is still in process */
    TUYA_OK,       		/* Operation has completed successfully */
    TUYA_ERROR,     	/* Operation encountered an error */
    TUYA_TIMEOUT     	/* Operation timed out */
} TuyaStatus_t;

typedef enum
{
    TUYA_FRAME_PREPARE,
    TUYA_FRAME_COPY_DATA,
    TUYA_FRAME_TRANSMIT,
    TUYA_FRAME_SENT
} TuyaSendState_t;

typedef struct
{
    uint16_t  usHeader;
    uint8_t   ucVersion;		/* Firmware Version */
    uint8_t   ucCommand;
    uint16_t  usDataLength;
    uint8_t   ucData[ TUYA_UART_BUFFER_SIZE ];
    uint8_t   ucChecksum;
} TuyaFrame_t;

typedef struct
{
    int32_t  lTotalForwardEnergy;
    int32_t  lWorkState;					/* Charging State */
    int32_t  lIset;							/* Charging Current Set */
    int32_t  lSinglePhasePower;
    int32_t  lVinR_Rms;
    int32_t  lVinS_Rms;
    int32_t  lVinT_Rms;
    int32_t  lIinR_Rms;
    int32_t  lIinS_Rms;
    int32_t  lIinT_Rms;
    int32_t  lActivePowerR;
    int32_t  lActivePowerS;
    int32_t  lActivePowerT;
    int32_t  lOnlineState;
    int32_t  lCurrentTemp;
    int32_t  lControlPilotState;
} TuyaRegisters_t;

typedef struct
{
    DataPointID_t xDataPoints[ TUYA_TX_FIFO_SIZE ]; 	/* Array to hold DP IDs */
    int16_t sCount;   									/* Number of data points to be sent */
    int16_t sHead;                                      /* Head index */
} TuyaTxFIFO_t;

typedef struct
{
    TuyaFrame_t xFrameBuffer[ TUYA_RX_FIFO_SIZE ];
    int16_t sCount;   									/* Number of commands to process */
    int16_t sHead;                                      /* Head index */
} TuyaRxFIFO_t;

typedef union
{
    uint32_t  all;

    struct
    {
        uint32_t STARTUP_COMPLETE 	: 1;
        /* --------------------- */
        uint32_t RESERVED  	   	    : 31;
    }Bits;
}TuyaStatusFlags_t;

/* Driver State */
typedef enum
{
    TUYA_STATE_IDLE = 0,
    TUYA_STATE_PROCESSING_RXFIFO,
	TUYA_STATE_PROCESSING_TXFIFO,
    TUYA_STATE_TRANSMITTING,
    TUYA_STATE_ERROR,
} TuyaState_t;


typedef struct
{
    /* Registers and Data Points */
    TuyaRegisters_t   xRegisters;
    TuyaDataPoints_t  xDataPoints;

    /* Communication Buffers */
    uint8_t 	  ucRxBuffer[ TUYA_UART_BUFFER_SIZE ];
    uint8_t 	  ucTxBuffer[ TUYA_UART_BUFFER_SIZE ];
    TuyaFrame_t   xTxFrame;
    TuyaFrame_t   xRxFrame;

    /* Frame FIFO */
    TuyaTxFIFO_t      xTxFIFO;
    TuyaRxFIFO_t 	  xRxFIFO;

    /* Internal */
    int16_t 	  sTxCount;			    		/* Transmit data count */
    int16_t 	  sRxCount;						/* Received Data Size */

    /* State Management */
    TuyaState_t        eState;
    TuyaCommand_t      eCommand;
    WifiStatus_t       eWifiStatus;
    PairingMode_t      ePairingMode;
    TuyaStatusFlags_t  xStatusFlags;

    UART_HandleTypeDef *huart;
    uint8_t isDataSent;

    /* Error Handling */
    uint32_t ErrorCount;
    uint32_t LastError;
} TuyaHandler_t;

TuyaStatus_t Tuya_Init( TuyaHandler_t* handler, UART_HandleTypeDef* huart );
TuyaStatus_t Tuya_Process(TuyaHandler_t* handler);
void Tuya_Set_Registers( TuyaRegisters_t *pxTuyaRegisters );
void Tuya_Update_Data_Points( TuyaRegisters_t *pxTuyaRegisters, TuyaDataPoints_t *xTuyaDataPoints );
void Tuya_Report_Data( TuyaHandler_t* handler);

#endif /* SRC_APPLICATIONLAYER_TUYADRIVER_H_ */
