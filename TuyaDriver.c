/*
 * TuyaDriver.c
 *
 *  Created on: Sep 6, 2024
 *      Author: 26081178
 */

#include "TuyaConfig.h"
#include "TuyaDriver.h"
#include "SoftwareModules.h"
#include <string.h>

/* Private function prototypes */
static TuyaStatus_t Tuya_Uart_Init( TuyaHandler_t* handler, UART_HandleTypeDef* huart );
static TuyaStatus_t Tuya_Reset( TuyaHandler_t* handler );
static TuyaStatus_t Tuya_Frame_RxFifo_Push( TuyaHandler_t* handler, TuyaFrame_t* frame );
static TuyaStatus_t Tuya_Frame_RxFifo_Pop( TuyaHandler_t* handler, TuyaFrame_t* frame );
static TuyaStatus_t Tuya_Data_TxFifo_Push( TuyaTxFIFO_t* pxFifo, const DataPointID_t DataPointID);
static TuyaStatus_t Tuya_Data_TxFifo_Pop( TuyaTxFIFO_t* pxFifo, DataPointID_t* DataPointID );
static TuyaStatus_t Tuya_Handle_Command( TuyaHandler_t* handler );
static TuyaStatus_t Tuya_Handle_Data_Point( TuyaHandler_t* handler, DataPointID_t DataPointId );
static TuyaStatus_t Tuya_Handle_Transmission( TuyaHandler_t* handler );
static void Tuya_Construct_Frame( TuyaCommand_t eCommand, const uint8_t* pucData, uint16_t usLength,
                              TuyaFrame_t *pxFrame, Endianness_t eEndianType );
static uint8_t Tuya_Calculate_Checksum( TuyaFrame_t *pxFrame );
static DataPoint_t* Get_Data_Point( DataPointID_t xDataPointID, TuyaDataPoints_t *pxTuyaDataPoints );
static TuyaStatus_t Copy_Frame_To_Data_Point( TuyaFrame_t *pxTuyaFrameRx, DataPoint_t *pxDataPoint );
static TuyaStatus_t Copy_Data_To_Data_Point( const uint8_t* pucData, DataPoint_t *pxDataPoint );
static int16_t Copy_Data_Points_To_Buffer( uint8_t* pucBuffer, TuyaDataPoints_t *pxTuyaDataPoints );
static TuyaStatus_t Extract_Frame_Data( const uint8_t* pucData, TuyaFrame_t *pxFrame );
static int16_t Tuya_Copy_Frame_Data( uint8_t* pucBuffer, const TuyaFrame_t *pxFrame );

static void Tuya_Init_Data_Points( TuyaDataPoints_t *pxDataPoints );
static void Tuya_Set_Data_Point( DataPoint_t *pDataPoint, uint8_t ucID, uint8_t ucType, uint8_t ucLength );
static _Bool Is_Data_Point_Valid( int16_t sDataPointID );

static TuyaStatus_t Tuya_Reset_Wifi( TuyaHandler_t* handler );
static WifiStatus_t Get_WiFi_Status( TuyaHandler_t *handler );
static TuyaStatus_t Tuya_Select_Pairing_Mode( TuyaHandler_t* handler );
static uint8_t Is_First_Heartbeat( void );

static uint64_t Format_Phase_Data( uint32_t voltage, uint32_t current, uint32_t power );
static void Update_Data_Point_Value( DataPoint_t *pDataPoint, uint8_t *pucData, uint8_t length );


TuyaStatus_t Tuya_Init( TuyaHandler_t* handler, UART_HandleTypeDef* huart )
{
    if ( !handler )
    {
    	handler->eState = TUYA_STATE_ERROR;
    	return TUYA_ERROR;
    }

    /* Init Tuya Parameters */
    handler->eState = TUYA_STATE_IDLE;
    handler->isDataSent = true;							/* Set initial value */
    Tuya_Init_Data_Points( &handler->xDataPoints );

    /* Init Uart Module */
    Tuya_Uart_Init( handler, huart );

    /* Start receiving */
    HAL_UARTEx_ReceiveToIdle_DMA( handler->huart, handler->ucRxBuffer, sizeof( handler->ucRxBuffer ) );

    return TUYA_OK;
}

static TuyaStatus_t Tuya_Uart_Init( TuyaHandler_t* handler, UART_HandleTypeDef* huart )
{
    if ( !handler || !huart )
    {   return TUYA_ERROR;  }

    /* Set Pairing Mode */
    handler->ePairingMode = TUYA_SMARTCONFIG_MODE;

    /* Set handler uart to user parameter */
	handler->huart = huart;
	handler->huart->Init.BaudRate = TUYA_UART_BAUDRATE;
	handler->huart->Init.WordLength = UART_WORDLENGTH_8B;
	handler->huart->Init.StopBits = UART_STOPBITS_1;
	handler->huart->Init.Parity = UART_PARITY_NONE;
	handler->huart->Init.Mode = UART_MODE_TX_RX;
	handler->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	handler->huart->Init.OverSampling = UART_OVERSAMPLING_16;
	handler->huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	handler->huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	handler->huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if ( HAL_UART_Init( handler->huart ) != HAL_OK )
	{
		return TUYA_ERROR;
	}

	if ( HAL_UARTEx_SetTxFifoThreshold( handler->huart, UART_TXFIFO_THRESHOLD_8_8) != HAL_OK )
	{
		return TUYA_ERROR;
	}

	if ( HAL_UARTEx_SetRxFifoThreshold( handler->huart, UART_RXFIFO_THRESHOLD_8_8) != HAL_OK )
	{
		return TUYA_ERROR;
	}

	if ( HAL_UARTEx_DisableFifoMode( handler->huart ) != HAL_OK )
	{
		return TUYA_ERROR;
	}

	return TUYA_OK;
}

static TuyaStatus_t Tuya_Reset( TuyaHandler_t* handler )
{
	if ( HAL_UART_Init(handler->huart) != HAL_OK )
	{
		return TUYA_ERROR;
	}

	if ( HAL_UARTEx_SetTxFifoThreshold(handler->huart, UART_TXFIFO_THRESHOLD_8_8) != HAL_OK )
	{
		return TUYA_ERROR;
	}

	if ( HAL_UARTEx_SetRxFifoThreshold(handler->huart, UART_RXFIFO_THRESHOLD_8_8) != HAL_OK )
	{
		return TUYA_ERROR;
	}

	if ( HAL_UARTEx_DisableFifoMode(handler->huart) != HAL_OK )
	{
		return TUYA_ERROR;
	}

	/* Start receiving */
	HAL_UARTEx_ReceiveToIdle_DMA( handler->huart, handler->ucRxBuffer, sizeof( handler->ucRxBuffer ) );

	return TUYA_OK;
}

TuyaStatus_t Tuya_Process(TuyaHandler_t* handler)
{
    if ( !handler )
    {   return TUYA_ERROR;  }

    if ( handler->huart->ErrorCode )
    {   handler->eState = TUYA_STATE_ERROR;  }

    switch ( handler->eState )
    {
        case TUYA_STATE_IDLE:
        {
            /* Check for data in FIFO */
            if ( handler->xRxFIFO.sCount > 0 )
            {
                handler->eState = TUYA_STATE_PROCESSING_RXFIFO;
            }
            else if ( handler->xTxFIFO.sCount > 0 )
            {
            	handler->eState = TUYA_STATE_PROCESSING_TXFIFO;
            }
        }
        break;

        case TUYA_STATE_PROCESSING_RXFIFO:
        {
            /* Process next frame from Rx FIFO */
            if ( Tuya_Frame_RxFifo_Pop( handler, &handler->xRxFrame ) == TUYA_OK )
            {
                if ( Tuya_Handle_Command( handler ) == TUYA_OK )
                {
                    /* Put Tuya in Transmit State */
                    handler->eState = TUYA_STATE_TRANSMITTING;
                }
                else
                {
                    handler->eState = TUYA_STATE_IDLE;
                }
            }
            else
            {
                handler->eState = TUYA_STATE_IDLE;
            }
        }
        break;

        case TUYA_STATE_PROCESSING_TXFIFO:
        {
        	DataPointID_t DataPointId;

            /* Process next frame from Tx FIFO */
        	if ( Tuya_Data_TxFifo_Pop( &handler->xTxFIFO, &DataPointId ) == TUYA_OK )
        	{
        		if ( Tuya_Handle_Data_Point( handler, DataPointId ) == TUYA_OK )
        		{
        			handler->eState = TUYA_STATE_TRANSMITTING;
        		}
                else
                {
                    handler->eState = TUYA_STATE_IDLE;
                }
        	}
        	else
        	{
        		handler->eState = TUYA_STATE_IDLE;
        	}
        }
        break;

        case TUYA_STATE_TRANSMITTING:
        {
            if ( Tuya_Handle_Transmission( handler ) == TUYA_OK )
            {
                handler->eState = TUYA_STATE_IDLE;
            }
        }
        break;

        case TUYA_STATE_ERROR:
        {
            /* Handle Tuya reset here */
            if ( handler->huart->ErrorCode && Tuya_Reset( handler ) == TUYA_OK )
            {
                /* Reset all uart errors */
                handler->huart->ErrorCode = HAL_UART_ERROR_NONE;

                /* Update tuya state */
                handler->eState = TUYA_STATE_IDLE;
            }
        }
        break;
    }

    return TUYA_OK;
}

static TuyaStatus_t Tuya_Handle_Command( TuyaHandler_t* handler )
{
    switch ( handler->xRxFrame.ucCommand )
    {
        case TUYA_CMD_HEARTBEAT:
        {
            const uint8_t heartbeat = Is_First_Heartbeat();

            /* Respond to heartbeat */
            Tuya_Construct_Frame( TUYA_CMD_HEARTBEAT, &heartbeat, sizeof(heartbeat), &handler->xTxFrame, BIG_ENDIAN );
        }
        break;

        case TUYA_CMD_PRODUCT_INFO:
        {
            /* Send product information */
            const char* product_info = TUYA_PRODUCT_INFO_STRING;
            Tuya_Construct_Frame( TUYA_CMD_PRODUCT_INFO, (const uint8_t *)product_info, strlen(product_info), &handler->xTxFrame, LITTLE_ENDIAN );
        }
        break;

        case TUYA_CMD_WORKING_MODE:
        {
            #ifdef  COOPERATIVE_MODE
                /* Coordinated processing mode of the MCU and the module */
                Tuya_Construct_Frame( TUYA_CMD_WORKING_MODE, NULL, 0, &handler->xTxFrame, BIG_ENDIAN );
            #else
                int16_t sConfig = ( TUYA_LED_PIN << 8 | TUYA_WIFI_RESET_PIN ) & 0xFFFF;

                /* Self-processing mode of the module */
                vConstructResponseFrame( TUYA_CMD_WORKING_MODE, (const uint8_t *)&sConfig, sizeof( sData ),
                                         &handler->xTxFrame, BIG_ENDIAN );
            #endif
        }
        break;

        case TUYA_CMD_REPORT_NETWORK_STATUS:
        {
            handler->eWifiStatus = Get_WiFi_Status( handler );

            /* Send response */
            Tuya_Construct_Frame( TUYA_CMD_REPORT_NETWORK_STATUS, NULL, 0, &handler->xTxFrame, BIG_ENDIAN );
        }
        break;

        case TUYA_CMD_RESET_WIFI:
        {
        	/* MCU gets the response */
        }
        break;

        case TUYA_CMD_SELECT_PAIRING_MODE:
        {
        	/* MCU gets the response */
        }
        break;

        case TUYA_CMD_REPORT_STATUS:
        {
            DataPointID_t eDataPointId = handler->xRxFrame.ucData[ DP_INDEX_ID ];

            if ( !Is_Data_Point_Valid( eDataPointId ) )
            {   return TUYA_ERROR;  }

            DataPoint_t *pxDataPoint = Get_Data_Point( eDataPointId, &handler->xDataPoints );

            Tuya_Construct_Frame( TUYA_CMD_REPORT_STATUS, (const uint8_t *)pxDataPoint,
                                  DATA_POINT_SIZE(pxDataPoint->ucLengthLow), &handler->xTxFrame, LITTLE_ENDIAN );
        }
        break;

        case TUYA_CMD_SEND:
        {
            DataPoint_t xDataPoint = {0};

            if ( Copy_Frame_To_Data_Point( &handler->xRxFrame, &xDataPoint ) != TUYA_OK )
            {   return TUYA_ERROR;  }

            DataPoint_t *pxDataPoint = Get_Data_Point( xDataPoint.ucID, &handler->xDataPoints );

            /* Set data point value with income data */
            if ( Copy_Data_To_Data_Point( xDataPoint.ucValue, pxDataPoint ) != TUYA_OK )
            {   return TUYA_ERROR;  }

            /* Prepare Response Frame */
            Tuya_Construct_Frame( TUYA_CMD_REPORT_STATUS, (const uint8_t *)pxDataPoint, DATA_POINT_SIZE(pxDataPoint->ucLengthLow), &handler->xTxFrame, LITTLE_ENDIAN );
        }
        break;

        case TUYA_CMD_QUERY_WORKING_STATUS:
        {
            uint8_t ucDataPointBuffer[ TUYA_UART_BUFFER_SIZE ] = { 0 };

            /* Copy Data Points Data */
            int16_t sDataSize = Copy_Data_Points_To_Buffer( ucDataPointBuffer, &handler->xDataPoints );

            /* Prepare Response Frame */
            Tuya_Construct_Frame( TUYA_CMD_REPORT_STATUS, (const uint8_t *)ucDataPointBuffer, sDataSize, &handler->xTxFrame, LITTLE_ENDIAN );

            /* Update status flag */
            handler->xStatusFlags.Bits.STARTUP_COMPLETE = true;
        }
        break;

        default:
            break;
    }

    return TUYA_OK;
}

static TuyaStatus_t Tuya_Handle_Data_Point( TuyaHandler_t* handler, DataPointID_t DataPointId )
{
    if ( !Is_Data_Point_Valid( DataPointId ) )
    {   return TUYA_ERROR;  }

    DataPoint_t *pxDataPoint = Get_Data_Point( DataPointId, &handler->xDataPoints );

    Tuya_Construct_Frame( TUYA_CMD_REPORT_STATUS, (const uint8_t *)pxDataPoint,
                          DATA_POINT_SIZE(pxDataPoint->ucLengthLow), &handler->xTxFrame, LITTLE_ENDIAN );

    return TUYA_OK;
}

static TuyaStatus_t Tuya_Handle_Transmission( TuyaHandler_t* handler )
{
	/* Check if transmission is complete */
	if ( !handler->isDataSent )
	{	return TUYA_BUSY;	}

	/* Copy frame data to tx buffer */
	handler->sTxCount = Tuya_Copy_Frame_Data( handler->ucTxBuffer, &handler->xTxFrame );

    /* Start transmission */
    HAL_UART_Transmit_DMA( handler->huart, handler->ucTxBuffer, handler->sTxCount );

    /* Update transmission flag */
    xTuyaHandler.isDataSent = 0;

    return TUYA_OK;
}

static void Tuya_Construct_Frame( TuyaCommand_t eCommand, const uint8_t* pucData, uint16_t usLength,
                                  TuyaFrame_t *pxFrame, Endianness_t eEndianType  )
{
    pxFrame->usHeader      = TUYA_FRAME_HEADER;
    pxFrame->ucVersion     = TUYA_FIRMWARE_VERSION;
    pxFrame->ucCommand     = eCommand;
    pxFrame->usDataLength  = usLength;

    for ( int16_t i = 0; i < pxFrame->usDataLength; i++ )
    {
        if ( eEndianType == BIG_ENDIAN )
        {	pxFrame->ucData[i] = pucData[ pxFrame->usDataLength - i - 1 ];	}		/* Big Endian Order */
        else
        {	pxFrame->ucData[i] = pucData[i];	}									/* Little Endian Order */
    }

    pxFrame->ucChecksum    = Tuya_Calculate_Checksum( pxFrame );
}

static uint8_t Tuya_Calculate_Checksum( TuyaFrame_t *pxFrame )
{
    uint32_t  ulSum = 0;

    ulSum += (uint8_t)( pxFrame->usHeader );			/* Header Bytes should be sum as low and high bytes */
    ulSum += (uint8_t)( pxFrame->usHeader >> 8 );
    ulSum += pxFrame->ucVersion;
    ulSum += pxFrame->ucCommand;
    ulSum += (uint8_t)( pxFrame->usDataLength );
    ulSum += (uint8_t)( pxFrame->usDataLength >> 8 );

    for ( int16_t i = 0; i < pxFrame->usDataLength; i++ )
    {	ulSum += pxFrame->ucData[i];	}

    return ( ulSum % 256 );
}

static TuyaStatus_t Tuya_Frame_RxFifo_Push( TuyaHandler_t* handler, TuyaFrame_t* frame )
{
    if ( handler->xRxFIFO.sCount >= TUYA_RX_FIFO_SIZE )
    {
        return TUYA_ERROR;  /* FIFO full */
    }

    memcpy( &handler->xRxFIFO.xFrameBuffer[ handler->xRxFIFO.sCount++ ], frame, sizeof( TuyaFrame_t ) );

    return TUYA_OK;
}

static TuyaStatus_t Tuya_Frame_RxFifo_Pop( TuyaHandler_t* handler, TuyaFrame_t* frame )
{
    if ( handler->xRxFIFO.sCount == 0 )
    {
        return TUYA_ERROR;  /* FIFO empty */
    }

    memcpy( frame, &handler->xRxFIFO.xFrameBuffer[ handler->xRxFIFO.sHead ], sizeof( TuyaFrame_t ) );
    handler->xRxFIFO.sCount--;      /* Decrease Frame Counter */

    /* Move remaining bytes to start of buffer */
    memmove( &handler->xRxFIFO.xFrameBuffer[handler->xRxFIFO.sHead], &handler->xRxFIFO.xFrameBuffer[handler->xRxFIFO.sHead + 1],
             ( TUYA_RX_FIFO_SIZE - 1 ) * sizeof(TuyaFrame_t) );

    return TUYA_OK;
}

static TuyaStatus_t Tuya_Data_TxFifo_Push( TuyaTxFIFO_t* pxFifo, const DataPointID_t DataPointID )
{
    if ( pxFifo->sCount >= TUYA_TX_FIFO_SIZE )
    {
        return TUYA_ERROR;  /* FIFO full */
    }

    /* Push data point to fifo */
    pxFifo->xDataPoints[pxFifo->sCount++] = DataPointID;

	return TUYA_OK;
}

static TuyaStatus_t Tuya_Data_TxFifo_Pop( TuyaTxFIFO_t* pxFifo, DataPointID_t* pDataPointID )
{
    if ( pxFifo->sCount == 0 )
    {
        return TUYA_ERROR;  /* FIFO empty */
    }

    /* Copy first data point id */
    memcpy( pDataPointID, &pxFifo->xDataPoints[ pxFifo->sHead ], sizeof( DataPointID_t ) );

    /* Decrease data counter */
    pxFifo->sCount--;

    /* Pop data point from fifo */
    memmove( &pxFifo->xDataPoints[ pxFifo->sHead ], &pxFifo->xDataPoints[ pxFifo->sHead + 1 ], ( TUYA_TX_FIFO_SIZE - 1 ) * sizeof( DataPointID_t ) );

	return TUYA_OK;
}

static DataPoint_t* Get_Data_Point( DataPointID_t xDataPointID, TuyaDataPoints_t *pxTuyaDataPoints )
{
    DataPoint_t *pDataPoint = ( DataPoint_t * )pxTuyaDataPoints;

    for ( int16_t i = 0; i < DATA_POINT_COUNT; ++i )
    {
        if ( pDataPoint[i].ucID == xDataPointID )
        {	return &pDataPoint[i];	}
    }

    return NULL;
}

static TuyaStatus_t Copy_Frame_To_Data_Point( TuyaFrame_t *pxTuyaFrameRx, DataPoint_t *pxDataPoint )
{
    if ( !pxDataPoint )
    {	return TUYA_ERROR;  }

    /* Copy frame data to data point object */
    pxDataPoint->ucID     	  = pxTuyaFrameRx->ucData[ DP_INDEX_ID ];
    pxDataPoint->ucType   	  = pxTuyaFrameRx->ucData[ DP_INDEX_TYPE ];
    pxDataPoint->ucLengthHigh = pxTuyaFrameRx->ucData[ DP_INDEX_LENGTH_HB ];
    pxDataPoint->ucLengthLow  = pxTuyaFrameRx->ucData[ DP_INDEX_LENGTH_LB ];

    int16_t sDataLength = ( pxDataPoint->ucLengthHigh << 8 | pxDataPoint->ucLengthLow ) & 0xFF;

    for ( int16_t i = 0; i < sDataLength; i++ )
    {	pxDataPoint->ucValue[i] = pxTuyaFrameRx->ucData[ DP_INDEX_VALUE + i ];	}

    return TUYA_OK;
}

static TuyaStatus_t Copy_Data_To_Data_Point( const uint8_t* pucData, DataPoint_t *pxDataPoint )
{
    if ( !pxDataPoint )
    {	return TUYA_ERROR;  }

    int16_t sDataLength = ( pxDataPoint->ucLengthHigh << 8 | pxDataPoint->ucLengthLow ) & 0xFF;

    for ( int16_t i = 0; i < sDataLength; i++ )
    {	pxDataPoint->ucValue[i] = pucData[i];	}

    return TUYA_OK;
}

static int16_t Copy_Data_Points_To_Buffer( uint8_t* pucBuffer, TuyaDataPoints_t *pxTuyaDataPoints )
{
    DataPoint_t *pDataPoint = ( DataPoint_t * )pxTuyaDataPoints;
    int16_t sDataCount = 0;

    /* Copy all data points data to buffer */
    for ( int16_t i = 0; i < DATA_POINT_COUNT; ++i )
    {
        memcpy( &pucBuffer[sDataCount], &pDataPoint[i], DATA_POINT_SIZE( pDataPoint[i].ucLengthLow ) );

        /* Update data index with previous data point size */
        sDataCount += DATA_POINT_SIZE( pDataPoint[i].ucLengthLow );
    }

    return sDataCount;
}

static TuyaStatus_t Extract_Frame_Data( const uint8_t* pucData, TuyaFrame_t *pxFrame )
{
    pxFrame->usHeader      = ( pucData[ HEADER_INDEX ] << 8 ) | pucData[ HEADER_INDEX + 1 ];
    pxFrame->ucVersion     = pucData[ VERSION_INDEX ];
    pxFrame->ucCommand     = pucData[ COMMAND_INDEX ];
    pxFrame->usDataLength  = ( pucData[ DATA_LENGTH_INDEX ] << 8 ) | pucData[ DATA_LENGTH_INDEX + 1 ];

    for ( int16_t i = 0; i < pxFrame->usDataLength; i++ )
    {	pxFrame->ucData[i] = pucData[ DATA_START_INDEX + i ];	}

    pxFrame->ucChecksum   = pucData[ DATA_START_INDEX + pxFrame->usDataLength ];

    return TUYA_OK;
}

static int16_t Tuya_Copy_Frame_Data( uint8_t* pucBuffer, const TuyaFrame_t *pxFrame )
{
    /* Reset Tx counter */
    int16_t sDataCount = 0;

    pucBuffer[sDataCount++] = pxFrame->usHeader >> 8 & 0xFF; 		/* Header High byte */
    pucBuffer[sDataCount++] = pxFrame->usHeader & 0xFF; 			/* Header Low byte */
    pucBuffer[sDataCount++] = pxFrame->ucVersion;
    pucBuffer[sDataCount++] = pxFrame->ucCommand;
    pucBuffer[sDataCount++] = pxFrame->usDataLength >> 8 & 0xFF; 	 /* Data Length High byte */
    pucBuffer[sDataCount++] = pxFrame->usDataLength & 0xFF;    		 /* Data Length Low byte */

    /* Copy Data */
    memcpy( &pucBuffer[sDataCount], pxFrame->ucData, pxFrame->usDataLength );
    sDataCount += pxFrame->usDataLength;

    pucBuffer[sDataCount++] = pxFrame->ucChecksum;

    return sDataCount;
}

static _Bool Is_Data_Point_Valid( int16_t sDataPointID )
{
    return ( sDataPointID >= DP_ID_TOTAL_FORWARD_ENERGY && sDataPointID <= DP_ID_IP_ADDRESS );
}

static TuyaStatus_t Tuya_Reset_Wifi( TuyaHandler_t* handler )
{
    if ( !handler )
    {   return TUYA_ERROR;  }

    /* Send wifi reset command */
    Tuya_Construct_Frame( TUYA_CMD_RESET_WIFI, NULL, 0, &handler->xTxFrame, BIG_ENDIAN );

    /* Put Tuya in Transmit State */
    handler->eState = TUYA_STATE_TRANSMITTING;

    return TUYA_OK;
}

static WifiStatus_t Get_WiFi_Status( TuyaHandler_t *handler )
{
    return handler->xRxFrame.ucData[ handler->xRxFrame.usDataLength - 1 ];
}

static TuyaStatus_t Tuya_Select_Pairing_Mode( TuyaHandler_t* handler )
{
    if ( !handler )
    {   return TUYA_ERROR;  }

    const uint8_t PairingMode = handler->ePairingMode;

    /* Reset wifi and select pairing mode */
    Tuya_Construct_Frame( TUYA_CMD_SELECT_PAIRING_MODE, &PairingMode, sizeof(PairingMode),
                         &handler->xTxFrame, BIG_ENDIAN );

    /* Put Tuya in Transmit State */
    handler->eState = TUYA_STATE_TRANSMITTING;

	return TUYA_OK;
}

static uint8_t Is_First_Heartbeat( void )
{
    static _Bool IsFirstHeartbeat = true;

    /* Assign 0 if it's the first heartbeat; otherwise, assign 1 */
    uint8_t HeartbeatData = IsFirstHeartbeat ? 0 : 1;
    IsFirstHeartbeat = false;

    return HeartbeatData;
}

static void Tuya_Init_Data_Points( TuyaDataPoints_t *pxDataPoints )
{
    Tuya_Set_Data_Point( &pxDataPoints->xTotalForwardEnergy, DP_ID_TOTAL_FORWARD_ENERGY, DP_TYPE_VALUE, DP_SIZE_VALUE );
    Tuya_Set_Data_Point( &pxDataPoints->xWorkState, DP_ID_WORK_STATE, DP_TYPE_ENUM, DP_SIZE_ENUM );
    Tuya_Set_Data_Point( &pxDataPoints->xChargeCurrentSet, DP_ID_CHARGE_CURRENT_SET, DP_TYPE_VALUE, DP_SIZE_VALUE );
    Tuya_Set_Data_Point( &pxDataPoints->xSinglePhasePower, DP_ID_SINGLE_PHASE_POWER, DP_TYPE_VALUE, DP_SIZE_VALUE );
    Tuya_Set_Data_Point( &pxDataPoints->xPhaseA, DP_ID_PHASE_A, DP_TYPE_RAW, DP_SIZE_RAW_PHASE_DATA );
    Tuya_Set_Data_Point( &pxDataPoints->xPhaseB, DP_ID_PHASE_B, DP_TYPE_RAW, DP_SIZE_RAW_PHASE_DATA );
    Tuya_Set_Data_Point( &pxDataPoints->xPhaseC, DP_ID_PHASE_C, DP_TYPE_RAW, DP_SIZE_RAW_PHASE_DATA );
    Tuya_Set_Data_Point( &pxDataPoints->xSwitch, DP_ID_SWITCH, DP_TYPE_BOOLEAN, DP_SIZE_BOOLEAN );
    Tuya_Set_Data_Point( &pxDataPoints->xClearEnergy, DP_ID_CLEAR_ENERGY, DP_TYPE_BOOLEAN, DP_SIZE_BOOLEAN );
    Tuya_Set_Data_Point( &pxDataPoints->xOnlineState, DP_ID_ONLINE_STATE, DP_TYPE_ENUM, DP_SIZE_ENUM );
    Tuya_Set_Data_Point( &pxDataPoints->xLockGun, DP_ID_LOCK_GUN, DP_TYPE_BOOLEAN, DP_SIZE_BOOLEAN );
    Tuya_Set_Data_Point( &pxDataPoints->xCurrentTemp, DP_ID_CURRENT_TEMP, DP_TYPE_VALUE, DP_SIZE_VALUE );
    Tuya_Set_Data_Point( &pxDataPoints->xConnectionState, DP_ID_CONNECTION_STATE, DP_TYPE_ENUM, DP_SIZE_ENUM );
}

static void Tuya_Set_Data_Point( DataPoint_t *pDataPoint, uint8_t ucID, uint8_t ucType, uint8_t ucLength )
{
    pDataPoint->ucID = ucID;
    pDataPoint->ucType = ucType;
    pDataPoint->ucLengthLow = ucLength;
    pDataPoint->ucLengthHigh = 0;
}

void Tuya_Set_Registers( TuyaRegisters_t *pxTuyaRegisters )
{
    pxTuyaRegisters->lTotalForwardEnergy = ( int32_t )fEnergy_kWh;
    pxTuyaRegisters->lWorkState 		 = 4;
    pxTuyaRegisters->lIset				 = 1;	  //( int32_t )fSetCurrent;
    pxTuyaRegisters->lSinglePhasePower	 = 123;   //( int32_t )xSineAnalyzer_R.xOutput.fActivePower;
    pxTuyaRegisters->lVinR_Rms 	         = 123;   //( int32_t )xSineAnalyzer_R.xOutput.fVoltageRms;
    pxTuyaRegisters->lVinS_Rms 	         = 123;   //( int32_t )xSineAnalyzer_S.xOutput.fVoltageRms;
    pxTuyaRegisters->lVinT_Rms 	         = 123;   //( int32_t )xSineAnalyzer_T.xOutput.fVoltageRms;
    pxTuyaRegisters->lIinR_Rms 	         = 123;   //( int32_t )xSineAnalyzer_R.xOutput.fCurrentRms;
    pxTuyaRegisters->lIinS_Rms 	         = 123;   //( int32_t )xSineAnalyzer_S.xOutput.fCurrentRms;
    pxTuyaRegisters->lIinT_Rms 	         = 123;   //( int32_t )xSineAnalyzer_T.xOutput.fCurrentRms;
    pxTuyaRegisters->lActivePowerR	     = 123;   //( int32_t )xSineAnalyzer_R.xOutput.fActivePower;
    pxTuyaRegisters->lActivePowerS	 	 = 123;   //( int32_t )xSineAnalyzer_S.xOutput.fActivePower;
    pxTuyaRegisters->lActivePowerT	 	 = 123;   //( int32_t )xSineAnalyzer_T.xOutput.fActivePower;
    pxTuyaRegisters->lOnlineState 		 = 1;	  												/* Offline or Online */
    pxTuyaRegisters->lCurrentTemp		 = 50;
    pxTuyaRegisters->lControlPilotState  = ( int32_t )*pxControlPilotState;
}

void Tuya_Update_Data_Points( TuyaRegisters_t *pxTuyaRegisters, TuyaDataPoints_t *xTuyaDataPoints )
{
    DataPoint_t *pDataPoint = ( DataPoint_t * )xTuyaDataPoints;
    uint8_t ucDataBuffer[ TUYA_UART_BUFFER_SIZE ] = { 0 };
    uint64_t RegisterData = 0;

    for ( int16_t sDpIndex = 0; sDpIndex < DATA_POINT_COUNT; ++sDpIndex )
    {
        switch ( pDataPoint[sDpIndex].ucID )
        {
            case DP_ID_SWITCH:
            case DP_ID_LOCK_GUN:
            case DP_ID_CLEAR_ENERGY:
            {
                continue;				/* Holding Registers */
            }

            case  DP_ID_TOTAL_FORWARD_ENERGY:
            {
                RegisterData = pxTuyaRegisters->lTotalForwardEnergy * 100;

                memcpy( ucDataBuffer, &RegisterData, pDataPoint[sDpIndex].ucLengthLow );
            }
            break;

            case  DP_ID_WORK_STATE:
            {
                memcpy( ucDataBuffer, &pxTuyaRegisters->lWorkState, pDataPoint[sDpIndex].ucLengthLow );
            }
            break;

            case  DP_ID_CHARGE_CURRENT_SET:
            {
                memcpy( ucDataBuffer, &pxTuyaRegisters->lIset, pDataPoint[sDpIndex].ucLengthLow );
            }
            break;

            case  DP_ID_SINGLE_PHASE_POWER:
            {
                memcpy( ucDataBuffer, &pxTuyaRegisters->lSinglePhasePower, pDataPoint[sDpIndex].ucLengthLow );
            }
            break;

            case  DP_ID_PHASE_A:
            {
                RegisterData = Format_Phase_Data( pxTuyaRegisters->lVinR_Rms, pxTuyaRegisters->lIinR_Rms, pxTuyaRegisters->lActivePowerR );

                memcpy( ucDataBuffer, &RegisterData, pDataPoint[sDpIndex].ucLengthLow  );
            }
            break;

            case  DP_ID_PHASE_B:
            {
                RegisterData = Format_Phase_Data( pxTuyaRegisters->lVinS_Rms, pxTuyaRegisters->lIinS_Rms, pxTuyaRegisters->lActivePowerS );

                memcpy( ucDataBuffer, &RegisterData, pDataPoint[sDpIndex].ucLengthLow  );
            }
            break;

            case  DP_ID_PHASE_C:
            {
                RegisterData = Format_Phase_Data( pxTuyaRegisters->lVinT_Rms, pxTuyaRegisters->lIinT_Rms, pxTuyaRegisters->lActivePowerT );

                memcpy( ucDataBuffer, &RegisterData, pDataPoint[sDpIndex].ucLengthLow );
            }
            break;

            case  DP_ID_ONLINE_STATE:
            {
                memcpy( ucDataBuffer, &pxTuyaRegisters->lOnlineState, pDataPoint[sDpIndex].ucLengthLow );
            }
            break;

            case  DP_ID_CURRENT_TEMP:
            {
                memcpy( ucDataBuffer, &pxTuyaRegisters->lCurrentTemp, pDataPoint[sDpIndex].ucLengthLow );
            }
            break;

            case  DP_ID_CONNECTION_STATE:
            {
                memcpy( ucDataBuffer, &pxTuyaRegisters->lControlPilotState, pDataPoint[sDpIndex].ucLengthLow );
            }
            break;

            default: break;
        }

        /* Copy Registers Data to Data Points */
        Update_Data_Point_Value( &pDataPoint[sDpIndex], ucDataBuffer, pDataPoint[sDpIndex].ucLengthLow );
    }
}

static uint64_t Format_Phase_Data( uint32_t voltage, uint32_t current, uint32_t power )
{
    return (( uint64_t )( ( voltage * 10 ) & 0xFFFF) << 48)       |
           (( uint64_t )( ( current * 1000 ) & 0xFFFFFF ) << 24 ) |
           (( power * 1000 ) & 0xFFFFFF );
}

static void Update_Data_Point_Value( DataPoint_t *pDataPoint, uint8_t *pucData, uint8_t length )
{
    for ( int16_t i = 0; i < length; ++i )
    {
        pDataPoint->ucValue[i] = pucData[length - 1 - i];
    }
}

void Tuya_Report_Data( TuyaHandler_t* handler )
{
	if ( handler->xTxFIFO.sCount == 0  && handler->xStatusFlags.Bits.STARTUP_COMPLETE )		/* Tx Fifo empty condition */
	{
		Tuya_Data_TxFifo_Push( &handler->xTxFIFO, DP_ID_PHASE_A );
		Tuya_Data_TxFifo_Push( &handler->xTxFIFO, DP_ID_PHASE_B );
		Tuya_Data_TxFifo_Push( &handler->xTxFIFO, DP_ID_PHASE_C );
		Tuya_Data_TxFifo_Push( &handler->xTxFIFO, DP_ID_TOTAL_FORWARD_ENERGY );
		Tuya_Data_TxFifo_Push( &handler->xTxFIFO, DP_ID_CURRENT_TEMP );
		Tuya_Data_TxFifo_Push( &handler->xTxFIFO, DP_ID_CONNECTION_STATE );
	}
}

/* ---------------------------- Driver Layer Functions ----------------------------- */

void HAL_UARTEx_RxEventCallback( UART_HandleTypeDef *huart, uint16_t Size )
{
    if ( huart == xTuyaHandler.huart )
    {
        /* ------------------- Handling DMA Data -------------------- */
        if ( Size >= TUYA_MINIMUM_FRAME_SIZE && Size < TUYA_UART_BUFFER_SIZE )		/* Check minimum frame size */
        {
            /* Get received data size */
            xTuyaHandler.sRxCount = Size;

            /* Copy data to created frame */
            TuyaFrame_t xFrame = {0};
            Extract_Frame_Data( xTuyaHandler.ucRxBuffer, &xFrame );

            /* Check Frame Header and verify checksum  */
            if ( xFrame.usHeader == TUYA_FRAME_HEADER  &&
            	 xFrame.ucChecksum == Tuya_Calculate_Checksum( &xFrame ) )
            {
				/* Push frame to FIFO */
				Tuya_Frame_RxFifo_Push( &xTuyaHandler, &xFrame );
            }
        }

        /* Clear receive buffer */
        memset( xTuyaHandler.ucRxBuffer, 0, sizeof( xTuyaHandler.ucRxBuffer ) );
        /* ---------------------------------------------------------- */

        /* Restart DMA reception */
        HAL_UARTEx_ReceiveToIdle_DMA( xTuyaHandler.huart, xTuyaHandler.ucRxBuffer, sizeof( xTuyaHandler.ucRxBuffer ) );
    }
}

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart )
{
	if ( huart == xTuyaHandler.huart )
	{
		xTuyaHandler.isDataSent = true;

	    /* Clear Tx buffer */
	    memset( xTuyaHandler.ucTxBuffer, 0, sizeof(xTuyaHandler.ucTxBuffer) );
	}
}

/* DMA Error Callback */
void HAL_UART_ErrorCallback( UART_HandleTypeDef *huart )
{
    if ( huart == xTuyaHandler.huart )
    {
        xTuyaHandler.ErrorCount++;
        xTuyaHandler.LastError = huart->ErrorCode;
    }
}

