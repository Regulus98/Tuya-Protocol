/*
 * TuyaConfig.h
 *
 *  Created on: Dec 26, 2024
 *      Author: 26081178
 */

#ifndef SRC_APPLICATIONLAYER_TUYACONFIG_H_
#define SRC_APPLICATIONLAYER_TUYACONFIG_H_

#define TUYA_FRAME_HEADER			0x55aa
#define TUYA_FIRMWARE_VERSION		0x03

/* Product Information */
#define TUYA_PRODUCT_ID            "kyvcaudox4u89rzm"  /* Your Tuya product ID */
#define TUYA_MCU_VERSION       	   "1.0.0"
#define TUYA_OPERATION_MODE    	   "0"
#define TUYA_PRODUCT_INFO_STRING   "{\"p\":\""TUYA_PRODUCT_ID"\",\"v\":\""TUYA_MCU_VERSION"\",\"m\":"TUYA_OPERATION_MODE"}"

/* Communication Settings */
#define TUYA_UART_HANDLER        	&huart2				/* Uart handler address */
#define TUYA_UART_BAUDRATE       	9600                /* UART baud rate */
#define TUYA_HEARTBEAT_INTERVAL  	10000               /* Heartbeat interval in ms */

/* Working Mode */
#define COOPERATIVE_MODE
//#define SELF_MODE

/* Pairing Configuration Mode */
#define TUYA_SMARTCONFIG_MODE   	0x00
#define TUYA_AP_MODE            	0x01

/* GPIO Configuration */
#define TUYA_LED_PIN             	0x10     /* Led indication pin - GPIO_16 */
#define TUYA_WIFI_RESET_PIN      	0x1C     /* Wifi reset pin - GPIO_28 */

/* Debug Configuration */
#define TUYA_DEBUG_ENABLE        	1
#define TUYA_DEBUG_UART          	&huart2  	/* Debug UART handle */

#if TUYA_DEBUG_ENABLE
#define TUYA_DEBUG(fmt, ...)     	printf("[TUYA] " fmt "\r\n", ##__VA_ARGS__)
#else
#define TUYA_DEBUG(fmt, ...)
#endif

#endif /* SRC_APPLICATIONLAYER_TUYACONFIG_H_ */
