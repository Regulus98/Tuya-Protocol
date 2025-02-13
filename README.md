# Tuya Protocol Implementation for Embedded Systems

## Introduction  
This repository contains an **embedded implementation of the Tuya Protocol**, enabling seamless communication between IoT devices and the Tuya Cloud or local gateways. The implementation supports **UART-based** data exchange, allowing devices to be controlled via **Wi-Fi or Bluetooth** using Tuya’s proprietary communication framework.

## What is the Tuya Protocol?  
The **Tuya Protocol** is a standardized communication protocol designed for **smart home and IoT devices**, allowing them to exchange data with a Tuya-enabled gateway or cloud. It operates over **Wi-Fi, Bluetooth, or Zigbee**, ensuring reliable device control, status reporting, and OTA firmware updates.

---

## How Tuya Protocol Works  
Tuya-enabled embedded devices communicate via a structured message format over serial communication (UART). The protocol allows devices to:  
- **Send device status updates** (e.g., temperature, power state).  
- **Receive control commands** (e.g., turn on/off, change settings).  
- **Perform OTA firmware updates** securely.  
- **Maintain connectivity** through a heartbeat mechanism.  

Devices using **Wi-Fi** connect to the Tuya Cloud via an internet-enabled Tuya gateway. **Bluetooth**-enabled devices communicate with a Tuya app or bridge module, which relays data to the cloud.

---

## Protocol Structure  
Each Tuya message follows a fixed frame structure to ensure data integrity and consistency.  

### Frame Format
| Field       | Size (Bytes) | Description |
|------------|------------|-------------|
| Header     | 2          | Fixed (`0x55AA`) indicating the start of a message |
| Version    | 1          | Protocol version |
| Command Type | 1        | Defines the purpose of the frame (e.g., control, query, response) |
| Data Length | 2         | Length of the data payload |
| Data       | Variable   | Actual message content (e.g., status updates, sensor data) |
| Checksum   | 1          | Error detection field (sum of all previous bytes) |

### Key Commands
| Command Type | Value  | Description |
|-------------|--------|-------------|
| `HEARTBEAT` | `0x00` | Keeps the device connected |
| `STATUS REPORT` | `0x07` | Sends device state to the cloud/gateway |
| `CONTROL` | `0x06` | Receives control commands |
| `QUERY STATUS` | `0x08` | Requests latest device status |
| `OTA UPDATE` | `0x0A` | Firmware update command |

---

## Implementation Details  
This implementation includes the following key files:

### **Driver Implementation**
- **`TuyaDriver.c`** – Contains the main Tuya protocol handling logic, including **FIFO-based message receiving and sending**.  
- **`TuyaDriver.h`** – Header file defining Tuya protocol functions, structures, and data types.  

### **Configuration File**
- **`TuyaConfig.h`** – A configuration file where users can:  
  - Adjust **Tuya module settings** (e.g., **Product ID**, **operation mode**).  
  - Configure **Tuya UART settings** such as **UART handler** and **baud rate**.

---

## Main Features of This Implementation  
✔ **UART-based Tuya Protocol Parser** for STM32 or other MCUs.  
✔ **Reliable Message Processing** using checksum validation.  
✔ **Heartbeat Mechanism** to maintain cloud connectivity.  
✔ **Status Reporting** based on device **Data Points (DPID)**.  
✔ **OTA Firmware Update Handling** for remote updates.  
✔ **Wi-Fi & Bluetooth Communication** support through Tuya modules.  

---

## How to Use  
1. **Initialize UART Communication** on the embedded device.  
2. **Integrate the Tuya Parser** to decode incoming messages.  
3. **Implement the Command Handler** to process Tuya control commands.  
4. **Enable Status Reporting** to send device state periodically.  
5. **Test with Tuya Cloud or App** to verify communication.  

---

## Security Considerations  
- **Checksum validation** prevents corrupted data processing.   
- **Device authentication** ensures only trusted devices communicate.  

---

## Conclusion  
This implementation provides a **robust, embedded-friendly Tuya Protocol stack**, enabling **efficient IoT device communication** over **Wi-Fi & Bluetooth**. By integrating this with your MCU-based project, you can seamlessly connect your embedded system to the Tuya ecosystem.

