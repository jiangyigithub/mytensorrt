/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#ifndef XCP_COMMANDS_H
#define XCP_COMMANDS_H

/***************************************************************************/
/* Commands                                                                */
/***************************************************************************/

/*-------------------------------------------------------------------------*/
/* Standard Commands */

#define CC_CONNECT 0xFF
#define CC_DISCONNECT 0xFE
#define CC_GET_STATUS 0xFD
#define CC_SYNC 0xFC

#define CC_GET_COMM_MODE_INFO 0xFB
#define CC_GET_ID 0xFA
#define CC_SET_REQUEST 0xF9
#define CC_GET_SEED 0xF8
#define CC_UNLOCK 0xF7
#define CC_SET_MTA 0xF6
#define CC_UPLOAD 0xF5
#define CC_SHORT_UPLOAD 0xF4
#define CC_BUILD_CHECKSUM 0xF3

#define CC_TRANSPORT_LAYER_CMD 0xF2
#define CC_USER_CMD 0xF1

/*-------------------------------------------------------------------------*/
/* Calibration Commands*/

#define CC_DOWNLOAD 0xF0

#define CC_DOWNLOAD_NEXT 0xEF
#define CC_DOWNLOAD_MAX 0xEE
#define CC_SHORT_DOWNLOAD 0xED
#define CC_MODIFY_BITS 0xEC

/*-------------------------------------------------------------------------*/
/* Page switching Commands (PAG) */

#define CC_SET_CAL_PAGE 0xEB
#define CC_GET_CAL_PAGE 0xEA

#define CC_GET_PAG_PROCESSOR_INFO 0xE9
#define CC_GET_SEGMENT_INFO 0xE8
#define CC_GET_PAGE_INFO 0xE7
#define CC_SET_SEGMENT_MODE 0xE6
#define CC_GET_SEGMENT_MODE 0xE5
#define CC_COPY_CAL_PAGE 0xE4

/*-------------------------------------------------------------------------*/
/* DATA Acquisition and Stimulation Commands (DAQ/STIM) */

#define CC_CLEAR_DAQ_LIST 0xE3
#define CC_SET_DAQ_PTR 0xE2
#define CC_WRITE_DAQ 0xE1
#define CC_SET_DAQ_LIST_MODE 0xE0
#define CC_GET_DAQ_LIST_MODE 0xDF
#define CC_START_STOP_DAQ_LIST 0xDE
#define CC_START_STOP_SYNCH 0xDD

#define CC_GET_DAQ_CLOCK 0xDC
#define CC_READ_DAQ 0xDB
#define CC_GET_DAQ_PROCESSOR_INFO 0xDA
#define CC_GET_DAQ_RESOLUTION_INFO 0xD9
#define CC_GET_DAQ_LIST_INFO 0xD8
#define CC_GET_DAQ_EVENT_INFO 0xD7

#define CC_FREE_DAQ 0xD6
#define CC_ALLOC_DAQ 0xD5
#define CC_ALLOC_ODT 0xD4
#define CC_ALLOC_ODT_ENTRY 0xD3

/*-------------------------------------------------------------------------*/
/* Non volatile memory Programming Commands PGM */

#define CC_PROGRAM_START 0xD2
#define CC_PROGRAM_CLEAR 0xD1
#define CC_PROGRAM 0xD0
#define CC_PROGRAM_RESET 0xCF

#define CC_GET_PGM_PROCESSOR_INFO 0xCE
#define CC_GET_SECTOR_INFO 0xCD
#define CC_PROGRAM_PREPARE 0xCC
#define CC_PROGRAM_FORMAT 0xCB
#define CC_PROGRAM_NEXT 0xCA
#define CC_PROGRAM_MAX 0xC9
#define CC_PROGRAM_VERIFY 0xC8

/*-------------------------------------------------------------------------*/
/* Customer specific commands */

#define CC_WRITE_DAQ_MULTIPLE 0x81

/*-------------------------------------------------------------------------*/
/* Packet Identifiers Slave -> Master */
#define PID_RES 0xFF  /* response packet        */
#define PID_ERR 0xFE  /* error packet           */
#define PID_EV 0xFD   /* event packet           */
#define PID_SERV 0xFC /* service request packet */

/*-------------------------------------------------------------------------*/
/* Command Return Codes */

#define CRC_CMD_SYNCH 0x00

#define CRC_CMD_BUSY 0x10
#define CRC_DAQ_ACTIVE 0x11
#define CRC_PRM_ACTIVE 0x12

#define CRC_CMD_UNKNOWN 0x20
#define CRC_CMD_SYNTAX 0x21
#define CRC_OUT_OF_RANGE 0x22
#define CRC_WRITE_PROTECTED 0x23
#define CRC_ACCESS_DENIED 0x24
#define CRC_ACCESS_LOCKED 0x25
#define CRC_PAGE_NOT_VALID 0x26
#define CRC_PAGE_MODE_NOT_VALID 0x27
#define CRC_SEGMENT_NOT_VALID 0x28
#define CRC_SEQUENCE 0x29
#define CRC_DAQ_CONDIF 0x2A

#define CRC_MEMORY_OVERFLOW 0x30
#define CRC_GENERIC 0x31
#define CRC_VERIFY 0x32

/*-------------------------------------------------------------------------*/
/* Event Codes */

#define EVC_RESUME_MODE 0x00
#define EVC_CLEAR_DAQ 0x01
#define EVC_STORE_DAQ 0x02
#define EVC_STORE_CAL 0x03
#define EVC_CMD_PENDING 0x05
#define EVC_DAQ_OVERLOAD 0x06
#define EVC_SESSION_TERMINATED 0x07
#define EVC_USER 0xFE
#define EVC_TRANSPORT 0xFF

/*-------------------------------------------------------------------------*/
/* Service Request Codes */

#define SERV_RESET 0x00 /* Slave requesting to be reset */
#define SERV_TEXT 0x01  /* Plain ASCII text null terminated */

#endif
