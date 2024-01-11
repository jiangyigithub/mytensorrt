/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#ifndef XCP_H
#define XCP_H

#include "radar_manager_ros/internal/communication/common.h"
#include "radar_manager_ros/internal/communication/tcp_socket.h"
#include "radar_manager_ros/internal/communication/xcp_commands.h"

// max number of CTO parameters
#define MAX_CTO 10

#define MAX_ODT_ENTRIES 217
#define XCP_MAX_DATA_SIZE_DTO 1448
#define XCP_MAX_DATA_SIZE_CTO 256  // 256 needed as maximum response for a XCP shortupload querry
/*-------------------------------------------------------------------------*/
/* Packet definitions */

// XCP header used for XCP on TCP/IP
typedef struct tcp_header
{
  // Length field, number of bytes in the XCP packet
  uint16_t len;  // 2
  // Counter field, Master has to generate a CTR value and increment it of each new packet sent to
  // the slave
  uint16_t ctr;  // 4
} tcp_header;

typedef struct xcp_id_field
{
  uint16_t pid;
  uint16_t fill;
  uint32_t daq;
} xcp_id_field;

/*-------------------------DTO specific -------------------------------------*/
// the first message for one DAQ contains the timestamp.
typedef struct xcp_message_dto
{
  tcp_header header;    // 4
  uint8_t pid;          // 5
  uint8_t dto_list_id;  // 6 // attention: according to ASAP, dto list id can be 1 byte, 1 word, or
                        // one blank and 1 word
  uint8_t unused3;      // 7
  uint8_t unused4;      // 8
  uint8_t timestamp_byte1;  // 9
  uint8_t timestamp_byte2;  // 10
  uint8_t timestamp_byte3;  // 11
  uint8_t timestamp_byte4;  // 12
  uint8_t data[XCP_MAX_DATA_SIZE_DTO];
} xcp_message_dto;

/*-------------------------CTO specific -------------------------------------*/

/*
Structure for a XCP CTO message.
The size of the data field can vary
*/
typedef struct xcp_message_cto
{
  tcp_header header;
  uint8_t pid;
  uint8_t data[XCP_MAX_DATA_SIZE_CTO];
} xcp_message_cto;

typedef struct xcp_message_cto_list
{
  xcp_message_cto** cto_message_list;
  int nb_of_elements;
} xcp_message_cto_list;

/* Command specific : data field of particular commands */  // todo : fix padding
typedef struct cto_RES_data
{
  uint8_t id;  // 0xff // todo : delete ?
  uint8_t resource;
  uint8_t comm_mode_basic;
  uint8_t max_cto;
  uint16_t max_dto;
  uint8_t xcp_prot_layer_version;
  uint8_t xcp_transport_layer_version;
} cto_RES_data;

typedef struct cto_alloc_odt_entry_data
{
  uint8_t reserved;
  uint8_t daq_list_numberbyte1;  // to avoid padding
  uint8_t daq_list_numberbyte2;
  uint8_t odt_number;
  uint8_t odt_entries_count;
} cto_alloc_odt_entry_data;

typedef struct cto_set_daq_pointer_data
{
  uint8_t reserved;
  uint8_t daq_list_numberbyte1;  // to avoid padding
  uint8_t daq_list_numberbyte2;
  uint8_t odt_number;
  uint8_t odt_entry_number;
} cto_set_daq_pointer_data;

typedef struct cto_write_daq_data
{
  uint8_t bit_offset;
  uint8_t daq_elem_size;
  uint8_t daq_elem_addr_ext;
  uint8_t daq_elem_addr_byte1;
  uint8_t daq_elem_addr_byte2;
  uint8_t daq_elem_addr_byte3;
  uint8_t daq_elem_addr_byte4;
} cto_write_daq_data;

/*
typedef struct cto_write_daq_data_vector
{
    uint8_t bit_offset;
    uint8_t daq_elem_size;
    uint8_t daq_elem_addr_ext;
    uint8_t daq_elem_addr_byte1;
    uint8_t daq_elem_addr_byte2;
    uint8_t daq_elem_addr_byte3;
    uint8_t daq_elem_addr_byte4;
}cto_write_daq_data_vector;
*/

typedef struct cto_set_daq_list_mode_data
{
  uint8_t mode;
  uint8_t daq_list_numberbyte1;
  uint8_t daq_list_numberbyte2;
  uint8_t event_chan_nb_numberbyte1;
  uint8_t event_chan_nb_numberbyte2;
  uint8_t trans_rate_prescaler;
  uint8_t daq_list_priority;
} cto_set_daq_list_mode_data;

typedef struct cto_start_stop_daq_list_data
{
  uint8_t mode;
  uint8_t daq_list_numberbyte1;
  uint8_t daq_list_numberbyte2;
} cto_start_stop_daq_list_data;

typedef enum state_enum
{
  FATAL_STATE = -2,
  ERROR_STATE = -1,
  INIT_STATE = 0,
  CONNECTED_STATE,
  STATUS_WAIT_RES,
  GOT_STATUS_STATE,
  GOT_DAQ_PROCESSOR_INFO_STATE,
  GOT_DAQ_RESOLUTION_INFO_STATE,
  SENT_FREE_DAQ_STATE,
  ALLOC_DAQ_STATE,
  ALLOC_ODT_STATE,
  ALLOC_ODT_STATE2,
  DAQ_WRITE_STATE,
  DAQ_WRITE_STATE2,
  DAQ_WRITE_STATE3,
  DAQ_WRITE_STATE3_POINTER,
  DAQ_WRITE_STATE4,
  SET_DAQ_LIST_MODE_STATE,
  START_STOP_DAQ_LIST_STATE,
  START_STOP_DAQ_LIST_STATE1,
  START_STOP_DAQ_LIST_STATE2,
  GET_DAQ_CLOCK_STATE,
  DATA_AQUISITION_STATE_WAITRES,
  DATA_AQUISITION_STATE
} stage_enum;

/*-------------------------------------------------------------------------*/
/*Function prototypes */
int sendCtoMessage(uint8_t command, uint8_t* data, int sizeofdata, int socket);
xcp_message_cto* receiveCtoMessage(int SocketFD, size_t cto_size);
void freeCtoMessage(xcp_message_cto** cto_message_pp);
xcp_message_cto* mapToCtoMessage(unsigned char buffer[]);
void printCtoInfo(xcp_message_cto* message);
char* getPidInfo(int pid);
// void printDtoInfo(xcp_message_dto* message);
uint32_t printDtoInfo(xcp_message_dto* message); //lnl, timer

#endif
