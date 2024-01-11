/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include "radar_manager_ros/internal/communication/xcp.h"
#include "radar_manager_ros/internal/communication/xcp_commands.h"
#include "radar_manager_ros/internal/communication/tcp_socket.h"

#include <stdio.h>

// #define DUMP_DATA_MODE

int sendCtoMessage(uint8_t command, uint8_t *data, int sizeofdata, int socket)
{
	tcp_header header;
	// header len in network byte order

	static uint16_t sendCounter = 0;
	header.len = sizeofdata + 1;
	header.ctr = 0;
	sendCounter++;

	xcp_message_cto message;

	message.header = header;
	message.pid = command;

	if (sizeofdata)
		memcpy(message.data, (void *)data, sizeofdata);

	int sizeofmessage = sizeof(header) + sizeof(command) + sizeofdata;

#ifdef DUMP_DATA_MODE
	int i;

	char *buf = (char *)&message;

	fprintf(stderr, "preparing to send message : \n");
	for (i = 0; i < ((int)sizeof(header) + (int)sizeof(command) + sizeofdata); i++)
	{
		fprintf(stderr, "%02X:", (unsigned char)buf[i]);
	}
	fprintf(stderr, "\n data field : \n");
	for (i = 0; i < (sizeofdata); i++)
	{
		fprintf(stderr, "%02X:", (unsigned char)data[i]);
	}
	fprintf(stderr, "\n");
#endif

	int tmp = write(socket, &message, sizeofmessage);

	tmp = tmp; //prevent compiler warning on unused variable. This gets optimised out.

#ifdef DUMP_DATA_MODE
	fprintf(stderr, "wrote %i bytes to socket\n", tmp);
#endif

	return 0;
}

void freeCtoMessage(xcp_message_cto **cto_message_pp)
{
	// input: pointer to a cto-message-pointer
	//        (needed to null the cto-message-pointer, which had no effect in the old version)

	if (*cto_message_pp != NULL)
	{
		free(*cto_message_pp);
		*cto_message_pp = NULL;
	}
	return;
}

xcp_message_cto *mapToCtoMessage(unsigned char buffer[])
{

	int len = (int)buffer[0] + 4;
	int msglen = sizeof(xcp_message_cto) + len;

	xcp_message_cto *message = malloc(msglen);

	memcpy((void *)message, (void *)&buffer[0], msglen);

	return message;
}

xcp_message_cto *receiveCtoMessage(int SocketFD, size_t cto_size)
{
	if (0 == cto_size)
	{
		printf("Error: receiveCtoMessage() - max cto size not initialized.\n");
		return NULL;
	}

	// Read the first byte (length) and calculate frame length accordingly
	uint8_t s = 0;
	ssize_t rd = recv(SocketFD, &s, 1, MSG_PEEK); // read the first Byte but leave it in the queue

	if (rd > 0) // has something been read
	{
		int size = s + 4; // real size including the xcp header for ethernet
		//if( size > cto_size )
		//{
		//    printf("Error: receiveCtoMessage() - current data size (%d) exceeds max_cto_size (%d).\n", (int) size, (int) cto_size);
		//    return NULL;
		//}

		uint8_t buf[size];
		memset(buf, 0, size);

		// read the the frame
		rd = recv(SocketFD, buf, size, MSG_WAITALL);
		//printf("read %i of %i bytes from socket (buffer size is %i)\n", (int)rd, size, (int) cto_size);

		return mapToCtoMessage(buf);
	}
	else
	{
		return NULL;
	}
}

char *getPidInfo(int pid)
{
	switch (pid)
	{
	case 0xFF:
		return "RES";
		break;
	case 0xFD:
		return "EV";
		break;
	case 0xFC:
		return "SERV";
		break;
	case 0xFE:
		return "ERROR";
	default:
		return "UNKNOWN";
		break;
	}
}

// void printDtoInfo(xcp_message_dto *message) 
uint32_t printDtoInfo(xcp_message_dto *message) //lnl, timer
{
	if (!message)
	{
		printf("error, message null\n");
		// return;
		return 0;
	}
	fprintf(stderr, "xcp DTO message information :\n");
	fprintf(stderr, "	HEADER: len : %i\n		ctr : %i\n		pid : %i\n", message->header.len, message->header.ctr, message->pid);

	//lnl, timer
	// if (message->pid == 0) //print timestamp only for the first message
	// {
		// todo: use byte order according to sensor settings
	int timestamp_byte1 = (int)message->timestamp_byte1;
	int timestamp_byte2 = (int)message->timestamp_byte2;
	int timestamp_byte3 = (int)message->timestamp_byte3;
	int timestamp_byte4 = (int)message->timestamp_byte4;

		/*uint32_t timestamp = timestamp_byte4 + (timestamp_byte3 << 8)
		 + (timestamp_byte2 << 16)  + (timestamp_byte1 << 24);
        */
		// gen5: reverse timestamp gives valid time
	uint32_t timestamp = timestamp_byte1 + (timestamp_byte2 << 8) + (timestamp_byte3 << 16) + (timestamp_byte4 << 24);

	if (message->pid == 0) //print timestamp only for the first message
	{
		int s = (timestamp / 1000000); // valid for time stamp unit 1us = 10^-6sec --> see timestamp mode
		int h = s / 3600;
		s = s - (h * 3600);
		int m = s / 60;
		s = (s - (m * 60));

		fprintf(stderr, "		timestamp : %04X ( %i h, %i min, %i s )\n", timestamp, h, m, s);

	}

#ifdef DUMP_DATA_MODE
	int i;
	uint8_t *msg = (uint8_t *)message;
	for (i = 0; (i < (message->header.len + 4)); i++)
	{
		fprintf(stderr, "%02X:", msg[i]);
		if (((i % 32) == 0) && (i > 0))
			fprintf(stderr, " ----- %i\n", i);
	}
	fprintf(stderr, "\n");
#endif
	//lnl, timer
	return timestamp;
	// return;
}

void printCtoInfo(xcp_message_cto *message)
{
	if (!message)
	{
		fprintf(stderr, "error, message null\n");
		return;
	}
#ifdef DUMP_DATA_MODE
	fprintf(stderr, "xcp CTO message information :\n");
	fprintf(stderr, "	HEADER: len : %i\n		ctr : %i\n", message->header.len, message->header.ctr);
	fprintf(stderr, "	FRAME : pid : 0x%02X ( %s ) \n", message->pid, getPidInfo(message->pid));
#endif
	if ((message->header.len) > 1)
	{

#ifdef DUMP_DATA_MODE //do not print normal operation.
		int j;
		fprintf(stderr, "		data: ");
		for (j = 0; j < (message->header.len); j++)
		{
			fprintf(stderr, "%02X", message->data[j]);
			if (j == message->header.len - 2)
			{
				fprintf(stderr, "\n");
			}
			else
			{
				fprintf(stderr, ":");
			}
		}
#endif

		if ((message->pid) == PID_ERR)
		{
			//the following should be printed, as this should not happen in normaal operation anyway.
			fprintf(stderr, "		ERROR: ");
			switch (message->data[0])
			{
			case 0:
				fprintf(stderr, "		Command processor synchronisation error\n");
				break;
			case 0x10:
				fprintf(stderr, "		Command was not executed \n");
				break;
			case 0x11:
				fprintf(stderr, "		Command rejected because DAQ is running\n");
				break;
			case 0x12:
				fprintf(stderr, "		Command rejected because PGM is running\n");
				break;
			case 0x20:
				fprintf(stderr, "		Unknown command or not implemented optional command\n");
				break;
			case 0x21:
				fprintf(stderr, "		Command syntax invalid\n");
				break;
			case 0x22:
				fprintf(stderr, "		Command syntax valid but command parameter(s) out of range\n");
				break;
			case 0x23:
				fprintf(stderr, "		The memory location is write protected\n");
				break;
			case 0x24:
				fprintf(stderr, "		The memory location is not accessible\n");
				break;
			case 0x25:
				fprintf(stderr, "		Access denied : seeds & key is required\n");
				break;
			case 0x26:
				fprintf(stderr, "		selected page not available\n");
				break;
			case 0x27:
				fprintf(stderr, "		selected page mode not available\n");
				break;
			case 0x28:
				fprintf(stderr, "		selected segment not valid\n");
				break;
			case 0x29:
				fprintf(stderr, "		Sequence error \n");
				break;
			case 0x2A:
				fprintf(stderr, "		DAQ configuration not valid \n");
				break;
			case 0x30:
				fprintf(stderr, "		Memory overflow error \n");
				break;
			case 0x31:
				fprintf(stderr, "		Generic error \n");
				break;
			case 0x32:
				fprintf(stderr, "		The slave internal program verify routine detects an error\n");
				break;
			case 0x33:
				fprintf(stderr, "		Acess to the requested resource is temporary not possible\n");
				break;
			default:
				break;
			}
		}
		else if ((message->pid) == PID_EV)
		{
			fprintf(stderr, "EVENT : ");
			switch (message->data[0])
			{
			case 0:
				fprintf(stderr, "		Slave starting in RESUME mode\n");
				break;
			case 0x01:
				fprintf(stderr, "		The DAQ configuration in non-volatile memory has been cleared\n");
				break;
			case 0x02:
				fprintf(stderr, "		The DAQ configuration has been stored into non-volatile memory\n");
				break;
			case 0x03:
				fprintf(stderr, "		The calibration data has been stored into non-volatile memory\n");
				break;
			case 0x05:
				fprintf(stderr, "		Slave requesting to restart time-out\n");
				break;
			case 0x06:
				fprintf(stderr, "		DAQ processor overload\n");
				break;
			case 0x07:
				fprintf(stderr, "		Session terminated by slave device\n");
				break;
			case 0x08:
				fprintf(stderr, "		Transfer of externally triggered timestamp\n");
				break;
			case 0x09:
				fprintf(stderr, "		Indication of a STIM timeout\n");
				break;
			case 0x0A:
				fprintf(stderr, "		Slave entering SLEEP mode\n");
				break;
			case 0x0B:
				fprintf(stderr, "		Slave leaving SLEEP mode\n");
				break;
			case 0xFE:
				fprintf(stderr, "		User-defined event\n");
				break;
			case 0xFF:
				fprintf(stderr, "		Transport layer specific event\n");
				break;
			default:
				break;
			}
		}
		else if ((message->pid) == PID_SERV)
		{
#ifdef DUMP_DATA_MODE
			int i;
			if (!(message->data[0]))
				fprintf(stderr, "		Slave requesting to be reset\n");
			else
				fprintf(stderr, "		Slave transfering ascii text: ");
			for (i = 1; i < (message->header.len) - 1; i++)
			{
				fprintf(stderr, "%c", message->data[i]);
			}
			fprintf(stderr, "\n");
#endif
		}
		else if ((message->pid) == PID_RES)
		{
#ifdef DUMP_DATA_MODE
			//print ascii response data
			int i;
			fprintf(stderr, "ascii:");
			for (i = 0; i < (message->header.len) - 1; i++)
			{
				fprintf(stderr, "%c", message->data[i]);
			}
			fprintf(stderr, "\n");
#endif
		}
		return;
	}
}
