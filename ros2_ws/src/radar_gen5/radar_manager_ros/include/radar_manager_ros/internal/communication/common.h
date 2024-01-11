/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#ifndef COMMON_H
#define COMMON_H

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X, Y) ((X) > (Y) ? (X) : (Y))
typedef enum program_mode_enum
{
  ADTF_DAT_PROCESS_MODE = 0,
  RADAR_CONNEXION_MODE
} program_mode_enum;

typedef unsigned int UB;
typedef unsigned int UW;
typedef unsigned int UL;
typedef int SW;
typedef int SL;
typedef float F;

#define TEST_RADAR_CONNECTION

#define DTO_SEQUENCE_FINISHED 1
#define DTO_SEQUENCE_IN_PROGRESS 0
#define DTO_SEQUENCE_ERROR -1

#endif
