/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#ifndef TCP_SOCKET_H
#define TCP_SOCKET_H

#include "radar_manager_ros/internal/communication/common.h"

int createSocket(const char* ip, const int port);
int closeSocket(int socketFD);

#endif
