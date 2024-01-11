/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#include "radar_manager_ros/internal/communication/tcp_socket.h"

int createSocket(const char* ip, const int port)
{
  struct sockaddr_in stSockAddr;
  int Res;
  int SocketFD = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

  if (-1 == SocketFD) {
    perror("cannot create socket");
    // exit(EXIT_FAILURE);
    return SocketFD;
  }

  memset(&stSockAddr, 0, sizeof(stSockAddr));

  stSockAddr.sin_family = AF_INET;
  stSockAddr.sin_port = htons(port);
  Res = inet_pton(AF_INET, ip, &stSockAddr.sin_addr);

  if (0 > Res) {
    perror("error: first parameter is not a valid address family");
    close(SocketFD);
    // exit(EXIT_FAILURE);
    SocketFD = -1;
  } else if (0 == Res) {
    perror("char string (second parameter does not contain valid ip-address)");
    close(SocketFD);
    // exit(EXIT_FAILURE);
    SocketFD = -1;
  } else {
    if (-1 == connect(SocketFD, (struct sockaddr*)&stSockAddr, sizeof(stSockAddr))) {
      perror("connect failed");
      close(SocketFD);
      // exit(EXIT_FAILURE);
      SocketFD = -1;
    }
  }
  return SocketFD;
}

int closeSocket(int SocketFD)
{
  /* perform read write operations ... */

  (void)shutdown(SocketFD, SHUT_RDWR);

  close(SocketFD);
  return EXIT_SUCCESS;
}
