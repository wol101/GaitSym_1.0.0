/*
 *  Connection.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 04/03/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <errno.h> 
#include <string.h> 
#include <netdb.h> 
#include <sys/types.h> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <fcntl.h>

#include "Connection.h"

Connection::Connection()
{
}

Connection::~Connection()
{
}

ConnectionError Connection::SetServer(const char *host)
{
    if ((m_he = gethostbyname(host)) == 0)
    {
        return gethostbyname_error;
    }
    return noerror;
}

ConnectionError Connection::Connect(int port)
{
    if ((m_sockfd = socket(PF_INET, SOCK_STREAM, 0)) == -1) 
    { 
        return socket_error; 
    } 
    
    // currently use non-blocking sockets
    // would be better to either allow blocking and use threads
    // or perhaps use select to monitor sockets
    fcntl(m_sockfd, F_SETFL, O_NONBLOCK);
    
    m_their_addr.sin_family = AF_INET; // host byte order 
    m_their_addr.sin_port = htons(port); // short, network byte order 
    m_their_addr.sin_addr = *((struct in_addr *)m_he->h_addr); 
    memset(&(m_their_addr.sin_zero), 0, 8); // zero the rest of the struct 
    if (connect(m_sockfd, (struct sockaddr *)&m_their_addr, sizeof(struct sockaddr)) == -1) 
    { 
        return connect_error; 
    } 
    return noerror;
}

ConnectionError Connection::Close()
{
    close(m_sockfd); 
    return noerror;
}

// note may need to break up large messages into smaller chunks
// note check errno for EAGAIN if get error due to busy connection
ConnectionError Connection::Send(const char *data, int len)
{
    int charsSent;
    if ((charsSent = send(m_sockfd, data, len, 0)) == -1) 
    {
        return send_error;
    }
    if (charsSent != len)
    {
        return incomplete_send_error;
    }
    return noerror;
}

// note check errno for EAGAIN if get error due to busy connection
ConnectionError Connection::Receive(char *data, int maxLen, int *numBytes)
{
    if ((*numBytes = recv(m_sockfd, data, maxLen - 1, 0)) == -1) 
    { 
        return recv_error;
    }
    data[*numBytes] = 0;
    return noerror;
} 

