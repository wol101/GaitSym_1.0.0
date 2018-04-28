/*
 *  Connection.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 04/03/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

enum ConnectionError
{
    noerror = 0,
    gethostbyname_error = 1,
    socket_error = 2,
    connect_error = 3,
    send_error = 4,
    incomplete_send_error = 5,
    recv_error = 6
};

class Connection
{
public:
    Connection();
    ~Connection();
    
    ConnectionError SetServer(const char *host);
    ConnectionError Connect(int port);
    ConnectionError Close();
    ConnectionError Send(const char *data, int len);
    ConnectionError Receive(char *data, int maxLen, int *numBytes);
    
protected:
        
    struct hostent *m_he; 
    struct sockaddr_in m_their_addr; // connector’s address information 
    int m_sockfd;
};
