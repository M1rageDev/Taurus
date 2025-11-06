#pragma once

#ifndef WIN32_LEAN_AND_MEAN

#include <WinSock2.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <ws2tcpip.h>

#endif

#include "protocol/TaurusMessages.pb.h"

#include "logging.h"

namespace sock
{
	// setups and binds WSA and a UDP socket to the given port
	SOCKET SetupWSASocket();
	void BindWSASocket(SOCKET sock, u_short port);

	// closes a socket
	void CloseSocket(SOCKET sock);

	// cleanups winsock
	void CleanupComms();

	// checks if the received data is valid, returns true if data is valid
	bool CheckData(int len, char* buffer, int bufferSize);

	// tries to receive data, returns if data is valid, puts received length in len pointer
	bool ReceiveData(SOCKET sock, char* buffer, int bufferSize, int* len);

	// serializes a driver msg into a byte buffer
	void SerializeDriverMsg(const messages::DriverMessage& msg, const char* buffer);

	// sends data to the given port on localhost
	void SendData(SOCKET socket, u_short port, const char* buffer, int bufferLen);
}
