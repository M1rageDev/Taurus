#include "core/communication.h"

#include "core/logging.h"

#pragma comment(lib, "ws2_32.lib")

SOCKET taurus::sock::SetupWSASocket() {
	// init WSA
	WSAData data;
	int ret = WSAStartup(MAKEWORD(2, 2), &data);
	if (ret != 0) {
		logging::error("WSAStartup() failed: %d", WSAGetLastError());
		return NULL;
	}

	// create the socket
	SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock == INVALID_SOCKET) {
		logging::error("socket() failed: %d", WSAGetLastError());
		return NULL;
	}

	return sock;
}

void taurus::sock::BindWSASocket(SOCKET sock, u_short port) {
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(port);

	int ret = bind(sock, reinterpret_cast<SOCKADDR*>(&addr), sizeof(addr));
	if (ret < 0) {
		logging::error("bind() failed: %d", WSAGetLastError());
	}
}

void taurus::sock::CloseSocket(SOCKET sock) {
	closesocket(sock);
}

void taurus::sock::CleanupComms() {
	WSACleanup();
}

bool taurus::sock::CheckData(int len, char* buffer, int bufferSize) {
	// check if we actually received anything
	if (len < 1) return false;

	// check for overflow
	if (len > bufferSize - 1) {
		logging::error("Received data overflow! (%d bytes received)", len);
		return false;
	}

	return true;
}

bool taurus::sock::ReceiveData(SOCKET socket, char* buffer, int bufferSize, int* len) {
	sockaddr_in from;
	int fromSize = sizeof(from);

	int ret = recvfrom(socket, buffer, bufferSize, 0, reinterpret_cast<SOCKADDR*>(&from), &fromSize);
	if (ret < 0) {
		logging::error("recvfrom() failed: %d", WSAGetLastError());
		return false;
	}

	// make the buffer zero terminated
	buffer[ret] = 0;
	*len = ret;
	return sock::CheckData(ret, buffer, bufferSize);
}

void taurus::sock::SerializeTaurusMsg(const messages::TaurusMessage& msg, const char* buffer) {
	msg.SerializeToArray((void*)buffer, msg.ByteSizeLong());
}

void taurus::sock::SendData(SOCKET socket, u_short port, const char* buffer, int bufferLen) {
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	InetPton(AF_INET, L"127.0.0.1", &addr.sin_addr.s_addr);

	int ret = sendto(socket, buffer, bufferLen, 0, reinterpret_cast<SOCKADDR*>(&addr), sizeof(addr));

	if (ret < 0) {
		logging::error("send() failed: %d", WSAGetLastError());
		return;
	}
}
