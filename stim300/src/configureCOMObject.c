#include "../include/stim300/includes.h"

void configureCOMObject(PHANDLE pHComm,int portNumber) {

char  dev[] = "/dev/ttyUSB0";

struct termios tty;

//UDP Settings
extern struct sockaddr_in addr;
extern int txSocketHandle;

txSocketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
if(txSocketHandle!=-1){
	printf("TX SOCKET OLUSTURULDU.\n");
}


addr.sin_family = AF_INET;
addr.sin_port = htons(5150);
addr.sin_addr.s_addr = inet_addr("192.168.123.216");



*pHComm = open(dev,  O_RDONLY | O_NOCTTY );
	if (*pHComm < 0) {
		printf(stderr, "Error: Cannot open <%s> due to <%s>\n", dev, strerror(errno));
		return -1;
	}


printf("COM NESNESI OLUSTURULDU.\n");


memset(&tty, 0, sizeof(tty));
if (tcgetattr(*pHComm, &tty) != 0) {
        printf(stderr, "Error: tcgetattr failed due to <%s>\n", strerror(errno));
		return -1;
	}

tty.c_cflag &= ~PARENB;
tty.c_cflag &= ~CSTOPB;
tty.c_cflag &= ~CSIZE;
tty.c_cflag |= CS8;
tty.c_cflag &= ~CBAUD;
tty.c_cflag |= CBAUDEX;
	
cfsetspeed(&tty, 921600);

tty.c_cc[VTIME] = 0.02; // in desiseconds
cfmakeraw(&tty);

tcflush(*pHComm, TCIFLUSH);
	if ( tcsetattr(*pHComm, TCSANOW, &tty) != 0) {
		printf(stderr, "Error: tcsetattr failed due to <%s>\n", strerror(errno));
		return -1;
	}


return;
}