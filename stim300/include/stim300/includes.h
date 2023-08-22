
#define BUFFERSIZE 1024
#define IDXMASK 1023
#define HEADER_BO 0x93

#define OK 1
#define TRUE 1
#define FALSE 0

#define PACKETSIZE  38


typedef int HANDLE;
typedef HANDLE *PHANDLE;
typedef int HRESULT;



#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>
