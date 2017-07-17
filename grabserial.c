#include <time.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

void print_sys_time()
{
    struct tm       *p;
    struct timeval  tv;
    struct timezone tz;

    gettimeofday(&tv, &tz);
    p = localtime(&tv.tv_sec);

    fprintf(stdout, "[%02d:%02d:%02d:%06ld] ", p->tm_hour, p->tm_min, p->tm_sec, tv.tv_usec);
}

int set_serial_opt(int fd, char *portSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    int nSpeed = atoi(portSpeed);

    if (tcgetattr(fd,&oldtio) != 0) return -1;

    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch(nBits) {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch(nEvent) {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch(nSpeed) {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }

    if(nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);

    if((tcsetattr(fd,TCSANOW,&newtio))!=0) return -1;

    return 0;
}

int main(int argc ,char *argv[])
{
    int i = 0;
    static int j = 1;
    int fd = -1;
    int nread = -1;
    int outputHex = 1;
    char buff[256];
    unsigned char time_buf[25];

    if (argc != 3) {
        fprintf(stderr, "Usage: %s /dev/ttyS1 115200 \n", argv[0]);
        return -1;
    }

    if ((fd = open(argv[1], O_RDWR|O_NOCTTY|O_NDELAY)) < 0) {
        fprintf(stderr, "Can't open serial: %s\n", argv[1]);
        return -1;
    }

    if (set_serial_opt(fd, argv[2], 8, 'N', 1) < 0) {
        fprintf(stderr, "Set_opt error!\n");
        return -1;
    }

    while (1) {
        if ((nread = read(fd, buff, sizeof(buff))) > 0) {
            buff[nread] = '\0';

            if (outputHex != 0) {
                for(i = 0; i <= nread; ++i, j++) {
                    if(j == 1 || j == 17) {
                        j = 1;
                        fprintf(stdout, "\n");
                        print_sys_time();
                    }
                    else if (j == 9 )
                        fprintf(stdout, "- ");
                    fprintf(stdout, "%02X ", buff[i]);
                }
            }
            else {
                for(i = 0; i <= nread; ++i) {
                    fprintf(stdout, "%c", buff[i]);
                    if(buff[i] == '\n')
                        print_sys_time();
                }
            }
        }
        else usleep(200);
    }

    close(fd);

    return 0;
}
