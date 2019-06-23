#include <stdio.h>
#include "uart.h"
#include "ymodem.h"

extern void putchar(char);
extern int getchar(char *ch, int timeout);

void ymodem_putchar(char ch)
{
    putchar(ch);
}

int ymodem_getchar(char *ch, int timeout)
{
    return getchar(ch, timeout);
}

/* http://www.ccsinfo.com/forum/viewtopic.php?t=24977 */
unsigned short crc16(const unsigned char *buf, unsigned long count)
{
    unsigned short crc = 0;
    int i;

    while(count--) {
        crc = crc ^ *buf++ << 8;

        for (i=0; i<8; i++) {
            if (crc & 0x8000) {
                crc = crc << 1 ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

static const char *u32_to_str(unsigned int val)
{
    /* Maximum number of decimal digits in u32 is 10 */
    static char num_str[11];
    int  pos = 10;
    num_str[10] = 0;

    if (val == 0) {
        /* If already zero then just return zero */
        return "0";
    }

    while ((val != 0) && (pos > 0)) {
        num_str[--pos] = (val % 10) + '0';
        val /= 10;
    }

    return &num_str[pos];
}

static unsigned long str_to_u32(char* str)
{
    const char *s = str;
    unsigned long acc;
    int c;

    /* strip leading spaces if any */
    do {
        c = *s++;
    } while (c == ' ');

    for (acc = 0; (c >= '0') && (c <= '9'); c = *s++) {
        c -= '0';
        acc *= 10;
        acc += c;
    }
    return acc;
}

typedef void (*packet_handler_t)(char *buf, int len);

int receive_packet()
{

}

int ymodem_recv_start(char *buf, char *file_name, int *file_len)
{
    char packet_type;
    int packet_size = 0;
    char ch;
    int i;

    packet_type = *buf;

    switch (packet_type) {
        case SOH:
            packet_size = PACKET_SIZE_128;
            break;
        case STX:
            packet_size = PACKET_SIZE_1K;
            break;
        default:
            return 0;
    }

    for (i = 0; i < packet_size + PACKET_OVERHEAD; i++) {
        if (!ymodem_getchar(&buf[i], PACKET_TIMEOUT)) {
            break;
        }
    }


    return 0;
}

enum {
    YMODEM_IDLE=0,
    YMODEM_PROFILE,
    YMODEM_DATE,
    YMODEM_EOF,
    YMODEM_CAN,
};

static char recv_buf[PACKET_1K_SIZE];
static char file_name[PACKET_1K_SIZE];
static int file_len = 0;

int ymodem_receive_file(void)
{
    int state = YMODEM_IDLE;

    char *buf_ptr = recv_buf;

    memset(buf_ptr, 0,PACKET_1K_SIZE);

    while (1) {
        switch(state) {
            case YMODEM_IDLE:
                ymodem_putchar(CRC);
                if (ymodem_getchar(buf_ptr, PACKET_TIMEOUT)) {
                    state = YMODEM_START;
                }
                break;
            case YMODEM_START:
                if (ymodem_recv_start(buf_ptr, file_name, &file_len)) {
                    
                }
                break;
            case YMODEM_DATA:

                break;
            case YMODEM_EOF:

                break;
            case YMODEM_CAN:

                break;
            default:
                break;
        }
    }
}




