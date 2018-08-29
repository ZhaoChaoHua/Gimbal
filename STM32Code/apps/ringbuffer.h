#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <rtthread.h>
#include <rtdevice.h>
#include <stm32f4xx.h>

#define MAXSIZE          20
#define MAX_MAV_LENTH    255

typedef struct
{
	uint8_t stx;
	uint8_t	payload_lth;
	uint8_t packet_seq;
	uint8_t sys_id;
	uint8_t comp_id;
	uint8_t msg_id;
	uint8_t data[MAX_MAV_LENTH];
	uint8_t crcl;
	uint8_t crch;
}Mavlink_msg_t;

class QRingBuffer
{
public:
    QRingBuffer();
    ~QRingBuffer();

    bool isFull();
    bool isEmpty();
    void empty();
    int getLength();
    int write(Mavlink_msg_t* buf,int count);
    int read(Mavlink_msg_t* buf,int count);

    int getStart()
    {
        return m_nReadPos;
    }

    int getEnd()
    {
        return m_nWritePos;
    }
private:
    bool m_bEmpty;
    bool m_bFull;

    int  m_nBufSize;
    int  m_nReadPos;
    int  m_nWritePos;
    Mavlink_msg_t* m_pBuf;
    int  test;
};

#endif // RINGBUFFER_H
