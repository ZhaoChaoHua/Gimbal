#include "ringbuffer.h"
#include <string.h>
#include <assert.h>

QRingBuffer::QRingBuffer()
{
    m_bEmpty = true;
    m_bFull  = false;
    m_nBufSize = MAXSIZE;
    m_nReadPos  =0;
    m_nWritePos =0;
    // mcalloc
    m_pBuf = new Mavlink_msg_t[MAXSIZE];
    test = 0;
}

QRingBuffer::~QRingBuffer()
{
    delete[] m_pBuf;
}

int QRingBuffer::write(Mavlink_msg_t* buf,int count)
{
    if(count <=0)
        return 0;
    m_bEmpty =false;
    // buffer is full
    if(m_bFull)
    {
        return 0;
    }
    else if(m_nReadPos == m_nWritePos)// 缓冲区为空时
    {
        /*                          == 内存模型 ==
         (empty)             m_nReadPos                (empty)
        |----------------------------------|-----------------------------------------|
             m_nWritePos        m_nBufSize
        */
        int leftcount = m_nBufSize - m_nWritePos;
        if(leftcount > count)
        {
            memcpy(m_pBuf + m_nWritePos, buf, count*sizeof(Mavlink_msg_t));
            m_nWritePos += count;
            m_bFull =(m_nWritePos == m_nReadPos);
            return count;
        }
        else
        {
            memcpy(m_pBuf + m_nWritePos, buf, leftcount*sizeof(Mavlink_msg_t));
            m_nWritePos =(m_nReadPos > count - leftcount)? count - leftcount : m_nWritePos;
            memcpy(m_pBuf, buf + leftcount, m_nWritePos*sizeof(Mavlink_msg_t));
            m_bFull =(m_nWritePos == m_nReadPos);
            return leftcount + m_nWritePos;
        }
    }
    else if(m_nReadPos < m_nWritePos)// 有剩余空间可写入
    {
        /*                           == 内存模型 ==
        (empty)                 (data)                     (empty)
        |-------------------|----------------------------|---------------------------|
         m_nReadPos                m_nWritePos       (leftcount)
        */
        // 剩余缓冲区大小(从写入位置到缓冲区尾)

        int leftcount = m_nBufSize - m_nWritePos;
        int test = m_nWritePos;
        if(leftcount > count)   // 有足够的剩余空间存放
        {
            memcpy(m_pBuf + m_nWritePos, buf, count*sizeof(Mavlink_msg_t));
            m_nWritePos += count;
            m_bFull =(m_nReadPos == m_nWritePos);
            assert(m_nReadPos <= m_nBufSize);
            assert(m_nWritePos <= m_nBufSize);
            return count;
        }
        else       // 剩余空间不足
        {
            // 先填充满剩余空间，再回头找空间存放
            memcpy(m_pBuf + test, buf, leftcount*sizeof(Mavlink_msg_t));

            m_nWritePos =(m_nReadPos >= count - leftcount)? count - leftcount : m_nReadPos;
            memcpy(m_pBuf, buf + leftcount, m_nWritePos*sizeof(Mavlink_msg_t));
            m_bFull =(m_nReadPos == m_nWritePos);
            assert(m_nReadPos <= m_nBufSize);
            assert(m_nWritePos <= m_nBufSize);
            return leftcount + m_nWritePos;
        }
    }
    else
    {
        /*                          == 内存模型 ==
        (unread)                 (read)                     (unread)
        |-------------------|----------------------------|---------------------------|
        m_nWritePos    (leftcount)    m_nReadPos
        */
        int leftcount = m_nReadPos - m_nWritePos;
        if(leftcount > count)
        {
            // 有足够的剩余空间存放
            memcpy(m_pBuf + m_nWritePos, buf, count*sizeof(Mavlink_msg_t));
            m_nWritePos += count;
            m_bFull =(m_nReadPos == m_nWritePos);
            assert(m_nReadPos <= m_nBufSize);
            assert(m_nWritePos <= m_nBufSize);
            return count;
        }
        else
        {
            // 剩余空间不足时要丢弃后面的数据
            memcpy(m_pBuf + m_nWritePos, buf, leftcount*sizeof(Mavlink_msg_t));
            m_nWritePos += leftcount;
            m_bFull =(m_nReadPos == m_nWritePos);
            assert(m_bFull);
            assert(m_nReadPos <= m_nBufSize);
            assert(m_nWritePos <= m_nBufSize);
            return leftcount;
        }
    }
}

int QRingBuffer::read(Mavlink_msg_t* buf,int count)
{
    if(count <=0)
        return 0;
    m_bFull =false;
    if(m_bEmpty)       // 缓冲区空，不能继续读取数据
    {
        return 0;
    }
    else if(m_nReadPos == m_nWritePos)   // 缓冲区满时
    {
        /*                          == 内存模型 ==
        (data)          m_nReadPos                (data)
        |--------------------------------|--------------------------------------------|
        m_nWritePos         m_nBufSize
        */
        int leftcount = m_nBufSize - m_nReadPos;
        if(leftcount > count)
        {
            memcpy(buf, m_pBuf + m_nReadPos, count*sizeof(Mavlink_msg_t));
            m_nReadPos += count;
            m_bEmpty =(m_nReadPos == m_nWritePos);
            return count;
        }
        else
        {
            memcpy(buf, m_pBuf + m_nReadPos, leftcount*sizeof(Mavlink_msg_t));
            m_nReadPos =(m_nWritePos > count - leftcount)? count - leftcount : m_nWritePos;
            memcpy(buf + leftcount, m_pBuf, m_nReadPos*sizeof(Mavlink_msg_t));
            m_bEmpty =(m_nReadPos == m_nWritePos);
            return leftcount + m_nReadPos;
        }
    }
    else if(m_nReadPos < m_nWritePos)   // 写指针在前(未读数据是连接的)
    {
        /*                          == 内存模型 ==
        (read)                 (unread)                      (read)
        |-------------------|----------------------------|---------------------------|
        m_nReadPos                m_nWritePos                     m_nBufSize
        */
        int leftcount = m_nWritePos - m_nReadPos;
        int c =(leftcount > count)? count : leftcount;
        memcpy(buf, m_pBuf + m_nReadPos, c*sizeof(Mavlink_msg_t));
        m_nReadPos += c;
        m_bEmpty =(m_nReadPos == m_nWritePos);
        assert(m_nReadPos <= m_nBufSize);
        assert(m_nWritePos <= m_nBufSize);
        return c;
    }
    else          // 读指针在前(未读数据可能是不连接的)
    {
        /*                          == 内存模型 ==
        (unread)                (read)                      (unread)
        |-------------------|----------------------------|---------------------------|
        m_nWritePos                  m_nReadPos                  m_nBufSize

        */
        int leftcount = m_nBufSize - m_nReadPos;
        if(leftcount > count)   // 未读缓冲区够大，直接读取数据
        {
            memcpy(buf, m_pBuf + m_nReadPos, count*sizeof(Mavlink_msg_t));
            m_nReadPos += count;
            m_bEmpty =(m_nReadPos == m_nWritePos);
            assert(m_nReadPos <= m_nBufSize);
            assert(m_nWritePos <= m_nBufSize);
            return count;
        }
        else       // 未读缓冲区不足，需回到缓冲区头开始读
        {
            memcpy(buf, m_pBuf + m_nReadPos, leftcount*sizeof(Mavlink_msg_t));
            m_nReadPos =(m_nWritePos >= count - leftcount)? count - leftcount : m_nWritePos;
            memcpy(buf + leftcount, m_pBuf, m_nReadPos*sizeof(Mavlink_msg_t));
            m_bEmpty =(m_nReadPos == m_nWritePos);
            assert(m_nReadPos <= m_nBufSize);
            assert(m_nWritePos <= m_nBufSize);
            return leftcount + m_nReadPos;
        }
    }
}

int QRingBuffer::getLength()
{
    if(m_bEmpty)
    {
        return 0;
    }
    else if(m_bFull)
    {
        return m_nBufSize;
    }
    else if(m_nReadPos < m_nWritePos)
    {
        return m_nWritePos - m_nReadPos;
    }
    else
    {
        return m_nBufSize - m_nReadPos + m_nWritePos;
    }
}

void QRingBuffer::empty()
{
    m_nReadPos =0;
    m_nWritePos =0;
    m_bEmpty =true;
    m_bFull =false;
}

bool QRingBuffer::isEmpty()
{
    return m_bEmpty;
}

bool QRingBuffer::isFull()
{
    return m_bFull;
}
