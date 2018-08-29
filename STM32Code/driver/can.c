/*
 * File      : can.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author            Notes
 * 2015-05-14     aubrcool@qq.com   first version
 * 2015-07-06     Bernard           code cleanup and remove RT_CAN_USING_LED;
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "can.h"

#define CAN_LOCK(can)   rt_mutex_take(&(can->lock), RT_WAITING_FOREVER)
#define CAN_UNLOCK(can) rt_mutex_release(&(can->lock))

// can init
static rt_err_t rt_can_init(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;
    struct rt_can_device *can;

    RT_ASSERT(dev != RT_NULL);
    can = (struct rt_can_device *)dev;

    /* initialize rx/tx */
    can->can_rx = RT_NULL;
    can->can_tx = RT_NULL;

    /* apply configuration */
    if (can->ops->configure)
        result = can->ops->configure(can, &can->config);

    return result;
}

/*
 * can interrupt receive
 */
rt_inline int _can_int_rx(struct rt_can_device *can, struct rt_can_msg *data, int msgs)
{
    int size;
    struct rt_can_rx_fifo *rx_fifo;
    RT_ASSERT(can != RT_NULL);
    size = msgs;

    rx_fifo = (struct rt_can_rx_fifo *) can->can_rx;
    RT_ASSERT(rx_fifo != RT_NULL);

    /* read from software FIFO */
    while (msgs)
    {
        rt_base_t level;
#ifdef RT_CAN_USING_HDR
        rt_int32_t hdr;
#endif /*RT_CAN_USING_HDR*/
        struct rt_can_msg_list *listmsg = RT_NULL;

        /* disable interrupt */
        level = rt_hw_interrupt_disable();
#ifdef RT_CAN_USING_HDR
        // 过滤器初始化定义hdr对应过滤器索引
        // 确定链表节点位置
        hdr = data->hdr;

        if (hdr >= 0 && can->hdr && hdr < can->config.maxhdr && !rt_list_isempty(&can->hdr[hdr].list))
        {
            listmsg = rt_list_entry(can->hdr[hdr].list.next, struct rt_can_msg_list, hdrlist);
            // rt_can_msg_list链表中删除节点
            rt_list_remove(&listmsg->list);
            rt_list_remove(&listmsg->hdrlist);
            if (can->hdr[hdr].msgs)
            {
                can->hdr[hdr].msgs--;
            }
            listmsg->owner = RT_NULL;
        }
        // 未设置索引号
        else if (hdr == -1)
#endif /*RT_CAN_USING_HDR*/
        {
            // 查看是否有数据
            if (!rt_list_isempty(&rx_fifo->uselist))
            {
                listmsg = rt_list_entry(rx_fifo->uselist.next, struct rt_can_msg_list, list);
                rt_list_remove(&listmsg->list);
#ifdef RT_CAN_USING_HDR
                rt_list_remove(&listmsg->hdrlist);
                if (listmsg->owner != RT_NULL && listmsg->owner->msgs)
                {
                    listmsg->owner->msgs--;
                }
                listmsg->owner = RT_NULL;
#endif /*RT_CAN_USING_HDR*/
            }
            else
            {
                /* no data, enable interrupt and break out */
                rt_hw_interrupt_enable(level);
                break;
            }
        }

        /* enable interrupt */
        rt_hw_interrupt_enable(level);
        // 取出有效数据
        if (listmsg != RT_NULL)
        {
            // 取出数据
            rt_memcpy(data, &listmsg->data, sizeof(struct rt_can_msg));

            level = rt_hw_interrupt_disable();
            // 将节点插入空白链表
            rt_list_insert_before(&rx_fifo->freelist, &listmsg->list);
            rx_fifo->freenumbers++;
            RT_ASSERT(rx_fifo->freenumbers <= can->config.msgboxsz);
            rt_hw_interrupt_enable(level);

            listmsg = RT_NULL;
        }
        else
        {
            break;
        }
        data ++;
        msgs -= sizeof(struct rt_can_msg);
    }

    return (size - msgs);
}

// can interrupt transmit priority by id
rt_inline int _can_int_tx(struct rt_can_device *can, const struct rt_can_msg *data, int msgs)
{
    int size;
    struct rt_can_tx_fifo *tx_fifo;

    RT_ASSERT(can != RT_NULL);

    size = msgs;
    tx_fifo = (struct rt_can_tx_fifo *) can->can_tx;
    RT_ASSERT(tx_fifo != RT_NULL);

    while (msgs)
    {
        rt_base_t level;
        rt_uint32_t no;
        rt_uint32_t result;
        struct rt_can_sndbxinx_list *tx_tosnd = RT_NULL;

        level = rt_hw_interrupt_disable();
        if (!rt_list_isempty(&tx_fifo->freelist))
        {
            tx_tosnd = rt_list_entry(tx_fifo->freelist.next, struct rt_can_sndbxinx_list, list);
            RT_ASSERT(tx_tosnd != RT_NULL);
            rt_list_remove(&tx_tosnd->list);
        }
        else
        {
            rt_hw_interrupt_enable(level);

            rt_completion_wait(&(tx_fifo->completion), RT_WAITING_FOREVER);
            continue;
        }
        rt_hw_interrupt_enable(level);
        // 3个链表节点中任意一个
        no = ((rt_uint32_t)tx_tosnd - (rt_uint32_t)tx_fifo->buffer) / sizeof(struct rt_can_sndbxinx_list);
        tx_tosnd->result = RT_CAN_SND_RESULT_WAIT;
        // 发送数据
        if (can->ops->sendmsg(can, data, no) != RT_EOK)
        {
            /* 发送失败 */
            level = rt_hw_interrupt_disable();
            // 节点加入空节点
            rt_list_insert_after(&tx_fifo->freelist, &tx_tosnd->list);
            rt_hw_interrupt_enable(level);
            continue;
        }

        can->status.sndchange = 1;
        rt_completion_wait(&(tx_tosnd->completion), RT_WAITING_FOREVER);

        level = rt_hw_interrupt_disable();
        result = tx_tosnd->result;
        if (!rt_list_isempty(&tx_tosnd->list))
        {
            rt_list_remove(&tx_tosnd->list);
        }
        rt_list_insert_before(&tx_fifo->freelist, &tx_tosnd->list);
        rt_completion_done(&(tx_fifo->completion));
        rt_hw_interrupt_enable(level);

        if (result == RT_CAN_SND_RESULT_OK)
        {
            level = rt_hw_interrupt_disable();
            can->status.sndpkg++;
            rt_hw_interrupt_enable(level);

            data ++;
            msgs -= sizeof(struct rt_can_msg);
            if (!msgs) break;
        }
        else
        {
            level = rt_hw_interrupt_disable();
            can->status.dropedsndpkg++;
            rt_hw_interrupt_enable(level);
            break;
        }
    }

    return (size - msgs);
}

// can interrupt transmit priority by priv
rt_inline int _can_int_tx_priv(struct rt_can_device *can, const struct rt_can_msg *data, int msgs)
{
    int size;
    rt_base_t level;
    rt_uint32_t no, result;
    struct rt_can_tx_fifo *tx_fifo;

    RT_ASSERT(can != RT_NULL);

    size = msgs;
    tx_fifo = (struct rt_can_tx_fifo *) can->can_tx;
    RT_ASSERT(tx_fifo != RT_NULL);

    while (msgs)
    {
        no = data->priv;
        if (no >= can->config.sndboxnumber)
        {
            break;
        }

        level = rt_hw_interrupt_disable();
        if ((tx_fifo->buffer[no].result != RT_CAN_SND_RESULT_OK))
        {
            rt_hw_interrupt_enable(level);

            rt_completion_wait(&(tx_fifo->buffer[no].completion), RT_WAITING_FOREVER);
            continue;
        }
        tx_fifo->buffer[no].result = RT_CAN_SND_RESULT_WAIT;
        rt_hw_interrupt_enable(level);

        if (can->ops->sendmsg(can, data, no) != RT_EOK)
        {
            continue;
        }
        can->status.sndchange = 1;
        rt_completion_wait(&(tx_fifo->buffer[no].completion), RT_WAITING_FOREVER);

        result = tx_fifo->buffer[no].result;
        if (result == RT_CAN_SND_RESULT_OK)
        {
            level = rt_hw_interrupt_disable();
            can->status.sndpkg++;
            rt_hw_interrupt_enable(level);
            data ++;
            msgs -= sizeof(struct rt_can_msg);
            if (!msgs) break;
        }
        else
        {
            level = rt_hw_interrupt_disable();
            can->status.dropedsndpkg++;
            rt_hw_interrupt_enable(level);
            break;
        }
    }

    return (size - msgs);
}

// can open device
static rt_err_t rt_can_open(struct rt_device *dev, rt_uint16_t oflag)
{
    struct rt_can_device *can;

    RT_ASSERT(dev != RT_NULL);
    can = (struct rt_can_device *)dev;

    CAN_LOCK(can);

    /* get open flags */
    dev->open_flag = oflag & 0xff;
    if (can->can_rx == RT_NULL)
    {
        if (oflag & RT_DEVICE_FLAG_INT_RX)
        {
            int i = 0;
            struct rt_can_rx_fifo *rx_fifo;
            // 分配中断接收缓存
            rx_fifo = (struct rt_can_rx_fifo *) rt_malloc(sizeof(struct rt_can_rx_fifo) +
                      can->config.msgboxsz * sizeof(struct rt_can_msg_list));
            
            RT_ASSERT(rx_fifo != RT_NULL);

            rx_fifo->buffer = (struct rt_can_msg_list *)(rx_fifo + 1);
            // 清除中断接收区缓存
            rt_memset(rx_fifo->buffer, 0, can->config.msgboxsz * sizeof(struct rt_can_msg_list));
            // 链表初始化
            rt_list_init(&rx_fifo->freelist);
            rt_list_init(&rx_fifo->uselist);
            rx_fifo->freenumbers = can->config.msgboxsz; // 16
            
            for (i = 0;  i < can->config.msgboxsz; i++)
            {
                // rt_can_msg_list 中 list 插入freelist 空链表
                rt_list_insert_before(&rx_fifo->freelist, &rx_fifo->buffer[i].list);
#ifdef RT_CAN_USING_HDR
                // 每个rt_list_node hdrlist都是一个头节点
                rt_list_init(&rx_fifo->buffer[i].hdrlist);
                rx_fifo->buffer[i].owner = RT_NULL;
#endif
            }
            can->can_rx = rx_fifo;

            dev->open_flag |= RT_DEVICE_FLAG_INT_RX;
            /* configure low level device */
            can->ops->control(can, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
        }
    }

    if (can->can_tx == RT_NULL)
    {
        if (oflag & RT_DEVICE_FLAG_INT_TX)
        {
            int i = 0;
            struct rt_can_tx_fifo *tx_fifo;

            tx_fifo = (struct rt_can_tx_fifo *) rt_malloc(sizeof(struct rt_can_tx_fifo) +
                      can->config.sndboxnumber * sizeof(struct rt_can_sndbxinx_list));
            RT_ASSERT(tx_fifo != RT_NULL);

            tx_fifo->buffer = (struct rt_can_sndbxinx_list *)(tx_fifo + 1);
            rt_memset(tx_fifo->buffer, 0,
                      can->config.sndboxnumber * sizeof(struct rt_can_sndbxinx_list));
            rt_list_init(&tx_fifo->freelist);
            for (i = 0;  i < can->config.sndboxnumber; i++)
            {
                rt_list_insert_before(&tx_fifo->freelist, &tx_fifo->buffer[i].list);
                rt_completion_init(&(tx_fifo->buffer[i].completion));
                tx_fifo->buffer[i].result = RT_CAN_SND_RESULT_OK;
            }
            rt_completion_init(&(tx_fifo->completion));
            can->can_tx = tx_fifo;

            dev->open_flag |= RT_DEVICE_FLAG_INT_TX;
            /* configure low level device */
            can->ops->control(can, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_TX);
        }
    }

    can->ops->control(can, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_CAN_INT_ERR);

#ifdef RT_CAN_USING_HDR
    // 设置hdr，注册时 can->hdr = RT_NULL
    if (can->hdr == RT_NULL)
    {
        int i = 0;
        struct rt_can_hdr *phdr;
        // 分配内存
        phdr = (struct rt_can_hdr *) rt_malloc(can->config.maxhdr * sizeof(struct rt_can_hdr));
        RT_ASSERT(phdr != RT_NULL);
        // 初始化为0
        rt_memset(phdr, 0, can->config.maxhdr * sizeof(struct rt_can_hdr));
        for (i = 0;  i < can->config.maxhdr; i++)
        {
            // 初始化链表
            rt_list_init(&phdr[i].list);
        }

        can->hdr = phdr;
    }
#endif

    if (!can->timerinitflag)
    {
        can->timerinitflag = 1;

        rt_timer_start(&can->timer);
    }

    CAN_UNLOCK(can);

    return RT_EOK;
}

// can close device
static rt_err_t rt_can_close(struct rt_device *dev)
{
    struct rt_can_device *can;

    RT_ASSERT(dev != RT_NULL);
    can = (struct rt_can_device *)dev;

    CAN_LOCK(can);

    /* this device has more reference count */
    if (dev->ref_count > 1)
    {
        CAN_UNLOCK(can);
        return RT_EOK;
    }

    if (can->timerinitflag)
    {
        can->timerinitflag = 0;

        rt_timer_stop(&can->timer);
    }

    can->status_indicate.ind = RT_NULL;
    can->status_indicate.args = RT_NULL;

#ifdef RT_CAN_USING_HDR
    if (can->hdr != RT_NULL)
    {
        rt_free(can->hdr);
        can->hdr = RT_NULL;
    }
#endif

    if (dev->open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        struct rt_can_rx_fifo *rx_fifo;

        rx_fifo = (struct rt_can_rx_fifo *)can->can_rx;
        RT_ASSERT(rx_fifo != RT_NULL);

        rt_free(rx_fifo);
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_RX;
        /* configure low level device */
        can->ops->control(can, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_RX);
    }

    if (dev->open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        struct rt_can_tx_fifo *tx_fifo;

        tx_fifo = (struct rt_can_tx_fifo *)can->can_tx;
        RT_ASSERT(tx_fifo != RT_NULL);

        rt_free(tx_fifo);
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_TX;
        /* configure low level device */
        can->ops->control(can, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_TX);
    }

    can->ops->control(can, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_CAN_INT_ERR);

    CAN_UNLOCK(can);

    return RT_EOK;
}

// can read device
static rt_size_t rt_can_read(struct rt_device *dev,
                             rt_off_t          pos,
                             void             *buffer,
                             rt_size_t         size)
{
    struct rt_can_device *can;

    RT_ASSERT(dev != RT_NULL);
    if (size == 0) return 0;

    can = (struct rt_can_device *)dev;

    if ((dev->open_flag & RT_DEVICE_FLAG_INT_RX) && (dev->ref_count > 0))
    {
        return _can_int_rx(can, buffer, size);
    }

    return 0;
}

// can write device
static rt_size_t rt_can_write(struct rt_device *dev,
                              rt_off_t          pos,
                              const void       *buffer,
                              rt_size_t         size)
{
    struct rt_can_device *can;

    RT_ASSERT(dev != RT_NULL);
    if (size == 0) return 0;

    can = (struct rt_can_device *)dev;

    if ((dev->open_flag & RT_DEVICE_FLAG_INT_TX) && (dev->ref_count > 0))
    {
        if (can->config.privmode)
        {
            return _can_int_tx_priv(can, buffer, size);
        }
        else  // default mode
        {
            return _can_int_tx(can, buffer, size);
        }
    }
    return 0;
}

// can bus controller (suspend\resume\config\set_priority\SET_STATUS_IND\SET_FILTER\SET_BUS_HOOK)
static rt_err_t rt_can_control(struct rt_device *dev,
                               rt_uint8_t        cmd,
                               void             *args)
{
    struct rt_can_device *can;
    rt_err_t res;
    
    RT_ASSERT(dev != RT_NULL);
    can = (struct rt_can_device *)dev;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        /* suspend device */
        dev->flag |= RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_RESUME:
        /* resume device */
        dev->flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_CONFIG:
        /* configure device */
        can->ops->configure(can, (struct can_configure *)args);
        break;
    case RT_CAN_CMD_SET_PRIV:
        /* configure device */
        if ((rt_uint32_t)args != can->config.privmode)
        {
            int i;
            rt_base_t level;
            struct rt_can_tx_fifo *tx_fifo;

            res = can->ops->control(can, cmd, args);
            if (res != RT_EOK) return res;

            tx_fifo = (struct rt_can_tx_fifo *) can->can_tx;
            if (can->config.privmode)
            {
                rt_completion_done(&(tx_fifo->completion));

                for (i = 0;  i < can->config.sndboxnumber; i++)
                {
		    level = rt_hw_interrupt_disable();
                    rt_list_remove(&tx_fifo->buffer[i].list);
                    rt_hw_interrupt_enable(level);
                }
            }
            else
            {
                for (i = 0;  i < can->config.sndboxnumber; i++)
                {
                    level = rt_hw_interrupt_disable();
                    if (tx_fifo->buffer[i].result == RT_CAN_SND_RESULT_OK)
                    {
                        rt_list_insert_before(&tx_fifo->freelist, &tx_fifo->buffer[i].list);
                    }
                    rt_hw_interrupt_enable(level);
                }
            }
            return RT_EOK;
        }
        break;

    case RT_CAN_CMD_SET_STATUS_IND:
        can->status_indicate.ind = ((rt_can_status_ind_type_t)args)->ind;
        can->status_indicate.args = ((rt_can_status_ind_type_t)args)->args;
        break;

#ifdef RT_CAN_USING_HDR
    case RT_CAN_CMD_SET_FILTER:

        res = can->ops->control(can, cmd, args);
        if (res != RT_EOK || can->hdr == RT_NULL)
        {
            return res;
        }
        // 链表参数
        {
            struct rt_can_filter_config *pfilter;
            struct rt_can_filter_item *pitem;
            rt_uint32_t index = 0;
            
            rt_uint32_t count;
            rt_base_t level;
            // 配置参数中提取出rt_can_filter_config
            pfilter = (struct rt_can_filter_config *)args;
            count = pfilter->count;  // 过滤器总数
            pitem = pfilter->items;  // 过滤器配置参数
            
            if (pfilter->actived)
            {
                while (count)
                {
                    // 初始化时 hdr = -1
                    if (pitem[index].hdr >= can->config.maxhdr || pitem[index].hdr < 0)
                    {
                        count--;
                        index++;
                        continue;
                    }
                  
                    level = rt_hw_interrupt_disable();
                    // 若hdr结构体尚未连接至过滤器
                    if (!can->hdr[pitem[index].hdr].connected)
                    {
                        rt_hw_interrupt_enable(level);
                        // 将对应的过滤器配置参数赋值至can->hdr下
                        rt_memcpy(&can->hdr[pitem[index].hdr].filter, &pitem[index],
                                  sizeof(struct rt_can_filter_item));
                        level = rt_hw_interrupt_disable();
                        // 标记已经连接
                        can->hdr[pitem[index].hdr].connected = 1;
                        can->hdr[pitem[index].hdr].msgs = 0;
                        rt_list_init(&can->hdr[pitem[index].hdr].list);
                        
                    }
                    rt_hw_interrupt_enable(level);

                    count--;
                    index++;
                }
            }
            else   // 未激活状态
            {
                while (count)
                {
                    if (pitem[index].hdr >= can->config.maxhdr || pitem[index].hdr < 0)
                    {
                        count--;
                        index++;
                        continue;
                    }
                    level = rt_hw_interrupt_disable();
                    // 在过滤器未激活状态，若已经连接
                    if (can->hdr[pitem[index].hdr].connected)
                    {
                        // 取消连接状态
                        can->hdr[pitem[index].hdr].connected = 0;
                        can->hdr[pitem[index].hdr].msgs = 0;
                        // 删除list链表下的节点
                        if (!rt_list_isempty(&can->hdr[pitem[index].hdr].list))
                        {
                            rt_list_remove(can->hdr[pitem[index].hdr].list.next);
                        }
                        rt_hw_interrupt_enable(level);
                        rt_memset(&can->hdr[pitem[index].hdr].filter, 0,
                                  sizeof(struct rt_can_filter_item));
                    }
                    else
                    {
                                rt_hw_interrupt_enable(level);
                    }
                    count--;
                    index++;
                }
            }
        }
        break;
#endif /*RT_CAN_USING_HDR*/
#ifdef RT_CAN_USING_BUS_HOOK
    case RT_CAN_CMD_SET_BUS_HOOK:
        can->bus_hook = (rt_can_bus_hook) args;
        break;
#endif /*RT_CAN_USING_BUS_HOOK*/
    default :
        /* control device */
        if (can->ops->control != RT_NULL)
        {
            can->ops->control(can, cmd, args);
        }
        break;
    }

    return RT_EOK;
}

/*
 * can timer
 */
static void cantimeout(void *arg)
{
    rt_can_t can = (rt_can_t)arg;

    rt_device_control((rt_device_t)can, RT_CAN_CMD_GET_STATUS, (void *)&can->status);

    if (can->status_indicate.ind != RT_NULL)
    {
        can->status_indicate.ind(can, can->status_indicate.args);
    }
#ifdef RT_CAN_USING_BUS_HOOK
    if(can->bus_hook)
    {
        can->bus_hook(can);
    }
#endif /*RT_CAN_USING_BUS_HOOK*/
    if (can->timerinitflag == 1)
    {
        can->timerinitflag = 0xFF;
    }
}

/*
 * can register
 */
rt_err_t rt_hw_can_register(struct rt_can_device *can,
                            const char              *name,
                            const struct rt_can_ops *ops,
                            void                    *data)
{
    struct rt_device *device;
    RT_ASSERT(can != RT_NULL);

    device = &(can->parent);

    device->type        = RT_Device_Class_CAN;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
#ifdef RT_CAN_USING_HDR
    can->hdr            = RT_NULL;
#endif
    can->can_rx         = RT_NULL;
    can->can_tx         = RT_NULL;
    // mutex init
    rt_mutex_init(&(can->lock), "can", RT_IPC_FLAG_PRIO);
#ifdef RT_CAN_USING_BUS_HOOK
    can->bus_hook       = RT_NULL;
#endif /*RT_CAN_USING_BUS_HOOK*/
    device->init        = rt_can_init; // incloud hardware init
    device->open        = rt_can_open; // invoke open function will call init
    device->close       = rt_can_close;
    device->read        = rt_can_read;
    device->write       = rt_can_write;
    device->control     = rt_can_control;
    can->ops            = ops;

    can->status_indicate.ind  = RT_NULL;
    can->status_indicate.args = RT_NULL;
    rt_memset(&can->status, 0, sizeof(can->status));

    device->user_data   = data;

    can->timerinitflag  = 0;
    rt_timer_init(&can->timer,
                  name,
                  cantimeout,
                  (void *)can,
                  can->config.ticks,
                  RT_TIMER_FLAG_PERIODIC);
    /* register a character device */
    return rt_device_register(device, name, RT_DEVICE_FLAG_RDWR);
}

/* ISR for can interrupt */
void rt_hw_can_isr(struct rt_can_device *can, int event)
{
    switch (event & 0xff)
    {
    // 接收缓冲区溢出
    case RT_CAN_EVENT_RXOF_IND:
    {
        rt_base_t level;
        level = rt_hw_interrupt_disable();
        can->status.dropedrcvpkg++;
        rt_hw_interrupt_enable(level);
    }
    // Rx indication
    case RT_CAN_EVENT_RX_IND:
    {
        struct rt_can_msg tmpmsg;
        struct rt_can_rx_fifo *rx_fifo;
        struct rt_can_msg_list *listmsg = RT_NULL;
#ifdef RT_CAN_USING_HDR
        rt_int32_t hdr;
#endif
        int ch = -1;
        rt_base_t level;
        rt_uint32_t no;

        rx_fifo = (struct rt_can_rx_fifo *)can->can_rx;
        RT_ASSERT(rx_fifo != RT_NULL);
        /* interrupt mode receive */
        RT_ASSERT(can->parent.open_flag & RT_DEVICE_FLAG_INT_RX);

        no = event >> 8;
        // 接收数据
        ch = can->ops->recvmsg(can, &tmpmsg, no);
        if (ch == -1) break;

        /* disable interrupt */
        level = rt_hw_interrupt_disable();
        can->status.rcvpkg++;
        can->status.rcvchange = 1;
        // 判断自由节点是否为空
        if (!rt_list_isempty(&rx_fifo->freelist))
        {
            // 获取节点地址,删除rt_can_msg_list中的list
            listmsg = rt_list_entry(rx_fifo->freelist.next, struct rt_can_msg_list, list);
            // 链表中删除list节点
            rt_list_remove(&listmsg->list);
#ifdef RT_CAN_USING_HDR
            // 链表中删除hdrlist节点
            rt_list_remove(&listmsg->hdrlist);
            if (listmsg->owner != RT_NULL && listmsg->owner->msgs)
            {
                listmsg->owner->msgs--;
            }
            listmsg->owner = RT_NULL;
#endif /*RT_CAN_USING_HDR*/
            RT_ASSERT(rx_fifo->freenumbers > 0);
            rx_fifo->freenumbers--;
        }
        // 判断使用节点是否为空
        else if (!rt_list_isempty(&rx_fifo->uselist))
        {
            listmsg = rt_list_entry(rx_fifo->uselist.next, struct rt_can_msg_list, list);
            can->status.dropedrcvpkg++; // 覆盖
            rt_list_remove(&listmsg->list);
#ifdef RT_CAN_USING_HDR
            rt_list_remove(&listmsg->hdrlist);
            if (listmsg->owner != RT_NULL && listmsg->owner->msgs)
            {
                listmsg->owner->msgs--;
            }
            listmsg->owner = RT_NULL;
#endif
        }
        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        if (listmsg != RT_NULL)
        {
            // 拷贝参数
            rt_memcpy(&listmsg->data, &tmpmsg, sizeof(struct rt_can_msg));
            level = rt_hw_interrupt_disable();
            rt_list_insert_before(&rx_fifo->uselist, &listmsg->list);
#ifdef RT_CAN_USING_HDR
            hdr = tmpmsg.hdr;   // 过滤器索引号，在过滤器设置时设定
            if (can->hdr != RT_NULL)
            {
                RT_ASSERT(hdr < can->config.maxhdr && hdr >= 0);
                if (can->hdr[hdr].connected)
                {
                    rt_list_insert_before(&can->hdr[hdr].list, &listmsg->hdrlist);
                    listmsg->owner = &can->hdr[hdr];
                    can->hdr[hdr].msgs++;
                }
            }
#endif
            rt_hw_interrupt_enable(level);
        }

        /* invoke callback */
#ifdef RT_CAN_USING_HDR
        if (can->hdr != RT_NULL && can->hdr[hdr].connected && can->hdr[hdr].filter.ind)
        {
            rt_size_t rx_length;
            RT_ASSERT(hdr < can->config.maxhdr && hdr >= 0);

            level = rt_hw_interrupt_disable();
            rx_length = can->hdr[hdr].msgs * sizeof(struct rt_can_msg);
            rt_hw_interrupt_enable(level);
            // 回掉函数，发送事件
            can->hdr[hdr].filter.ind(&can->parent, can->hdr[hdr].filter.args, hdr, rx_length);
        }
        else
#endif
        {
            if (can->parent.rx_indicate != RT_NULL)
            {
                rt_size_t rx_length;

                level = rt_hw_interrupt_disable();
                /* get rx length */
                rx_length = rx_fifo->freenumbers * sizeof(struct rt_can_msg);
                rt_hw_interrupt_enable(level);

                can->parent.rx_indicate(&can->parent, rx_length);
            }
        }
        break;
    }
    // transmit complete
    case RT_CAN_EVENT_TX_DONE:
    // transmit fail
    case RT_CAN_EVENT_TX_FAIL:
    {
        struct rt_can_tx_fifo *tx_fifo;
        rt_uint32_t no;
        no = event >> 8;
        tx_fifo = (struct rt_can_tx_fifo *) can->can_tx;
        RT_ASSERT(tx_fifo != RT_NULL);

        if ((event & 0xff) == RT_CAN_EVENT_TX_DONE)
        {
            tx_fifo->buffer[no].result = RT_CAN_SND_RESULT_OK;
        }
        else
        {
            tx_fifo->buffer[no].result = RT_CAN_SND_RESULT_ERR;
        }
        rt_completion_done(&(tx_fifo->buffer[no].completion));
        break;
    }
    }
}

#ifdef RT_USING_FINSH
#include <finsh.h>
int can_stat(void)
{
    static const char *ErrCode[] =
    {
        "No Error!",
        "Warning !",
        "Passive !",
        "Bus Off !"
    };

    struct rt_can_status status;
    rt_device_t candev = rt_device_find("bxcan1");
    if (!candev)
    {
        rt_kprintf(" Can't find can device %s\n", "bxcan1");
        return -1;
    }
    rt_device_open(candev,RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);
    rt_kprintf(" Finded can device: %s...", "bxcan1");

    rt_device_control(candev, RT_CAN_CMD_GET_STATUS, &status);
    rt_kprintf("\n Receive...error..count: %010ld. Send.....error....count: %010ld.",
               status.rcverrcnt, status.snderrcnt);
    rt_kprintf("\n Bit..pad..error..count: %010ld. Format...error....count: %010ld",
               status.bitpaderrcnt, status.formaterrcnt);
    rt_kprintf("\n Ack.......error..count: %010ld. Bit......error....count: %010ld.",
               status.ackerrcnt, status.biterrcnt);
    rt_kprintf("\n CRC.......error..count: %010ld. Error.code.[%010ld]: ",
               status.crcerrcnt, status.errcode);
    switch (status.errcode)
    {
    case 0:
        rt_kprintf("%s.", ErrCode[0]);
        break;
    case 1:
        rt_kprintf("%s.", ErrCode[1]);
        break;
    case 2:
    case 3:
        rt_kprintf("%s.", ErrCode[2]);
        break;
    case 4:
    case 5:
    case 6:
    case 7:
        rt_kprintf("%s.", ErrCode[3]);
        break;
    }
    rt_kprintf("\n Total.receive.packages: %010ld. Droped.receive.packages: %010ld.",
               status.rcvpkg, status.dropedrcvpkg);
    rt_kprintf("\n Total..send...packages: %010ld. Droped...send..packages: %010ld.\n",
               status.sndpkg + status.dropedsndpkg, status.dropedsndpkg);

    return 0;
}
FINSH_FUNCTION_EXPORT(can_stat, Stat Can Device Status.);
#endif
