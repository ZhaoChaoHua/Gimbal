/******************************* (C) Embest ***********************************
* File Name          : e2pfs_port.c
* Author             : liren
* Date               : 2009-11-6
* Version            : 
* Description        : 
******************************************************************************/
#include "e2pfs_port.h"
#include "eeprom.h"
/*=============================================================================
* Function		: 
* Description	: 
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
int e2pfs_ecritical(void)
{
	return 0;
}

/*=============================================================================
* Function		: 
* Description	: 
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
int e2pfs_lcritical(void)
{
	return 0;
}

	
#if E2PFS_ECC
#define		_ECC_MAGIC		'X'
/*=============================================================================
* Function		: 
* Description	: 自定义校验函数
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
unsigned int _e2pfs_checkout(void *dat, int len)
{
	int i;
	unsigned char *p = dat, v = _ECC_MAGIC;
	
	for (i=0; i<len; i++) 
		v ^= p[i];
	return v;
}
#endif

/*****************************************************************************
	
				底层EEPROM接口

 *****************************************************************************/

#define E2ROM_CAPACITY	(64*512) 

static struct rt_mutex e2plock;

void parameter_init()
{
    rt_mutex_init(&e2plock, "e2p", RT_IPC_FLAG_FIFO);
}

int e2prom_read(int addr, void *_buf, int len)
{
	unsigned char *buf = (unsigned char *)_buf;
    rt_err_t result;
	
	if (addr < 0 || (addr + len > E2ROM_CAPACITY))
		return -1;
    result = rt_mutex_take(&e2plock, RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        rt_set_errno(-RT_EBUSY);

        return RT_ERROR;
    }
	len = sEE_ReadBuffer(buf, (uint16_t)addr,(uint16_t *)&len);
    rt_mutex_release(&e2plock);
	return len;
}

int e2prom_write(int addr, void *_buf, int len)
{
	unsigned char *buf = (unsigned char *)_buf;
	rt_err_t result;
    
	if (addr < 0 || (addr + len > E2ROM_CAPACITY))
		return -1;
    result = rt_mutex_take(&e2plock, RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        rt_set_errno(-RT_EBUSY);

        return RT_ERROR;
    }
	len = sEE_WriteBuffer(buf, (uint16_t)addr, len);
    rt_mutex_release(&e2plock);
    
	return len;
}
