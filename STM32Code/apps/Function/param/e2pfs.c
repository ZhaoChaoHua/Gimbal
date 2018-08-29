/******************************* (C) Embest ***********************************
* File Name          : e2pfs.c
* Author             : liren
* Date               : 2009-11-6
* Version            : 
* Description        : 
******************************************************************************/
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <rtthread.h>

#include "e2pfs.h"

#define DEBUG_ERR			(0)
#define DEBUG_WAN			(1)
#define DEBUG_MSG			(2)

#define FILE_ITEM_SIZE	(sizeof (Item_Info_t))					/* 文件目录项大小 */

/*系统属性*/
#define E2PFS_PAGE_SIZE		64				                    /* 文件系统页面大小64 */
#define E2PFS_PAGE_MASK		((~(E2PFS_PAGE_SIZE-1)) & 0xffff)	/* 页面大小掩码192 */
	
/*每一页面系统可利用数据大小 = 页面大小-页面ECC大小*/
#define FSPAGE_USE_SIZE		(E2PFS_PAGE_SIZE - _ECC_SIZE)  // 64-1 = 63

/*文件系统页面管理区起始页面地址*/
#define FSMENG_B			0

/* 文件系统页面管理区占用页面数 */
#if E2PFS_PAGES <= 0xff
#define FSMENG_ITEM_SIZE	1
#define FSMENG_PER_PAGE		FSPAGE_USE_SIZE
#elif E2PFS_PAGES <= 0xffff 
#define FSMENG_ITEM_SIZE	2
#define FSMENG_PER_PAGE		(FSPAGE_USE_SIZE>>1) // 63 / 2 = 31 每个页面可以存储31个页面管理信息
#endif

// 页面管理区占的页面数
#define FSMENG_PAGES		((E2PFS_PAGES+FSMENG_PER_PAGE-1)/FSMENG_PER_PAGE) // (512+30)/31 = 17

/* 一个页面存放目录项的数 */
#define ITEMS_PER_PAGE		(FSPAGE_USE_SIZE/FILE_ITEM_SIZE) // 63/16 = 3

/* 目录项存放起始页面地址*/
#define FSITEM_B			(FSMENG_B+FSMENG_PAGES) // 整页为起点 
/*文件系统目录占用页面数*/
#define FSITEM_PAGES		((E2PFS_FILES+ITEMS_PER_PAGE-1)/ITEMS_PER_PAGE)  // (100+3-1)/3 = 34
						

/*数据存放起始页面地址*/
#define FSDATA_B			(FSITEM_B+FSITEM_PAGES)
/*数据存放区页面数*/
#define FSDATA_PAGES		(E2PFS_PAGES-FSITEM_PAGES-FSMENG_PAGES) // 512-17-20

/*文件id转换成目录项存储逻辑地址*/
#define FSID2ADDR(x)		((FSITEM_B+((x)-1)/ITEMS_PER_PAGE)*E2PFS_PAGE_SIZE + \
								((x)-1)%ITEMS_PER_PAGE*FILE_ITEM_SIZE)

/*逻辑页地址[0, E2PFS_PAGES)转换成页面管理物理地址*/
#define PADDR2MADDR(x)		(((FSMENG_B+((x)/FSMENG_PER_PAGE))*E2PFS_PAGE_SIZE) + \
								((x)%FSMENG_PER_PAGE*FSMENG_ITEM_SIZE))

/*逻辑页号转换成页面地址*/
#define PAGE2ADDR(x)		((x)*E2PFS_PAGE_SIZE)
/*页内地址转换成页地址*/
#define ADDR2PADDR(x)		((x)&E2PFS_PAGE_MASK)// 130    1000 0010 & 1100 0000 = 1000 0000 = 128
/*页地址转换成页号*/
#define ADDR2PAGE(x)		((x)/E2PFS_PAGE_SIZE)
/*取页内地址在页内的偏移*/
#define ADDR2OFFS(x)		((x)&(~E2PFS_PAGE_MASK)) // 2 1000 0010 & 0011 1111 = 10 = 2

E2P_File sOFiles[E2PFS_OFILES] = {0,0};							/*打开的文件信息*/

#if E2PFS_ECC
static struct _page_buf_t 
{
	int pg;
	unsigned char data[E2PFS_PAGE_SIZE];
}_dbuf = {
	-1
};
#endif

#if E2PFS_DEBUG
static void DEBUG_INFO(int lv, const char * x, ...)
{
	if (E2PFS_DEBUG > (lv))  {	
		rt_kprintf("LEAVE[%d] : ",(lv));				
		if ((lv) == DEBUG_ERR)						
			rt_kprintf("%s(%d) : ", __FILE__, __LINE__);
            rt_kprintf("%s \n",x);
	}
}
#else
#define	DEBUG_INFO(lv, x)
#endif

#if E2PFS_ECC
unsigned int _e2pfs_checkout(void *dat, int len);
#endif
/*=============================================================================
* Function		: _fs_page_read
* Description	: 文件系统页面读
=============================================================================*/
static int _fs_page_read(int addr, void *dat, int len)
{
    // 读取数据长度超出页面可用大小
	if (ADDR2OFFS(addr) + len > FSPAGE_USE_SIZE)
    {
		len = FSPAGE_USE_SIZE - ADDR2OFFS(addr);
	}
// 使用校验
#if E2PFS_ECC
{
	unsigned int sum = 0;
    // 转换为页地址
	if (_dbuf.pg != ADDR2PADDR(addr)) {
		_dbuf.pg = ADDR2PADDR(addr);
		// 读取一整页的数据
		e2prom_read(_dbuf.pg +  (E2PFS_ADDR_BEGIN), _dbuf.data, E2PFS_PAGE_SIZE);
        // 拷贝取出校验位至sum
		memcpy(&sum, _dbuf.data + FSPAGE_USE_SIZE, _ECC_SIZE);
        // 寄偶校验
		if (sum != _e2pfs_checkout(_dbuf.data, FSPAGE_USE_SIZE))
        {
			DEBUG_INFO(DEBUG_ERR, "[err]E2F Checkout error\n");
			_dbuf.pg = -1;
			return ERR_ECC;
		}
	}
	memcpy(dat, _dbuf.data + (addr & ~E2PFS_PAGE_MASK), len);
}
#else
	len = e2prom_read((E2PFS_ADDR_BEGIN) + addr, dat, len);
#endif

	return len;
}
/*=============================================================================
* Function		: _fs_page_write
* Description	: 文件系统页面写
=============================================================================*/
static int _fs_page_write(int addr, void *dat, int len)
{
	if (ADDR2OFFS(addr) + len > FSPAGE_USE_SIZE) 
    {
		len = FSPAGE_USE_SIZE - ADDR2OFFS(addr);
	}

#if E2PFS_ECC
{
	unsigned int sum;

	if (_dbuf.pg != ADDR2PADDR(addr)) 
    {
		_dbuf.pg = ADDR2PADDR(addr);
        // 读取一整页数据
		e2prom_read(_dbuf.pg + (E2PFS_ADDR_BEGIN), _dbuf.data, E2PFS_PAGE_SIZE);
	}
    // dat->_dbuf.data+ADDR2OFFS(addr)
	memcpy(_dbuf.data+ADDR2OFFS(addr), dat, len);
	sum = _e2pfs_checkout(_dbuf.data, FSPAGE_USE_SIZE);
	memcpy(_dbuf.data + FSPAGE_USE_SIZE, &sum, _ECC_SIZE);
	
	/* 写数据*/
	e2prom_write(_dbuf.pg + ADDR2OFFS(addr) + (E2PFS_ADDR_BEGIN), _dbuf.data + ADDR2OFFS(addr), len);
	
	/*写校验*/
	e2prom_write(_dbuf.pg + FSPAGE_USE_SIZE + (E2PFS_ADDR_BEGIN), _dbuf.data + FSPAGE_USE_SIZE, _ECC_SIZE);	
}
#else
	len = e2prom_write(E2PFS_ADDR_BEGIN + addr, dat, len);
#endif

	return len;
}

/*页面使用标记 0：未用，1：保留系统使用, 其它：文件下一页面号*/
#define PAGE_UNUSE			0	
#define PAGE_RESEV			1
/*=============================================================================
* Function		: _fs_get_page_state
* Description	: 读取页面状态
* Input Para	: 页面编号
=============================================================================*/
static int _fs_get_page_state(int p)
{
	int f = 0;
	if (p<0 || p>E2PFS_PAGES) 
    {
		DEBUG_INFO(DEBUG_ERR, "_fs_get_page_state : Parameter ERROR p=%d\n", p);
		return ERR_ARG;
	}
    // 读取页面管理信息
	if (_fs_page_read(PADDR2MADDR(p), &f, FSMENG_ITEM_SIZE) < 0)
		return ERR_WRP;

	return f;
}
/*=============================================================================
* Function		: 
* Description	: 查找未使用的页
=============================================================================*/
static int _fs_find_unuse_page(void)
{
	int i;
	int f = PAGE_RESEV;

	for (i=FSDATA_B; i<FSDATA_B+FSDATA_PAGES && f!=PAGE_UNUSE; i++) {
		if (_fs_page_read(PADDR2MADDR(i), &f, FSMENG_ITEM_SIZE) < 0)
			return ERR_WRP;
	}
	--i;
	if (f != PAGE_UNUSE) {
		DEBUG_INFO(DEBUG_ERR, "[err]No empty page\n");
		return ERR_MEM;
	}

	return i;
}
/*=============================================================================
* Function		: 
* Description	: 设置页面状态
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
static int _fs_set_page_state(int p, int s)
{
	if (!(((p >= 0 && p < FSDATA_B+FSDATA_PAGES) &&  (p < FSDATA_B && s == PAGE_RESEV)) || ((s >= FSDATA_B && (s < FSDATA_B + FSDATA_PAGES)) || s == PAGE_UNUSE)))
	{
		DEBUG_INFO(DEBUG_ERR, "[err]_fs_set_page_state:Parameter ERROR: p=%d s=%d\r\n", p, s);
		return ERR_ARG;
	}
	if (_fs_page_write(PADDR2MADDR(p), &s, FSMENG_ITEM_SIZE) < 0)
		return ERR_WRP;
	return 0;
}
/*=============================================================================
* Function		: 
* Description	: 读取文件ID
=============================================================================*/
static int _fs_read_item(int i, Item_Info_t *pit)
{
	if (_fs_page_read(FSID2ADDR(i), pit, FILE_ITEM_SIZE) < 0)
		return ERR_WRP;
	return 0;
}

/*=============================================================================
* Function		: 
* Description	: 写入文件ID
=============================================================================*/
static int _fs_write_item(int i, Item_Info_t *pit)
{
	if (i<=0 || i>E2PFS_FILES) {
		DEBUG_INFO(DEBUG_ERR, "[err]_fs_write_item:parameter err id = %d\r\n", i);
		return ERR_ARG;
	}
	if (_fs_page_write(FSID2ADDR(i), pit, FILE_ITEM_SIZE) < 0)
		return ERR_WRP;

	return 0;
}

/*=============================================================================
* Function		: 
* Description	: 
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
static int _e2pfs_page_read(E2P_File *fp, unsigned char *buf, int len)
{
	int off, llen, rlen;

	assert(fp != NULL);
	assert(fp->f_info.i_id != 0);

	off = fp->f_offset % FSPAGE_USE_SIZE;    // 得到页面中的偏移地址
	llen = fp->f_info.i_size - fp->f_offset; // 

	if (llen == 0)
		return EOF;
	rlen = len<llen?len:llen;

	rlen = _fs_page_read(PAGE2ADDR(fp->f_page)+off, buf, rlen);
	if (rlen < 0) 
		return ERR_WRP;
	
	fp->f_offset += rlen;

	if (fp->f_offset % FSPAGE_USE_SIZE == 0)
		fp->f_page = _fs_get_page_state(fp->f_page);
	return rlen;
}

/*=============================================================================
* Function		: 
* Description	: 
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
static int _e2pfs_page_write(E2P_File *fp, unsigned char *buf, int len)
{
	int off, llen, wlen;
	
	assert(fp != NULL);
	assert(fp->f_info.i_id != 0);

	off = fp->f_offset % FSPAGE_USE_SIZE;
	llen = FSPAGE_USE_SIZE - off;
	if (off == 0 && fp->f_offset == fp->f_info.i_size) {
		/*分配空页*/
		int pg;
		pg = _fs_find_unuse_page();
		if (pg < 0) {
			return ERR_MEM;
		}
		if (fp->f_info.i_fpage == 0) 
			fp->f_info.i_fpage = pg;
		else
			_fs_set_page_state(fp->f_page, pg);

		_fs_set_page_state(pg, pg);
		fp->f_page = pg;
	}
	wlen = len<llen?len:llen;
	wlen = _fs_page_write(PAGE2ADDR(fp->f_page)+off, buf, wlen);
	if (wlen < 0) {
		DEBUG_INFO(DEBUG_ERR, "_fs_page_write\r\n");
		return wlen;
	}
	fp->f_offset += wlen;
	if (fp->f_offset > fp->f_info.i_size)
		fp->f_info.i_size = fp->f_offset;
	return wlen;
}


static int _e2pfs_get_fd(int fid)
{
	int i;
	int fd = ERR_EFD;

	for (i=0; i<E2PFS_OFILES; i++)
    {
		if (sOFiles[i].f_info.i_id == 0) 
        {
			fd = i;
		}
		if (sOFiles[i].f_info.i_id == fid)
			/* 文件已打开*/
			return ERR_FOP;	
	}
	/* 没有空闲文件描述符*/
	return fd;
}
// 释放内存
static int _e2pfs_free_fd(int fd)
{
	memset(sOFiles+fd, 0, sizeof (sOFiles));
	return 0;
}
/*=============================================================================
* Function		: 
* Description	: 
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
int e2pfs_format(void)
{
	int i;
	Item_Info_t fi;

	DEBUG_INFO(DEBUG_MSG, "FileSystem space size  : %d(K)\n", (E2PFS_PAGES*E2PFS_PAGE_SIZE)>>10);
	DEBUG_INFO(DEBUG_MSG, "FileSystem page size   : %d(B)\n", E2PFS_PAGE_SIZE);
	DEBUG_INFO(DEBUG_MSG, "FileSystem page number : %d\n", E2PFS_PAGES);
	DEBUG_INFO(DEBUG_MSG, "Page Manger begin addr : %d\n", FSMENG_B);
	DEBUG_INFO(DEBUG_MSG, "Page Manger per page   : %d\n", FSMENG_PER_PAGE);
	DEBUG_INFO(DEBUG_MSG, "Pages for Page Manger  : %d\n", FSMENG_PAGES);
	DEBUG_INFO(DEBUG_MSG, "ID Tiem begin addr     : %d\n", FSITEM_B);
	DEBUG_INFO(DEBUG_MSG, "Pages for ID Tiem      : %d\n", FSITEM_PAGES);
	DEBUG_INFO(DEBUG_MSG, "Data address begin addr: %d\n", FSDATA_B);
	DEBUG_INFO(DEBUG_MSG, "Pages for data         : %d\n", FSDATA_PAGES);

	/* 格式化目录管理区 */
	memset(&fi, 0, FILE_ITEM_SIZE);
	for (i=1; i<=E2PFS_FILES; i++)
		_fs_write_item(i, &fi);
	
	/* 格式化页面管理区 */
	for (i=0; i<E2PFS_PAGES; i++) {
		int f = PAGE_RESEV;
		if (i >= FSDATA_B && i < (FSDATA_B+FSDATA_PAGES)) 
			f = PAGE_UNUSE;
		_fs_set_page_state(i, f);
				
	}

	return 0;
}

/*=============================================================================
* Function		: 
* Description	: 
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
int e2pfs_open(int fid/*, E2P_File *file*/)
{
	int ret;
	int fd;
	E2P_File *file;

//	assert(file != NULL);
    // 判断文件标识符是否正确
	if (fid<=0 || fid>E2PFS_FILES) {
		DEBUG_INFO(DEBUG_ERR, "e2pfs_open:Parameter ERROR id=%d\n", fid);
		return ERR_ARG;
	}
    // 得到文件描述符(若为新文件,返回空闲的文件描述符)
	fd = _e2pfs_get_fd(fid);
	if (fd < 0) {
		DEBUG_INFO(DEBUG_ERR, "No free file descriptors fid=%d\n", fid);
		return ERR_EFD;
	}
    // 得到文件描述结构体内存地址
	file = FD2FILEP(fd);
	// 读取文件目录信息
	ret = _fs_read_item(fid, &file->f_info);
	if (ret < 0) {
		return ret;
	}
    // 偏移
	file->f_offset = 0;
    // 当前所在页
	file->f_page = file->f_info.i_fpage;
    // 文件已经存在
	if (file->f_info.i_id != 0)
    {
		DEBUG_INFO(DEBUG_MSG, "Open file id : %d\n", fid);
		return fd;
	}
    
	/* 文件不存在,创建文件 */
	file->f_info.i_id    = fid;
	file->f_info.i_fpage = 0;
	file->f_info.i_size  = 0;
    file->f_info._default = 0; // 创建文件为参数未定义
	/**/
	file->f_page = 0;

	DEBUG_INFO(DEBUG_MSG, "Mkdir file id : %d\n", fid);

	return fd;
}



/*=============================================================================
* Function		: 
* Description	: 
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
int e2pfs_read(int fd/*E2P_File *fp*/, void *buf, int len)
{
	unsigned char *_buf = buf;
	int rlen, tlen = 0;
    // 取得文件信息
	E2P_File *fp = FD2FILEP(fd);

	assert(buf != NULL);

	if (fp == NULL || fp->f_info.i_id == 0) {
		DEBUG_INFO(DEBUG_ERR, "fd[%d]no opened file\n", fd);
		return ERR_EFD;
	}
r:
	rlen = _e2pfs_page_read(fp, _buf+tlen, len-tlen);
	if (rlen < 0) {
		if (tlen)
			return tlen;
		return EOF;
	}
	tlen += rlen;
	if (tlen < len)
		goto r;
	return tlen;
}


/*=============================================================================
* Function		: 
* Description	: 
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
int e2pfs_write(int fd/*E2P_File *fp*/, void *buf, int len)
{
	unsigned char *_buf = buf;
	int wlen, tlen = 0;
	E2P_File *fp = FD2FILEP(fd);
	
	assert(_buf != NULL);
	
	if (fp == NULL || fp->f_info.i_id == 0) {
		DEBUG_INFO(DEBUG_ERR, "fd[%d]no opened file\n", fd);
		return ERR_EFD;
	}

w:
	wlen = _e2pfs_page_write(fp, _buf+tlen, len-tlen);
	if (wlen < 0) {
		if (tlen > 0)
			return tlen;

		DEBUG_INFO(DEBUG_ERR, "_fs_page_write : Overflow EEPROM address space\n");
		return wlen;
	}
	tlen += wlen;
	if (tlen < len)
		goto w;
	return tlen;
}

/*=============================================================================
* Function		: 
* Description	: 
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
int e2pfs_lseek(int fd, /*E2P_File *fp,*/ int offset, int which)
{
	E2P_File *fp = FD2FILEP(fd);
	long cur   = fp->f_offset;	
	long fsize = fp->f_info.i_size;
	unsigned short  npg;
	int	 noff;
	
	if (fp == NULL || fp->f_info.i_id == 0) {
		DEBUG_INFO(DEBUG_ERR, "fd[%d]no opened file\n", fd);
		return ERR_EFD;
	}

	switch (which) {
	case SEEK_SET:
		noff = offset;
		break;
	case SEEK_CUR:
		noff = cur + offset;
		break;
	case SEEK_END:
		noff = fsize + offset;
		break;
	default:
		return ERR_ARG;
	}
	if (noff < 0 || noff > fsize)
		return ERR_ARG;
	npg = fp->f_page;
	if (noff<cur) {
		npg = fp->f_info.i_fpage;
		cur = 0;
	} else {
		cur = cur / FSPAGE_USE_SIZE * FSPAGE_USE_SIZE; 
	}
	while (cur + FSPAGE_USE_SIZE <= noff) {
		npg = _fs_get_page_state(npg);
		cur += FSPAGE_USE_SIZE;
	}
	fp->f_page = npg;
	fp->f_offset = noff;
	return noff;
}

/*=============================================================================
* Function		: 
* Description	: 
=============================================================================*/
int e2pfs_close(int fd/*E2P_File *fp*/)
{
    // 得到打开的文件描述符内存地址
	E2P_File *fp = FD2FILEP(fd);
	
	if (fp == NULL || fp->f_info.i_id == 0) {
		DEBUG_INFO(DEBUG_ERR, "fd[%d]no opened file\n", fd);
		return ERR_EFD;
	}
//	assert(fp != NULL);
    // 文件信息写入EEPROM
	_fs_write_item(fp->f_info.i_id, &fp->f_info);
	
	DEBUG_INFO(DEBUG_MSG, "Close file id : %d\r\n", fp->f_info.i_id);

	/* 释放文件描述符*/
	_e2pfs_free_fd(fd);

	return 0;
}

/*=============================================================================
* Function		: 
* Description	: 
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
int e2pfs_remove(int fid)
{
	Item_Info_t it;
	int pg, np;

	if (fid<=0 || fid>E2PFS_FILES) {
		DEBUG_INFO(DEBUG_ERR, "e2pfs_remove:Parameter ERROR id=%d\r\n", fid);
		return ERR_ARG;
	}

	_fs_read_item(fid, &it);
	if (it.i_fpage == 0)
		goto rm;
	/*删除文件使用的页面*/
	pg = it.i_fpage; // 起始页面
	np = _fs_get_page_state(pg); // 文件的下一页面号
	while (pg != np) {
		_fs_set_page_state(pg, PAGE_UNUSE);
		pg = np;
		np = _fs_get_page_state(pg);
	}
	_fs_set_page_state(pg, PAGE_UNUSE);
rm:
	memset(&it, 0, FILE_ITEM_SIZE);
	_fs_write_item(fid, &it);
	return 0;
}

#if E2PFS_EFUN
/*=============================================================================
* Function		: e2pfs_ls 
* Description	: 显示文件信息及EEPROM使用信息 
=============================================================================*/
int e2pfs_fsinfo(void)
{
	rt_kprintf("         E2PFS Table\n");
	rt_kprintf("=====================================\n");
	rt_kprintf("FSEEPROM RAM: %-6d(K)\n", (E2PFS_PAGES*E2PFS_PAGE_SIZE)>>10);
	rt_kprintf("  FSMENG RAM: %-6d(B)  %d.%d %%\n", FSMENG_PAGES*FSPAGE_USE_SIZE,
		(int)(FSMENG_PAGES*FSPAGE_USE_SIZE*100.0/((E2PFS_PAGES*E2PFS_PAGE_SIZE))),
        (int)(10*FSMENG_PAGES*FSPAGE_USE_SIZE*100.0/((E2PFS_PAGES*E2PFS_PAGE_SIZE)))%10);
    
	rt_kprintf("  FSITEM RAM: %-6d(B)  %d.%d %%\n", FSITEM_PAGES*FSPAGE_USE_SIZE,
		(int)(FSITEM_PAGES*FSPAGE_USE_SIZE*100.0/((E2PFS_PAGES*E2PFS_PAGE_SIZE))),
        (int)(10*FSITEM_PAGES*FSPAGE_USE_SIZE*100.0/((E2PFS_PAGES*E2PFS_PAGE_SIZE)))%10);
    
	rt_kprintf("     ECC RAM: %-6d(B)  %d.%d %%\n", _ECC_SIZE*E2PFS_PAGES,
		(int)(_ECC_SIZE*E2PFS_PAGES*100.0/((E2PFS_PAGES*E2PFS_PAGE_SIZE))),
        (int)(10*_ECC_SIZE*E2PFS_PAGES*100.0/((E2PFS_PAGES*E2PFS_PAGE_SIZE)))%10);
    
	rt_kprintf("EEPROM Usage: %-5c     %d.%d %%\n", ' ', 
		(int)((FSDATA_PAGES*FSPAGE_USE_SIZE*100.0)/(E2PFS_PAGES*E2PFS_PAGE_SIZE)),
        (int)(10*(FSDATA_PAGES*FSPAGE_USE_SIZE*100.0)/(E2PFS_PAGES*E2PFS_PAGE_SIZE))%10);
	rt_kprintf("=====================================\n");
	return 0;
}

/*=============================================================================
* Function		: e2pfs_file_pgs
* Description	: 计算文件使用的页面数
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
int e2pfs_file_pgs(int id)
{
	int bpg, pg, num = 1;
	Item_Info_t it;

	if (id<=0 || id>E2PFS_FILES) {
		DEBUG_INFO(DEBUG_ERR, "e2pfs_remove:Parameter ERROR id=%d\r\n", id);
		return ERR_ARG;
	}

	_fs_read_item(id, &it);
	bpg = it.i_fpage;

	if (it.i_id == 0 || bpg == 0)
		return 0;

	pg = _fs_get_page_state(bpg);
	while (pg != bpg) {
		num++;
		bpg = pg;
		pg = _fs_get_page_state(pg);
	}
	return num;
}

/*=============================================================================
* Function		: e2pfs_fexist
* Description	: 判断给定的文件是否存在
* Input Para	: 
* Output Para	: 
* Return Value  : 0：不存在，其他：存在文件号为id的文件
=============================================================================*/
int e2pfs_fexist(int id)
{
	Item_Info_t it;
	if (id<=0 || id>E2PFS_FILES) {
		DEBUG_INFO(DEBUG_ERR, "e2pfs_fexist:Parameter ERROR id=%d\r\n", id);
		return ERR_ARG;
	}

	if (_fs_read_item(id, &it) < 0)
		return ERR_WRP;
	return it.i_id;
}

/*=============================================================================
* Function		: e2pfs_ls 
* Description	: 显示文件信息及EEPROM使用信息
* Input Para	: 
* Output Para	: 
* Return Value  : 
=============================================================================*/
int e2pfs_ls(void)
{
	fid_t			i; 
	int				pgs;
	Item_Info_t		it;
	unsigned int	tbyte;
	unsigned int	ubyte = 0;

	rt_kprintf("Id      ");//创建日期          长 度   起始页面  页面数  百分比\r\n");
	rt_kprintf("Size   Page_B  Page Size  Percent\r\n");
	rt_kprintf("=========================================");
	rt_kprintf("\r\n");
	for (i=1; i<=E2PFS_FILES; i++) {
		_fs_read_item(i, &it);
		if (it.i_id == 0)
			continue;
		if (it.i_id != i) { 
			DEBUG_INFO(DEBUG_WAN, "Item Data ERROR fid = %d\r\n", i);
			continue;
		}
		ubyte += it.i_size;
		pgs = e2pfs_file_pgs(i);

		rt_kprintf("%-6u  ", it.i_id);
		rt_kprintf("%-6u  %-8u  %-6u    %d.%d%%\n",
			it.i_size, it.i_fpage, pgs, (int)(100.0*pgs)/FSDATA_PAGES, (int)(10*(100.0*pgs)/FSDATA_PAGES)%10);
	}
	
	tbyte = FSDATA_PAGES*FSPAGE_USE_SIZE;
	
	rt_kprintf("=========================================");
	rt_kprintf("\r\n");
	rt_kprintf("RAM: %d(B) Used: %d(B) Rest: %d(B) Usage: %d.%d %%\r\n", 
		tbyte, ubyte, tbyte-ubyte, 100*ubyte/tbyte, 1000*ubyte/tbyte%10);
		
	return 0;
}

rt_uint32_t get_used_ram(void)
{
	rt_uint32_t used_ram = 0;
	Item_Info_t		e2prom;
	
	for (int i=1; i<=E2PFS_FILES; i++)
	{
		_fs_read_item(i, &e2prom);
		if (e2prom.i_id == 0)
			continue;
		if (e2prom.i_id != i) { 
			DEBUG_INFO(DEBUG_WAN, "Item Data ERROR fid = %d\r\n", i);
			continue;
		}
		used_ram += e2prom.i_size;
	}
	
	return used_ram;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

// 格式化EEPROM
void format_e2pfs()
{
    e2pfs_format();
}
FINSH_FUNCTION_EXPORT(format_e2pfs, Formate EEPROM RAM.)

// 
void rm_e2pfs(int id)
{
    if (e2pfs_fexist(id) == 0) 
    {
		rt_kprintf("No found id [%d]\n", id);
	}
    else
    {
        e2pfs_remove(id);
    }
}
FINSH_FUNCTION_EXPORT(rm_e2pfs, Remove EEPROM RAM.)

// 列出目录信息
void ls_e2pfs()
{
	e2pfs_ls();
}
FINSH_FUNCTION_EXPORT(ls_e2pfs, List EEPROM RAM.)

// 列出目录信息
void fsinfo_e2pfs()
{
	e2pfs_fsinfo();
}
FINSH_FUNCTION_EXPORT(fsinfo_e2pfs, Filesystem info EEPROM RAM.)

#endif

#endif

