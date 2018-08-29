/******************************* (C) Embest ***********************************
* File Name          : e2pfs.h
* Author             : liren
* Date               : 2009-11-6
* Version            : 
* Description        : 
******************************************************************************/

#ifndef __E2ROMFS_H__
#define __E2ROMFS_H__

/* --------------------------------------------------------------------------------*/
#include "e2pfs_port.h"
#include <rtthread.h>

/*错误代码*/
#ifndef EOF
#define EOF					(-1)
#endif
#define ERR_ARG				(-2)					/* 参数错误*/
#define ERR_MEM				(-3)					/* 空间不足*/
#define ERR_WRP				(-4)					/* 页面读写超出边界*/	
#define ERR_CRI				(-5)					/* 进入临界区失败*/
#define ERR_ECC				(-6)					/* 校验错误*/
#define ERR_EFD				(-7)					/* 错误文件描述符*/
#define ERR_FOP				(-8)					/* 文件已经打开，不能多次打开*/
//#define ERR_FNO				(-8)					/* 描述符没有对应打开文件*/

#ifndef SEEK_SET
#define SEEK_SET	1
#endif
#ifndef SEEK_CUR
#define SEEK_CUR	2
#endif
#ifndef SEEK_END
#define SEEK_END	3
#endif

typedef struct 
{
	fid_t			i_id;							/* 文件索引号 */
	unsigned short	i_fpage;						/* 文件内容存放的始页面号 */
	fsize_t			i_size;							/* 文件大小 */
    unsigned short  _default;                       /* 判断是否修改过 0:参数未定义 1:参数为修改参数 */
} Item_Info_t;

typedef struct 
{
	Item_Info_t		f_info;
	unsigned short  f_page;							/* 当前offset所在的页面,起始地址所在页面 */
	unsigned short	f_offset;                       /* 存储位置 */
} E2P_File;

extern E2P_File sOFiles[E2PFS_OFILES];							/*打开的文件信息*/

#define FD2FILEP(x)			((x<0 || x>=E2PFS_OFILES) ? RT_NULL : &sOFiles[x])

int e2pfs_format(void);
int e2pfs_open(int id);
int e2pfs_read(int fd, void *buf, int len);
int e2pfs_write(int fd, void *buf, int len);
int e2pfs_lseek(int fd, int offset, int which);
int e2pfs_close(int fd);
int e2pfs_remove(int fid);

int e2pfs_fsinfo(void);
int e2pfs_file_pgs(int id);
int e2pfs_fexist(int id);							/* 判断给定的文件是否存在*/ 
int e2pfs_ls(void);									/* 显示文件信息及EEPROM使用信息*/
int e2pfs_file_pgs(int id);						/* 计算文件使用的页面数*/

rt_uint32_t get_used_ram(void);

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

#endif
