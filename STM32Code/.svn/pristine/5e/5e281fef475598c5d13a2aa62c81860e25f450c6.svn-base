#ifndef __TIIME_MEASURE_H__
#define __TIIME_MEASURE_H__

#ifdef __cplusplus
#include "timers.h"

#define TIME_OLD     0
#define TIME_NOW     1
#define TIME_NEW     2

//测量精度 ms 或者 us 不需要的注释掉即可（注意：计数最大值都是65535）
#define TIME_RANGE_US
//#define TIME_RANGE_MS

//输入定时器的时钟（系统时钟）单位MHz
#define TIMER_FREQUENCY       42
//时间容量(最大时间点数)
#define TIME_CAP 		      18

//总结构体
struct _time_meas
{
	uint32_t point[TIME_CAP];			// 时间点
	uint32_t distant[TIME_CAP];			// 此次测量的时间点间隔
	uint32_t distant_max[TIME_CAP];		// 系统启动以来distant的最大值
	
	uint32_t period;					// 第二次回到初始点的时间
	uint32_t period_max;				// period 的最大值

	uint32_t total;						// 此次从开始测量到最后一个时间点的时间长度
	uint32_t total_max;					// 系统启动以来，total的最大值
    uint32_t interval[TIME_CAP][3];     // 时间间隔提取
};

// 时间点记录点
class Time_point : public SPRain_TIMER
{  
public:
    struct _time_meas time_p;
    /* 构造函数
     * tim - 定时器编号
     */
    Time_point():SPRain_TIMER(2)
    {

    };
    ~Time_point();
    // 设定测量起点
    void start_measure_time(void);
    // 提取测量值
    uint32_t return_time(rt_uint8_t point);
    uint32_t get_time(rt_uint8_t point);
    // 停止测量
    void stop_measure_time(void);

};

#endif

#endif

