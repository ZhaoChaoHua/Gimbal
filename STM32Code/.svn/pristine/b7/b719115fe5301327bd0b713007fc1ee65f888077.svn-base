#include "time_measure.h"

// 设定测量起点
void Time_point::start_measure_time(void)
{
    uint8_t i;
    
    //记录下周期
	time_p.period  =  getCount();
	if(time_p.period > time_p.period_max) 
        time_p.period_max = time_p.period ;
    
    //计算上一次的时间间隔
	for(i=0; i<TIME_CAP; i++)
	{
		//第一个，直接时间间隔直接等于第一个点的值
		if(i == 0)
		{
			time_p.distant[0] = time_p.point[0];
			continue;
		}
		//无效点，则跳出
		if(time_p.point[i] < time_p.point[i-1]) break;
		
		time_p.distant[i] = time_p.point[i] - time_p.point[i-1];
	}
    
	//记录下最大总时间
	time_p.total = time_p.point[i-1];
	if(time_p.total_max < time_p.total)
        time_p.total_max = time_p.total;

	//提取distant最大值
	for(i=0; i<TIME_CAP; i++)
	{
		if(time_p.distant_max[i] < time_p.distant[i])
            time_p.distant_max[i] = time_p.distant[i];
	}
    
    // 停止定时器
    pause();
    // 设定溢出值
    setOverflow(0xFFFFFFFF);
    // 
	setPrescaleFactor(TIMER_FREQUENCY - 1, TIM_PSCReloadMode_Update);

    setCounterMode(TIM_CounterMode_Up);
    setClockDiv(TIM_CKD_DIV1);
    // 启动定时器
    resume();
}

// 提取测量值返回两次测量之间的时间差
uint32_t Time_point::return_time(rt_uint8_t point)
{
    time_p.point[point] = getCount();
    time_p.interval[point][TIME_OLD] = time_p.interval[point][TIME_NOW];
    time_p.interval[point][TIME_NOW] = getCount();
    time_p.interval[point][TIME_NEW] = time_p.interval[point][TIME_NOW] - time_p.interval[point][TIME_OLD];
   	return time_p.interval[point][TIME_NEW];
}

// 提取测量值，提取固定点的定时器计数值
uint32_t Time_point::get_time(rt_uint8_t point)
{
    time_p.point[point] = getCount();
    return getCount();
}

// 停止测量
void Time_point::stop_measure_time(void)
{
    uint8_t i ;

	//计算上一次的时间间隔
	for(i=0; i<TIME_CAP; i++)
	{
		//第一个，直接时间间隔直接等于第一个点的值
		if(i == 0)
		{
			time_p.distant[0] = time_p.point[0];
			continue;
		}
		//无效点，则跳出
		if(time_p.point[i] < time_p.point[i-1]) break;

		time_p.distant[i] = time_p.point[i] - time_p.point[i-1];
	}
	//记录下最大总时间
	time_p.total = time_p.point[i-1];
	if(time_p.total_max < time_p.total)
        time_p.total_max = time_p.total;

	//提取最大值
	for(i=0; i<TIME_CAP; i++)
	{
		if(time_p.distant_max[i] < time_p.distant[i])
            time_p.distant_max[i] = time_p.distant[i];
	}
   	//停止测量
	pause();
}

Time_point::~Time_point()
{
    stop_measure_time();
}


