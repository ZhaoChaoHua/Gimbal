#include "time_measure.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f40x_define.h"
#include <math.h>
#include <string.h>

#define TIMER_FREQUENCY       84

/**
 * Header common to all counters.
 */
struct perf_ctr_header {
	struct rt_list_node	    link;	/**< list linkage */
	enum perf_counter_type	type;	/**< counter type */
	const char		        *name;	/**< counter name */
};

/**
 * PC_EVENT counter.
 */
struct perf_ctr_count {
	struct perf_ctr_header	hdr;
	uint64_t		event_count;
};

/**
 * PC_ELAPSED counter.
 */
struct perf_ctr_elapsed {
	struct perf_ctr_header	hdr;
	uint64_t		event_count;
	uint64_t		event_overruns;
	uint64_t		time_start;
	uint64_t		time_total;
	uint64_t		time_least;
	uint64_t		time_most;
	float			mean;
	float			M2;
};

/**
 * PC_INTERVAL counter.
 */
struct perf_ctr_interval {
	struct perf_ctr_header	hdr;
	uint64_t		event_count;
	uint64_t		time_event;
	uint64_t		time_first;
	uint64_t		time_last;
	uint64_t		time_least;
	uint64_t		time_most;
	float			mean;
	float			M2;
};

/**
 * List of all known counters.
 */
static rt_list_t perf_counters;

void perf_init()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    
    rt_list_init(&perf_counters);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    // init pref timer
    TIM_Cmd(TIM2, DISABLE);
    TIM_Cmd(TIM5, DISABLE);
    // TIM5 从定时器
    TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Trigger);
    TIM_SelectInputTrigger(TIM5, TIM_TS_ITR0);
    TIM_ITRxExternalClockConfig(TIM5, TIM_TS_ITR0);
    TIM_SetAutoreload(TIM5, 0xFFFFFFFF);
    TIM_CounterModeConfig(TIM5, TIM_CounterMode_Up);
    TIM_SelectOnePulseMode(TIM5, TIM_OPMode_Single);
    TIM_PrescalerConfig(TIM5, TIMER_FREQUENCY - 1, TIM_PSCReloadMode_Update);
    // TIME2 主定时器
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler = TIMER_FREQUENCY - 1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
    
    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
    
    TIM_SetAutoreload(TIM2, 0xFFFFFFFF);
    TIM_SetAutoreload(TIM5, 0xFFFFFFFF);
    
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
}
// Read the absolute time
uint64_t sys_absolute_time(void)
{
    uint64_t time;
    time = (TIM5->CNT * TIM2->ARR) + TIM2->CNT;
    // transmit the system clock to us
    return time;
}

uint64_t millis(void)
{
    return sys_absolute_time()/1000;
}

perf_counter_t perf_alloc(enum perf_counter_type type, const char *name)
{
	perf_counter_t ctr = RT_NULL;

	switch (type) {
	case PC_COUNT:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_count), 1);
		break;

	case PC_ELAPSED:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_elapsed), 1);
		break;

	case PC_INTERVAL:
		ctr = (perf_counter_t)calloc(sizeof(struct perf_ctr_interval), 1);
		break;

	default:
		break;
	}

	if (ctr != NULL) {
		ctr->type = type;
		ctr->name = name;
        // 插入节点
        rt_list_insert_before(&perf_counters, &ctr->link);
	}
	return ctr;
}

perf_counter_t perf_alloc_once(enum perf_counter_type type, const char *name)
{
	perf_counter_t handle = (perf_counter_t)perf_counters.next;

	while (handle != NULL) {
		if (!strcmp(handle->name, name)) {
			if (type == handle->type) {
				/* they are the same counter */
				return handle;
			} else {
				/* same name but different type, assuming this is an error and not intended */
				return NULL;
			}
		}
		handle = (perf_counter_t)handle->link.next;
	}

	/* if the execution reaches here, no existing counter of that name was found */
	return perf_alloc(type, name);
}

void perf_free(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	rt_list_remove(&handle->link);
	free(handle);
}

void perf_count(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count++;
		break;

	case PC_INTERVAL: {
		struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
		uint64_t now = sys_absolute_time();
        
		switch (pci->event_count) {
		case 0:
            pci->time_first = now;
			break;
		case 1:
            pci->time_least = now - pci->time_last;
			pci->time_most = now - pci->time_last;
			pci->mean = pci->time_least / 1e6f;
			pci->M2 = 0;
			break;
		default: {
                uint32_t interval;

                interval = now - pci->time_last;
                
				if (interval < pci->time_least)
					pci->time_least = interval;
				if (interval > pci->time_most)
					pci->time_most = interval;
                
				float dt = interval / 1e6f;
				float delta_intvl = dt - pci->mean;
				pci->mean += delta_intvl / pci->event_count;
				pci->M2 += delta_intvl * (dt - pci->mean);
				break;
			}
		}
		pci->time_last = now;
		pci->event_count++;
		break;
	}

	default:
		break;
	}
}

void perf_begin(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	switch (handle->type) {
	case PC_ELAPSED:
		((struct perf_ctr_elapsed *)handle)->time_start = sys_absolute_time();
		break;

	default:
		break;
	}
}

void perf_end(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (pce->time_start != 0) {
				int64_t elapsed = sys_absolute_time() - pce->time_start;

				if (elapsed < 0) {
					pce->event_overruns++;
				} else {

					pce->event_count++;
					pce->time_total += elapsed;

					if ((pce->time_least > (uint32_t)elapsed) || (pce->time_least == 0))
						pce->time_least = elapsed;

					if (pce->time_most < (uint32_t)elapsed)
						pce->time_most = elapsed;

					pce->mean = elapsed;

					pce->time_start = 0;
				}
			}
		}
		break;

	default:
		break;
	}
}

void perf_set(perf_counter_t handle, int64_t elapsed)
{
	if (handle == NULL) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
        struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

        if (elapsed < 0) {
            pce->event_overruns++;
        } else {

            pce->event_count++;
            pce->time_total += elapsed;

            if ((pce->time_least > (uint32_t)elapsed) || (pce->time_least == 0))
                pce->time_least = elapsed;

            if (pce->time_most < (uint32_t)elapsed)
                pce->time_most = elapsed;

            // maintain mean and variance of the elapsed time in seconds
            // Knuth/Welford recursive mean and variance of update intervals (via Wikipedia)
            float dt = elapsed / 1e6f;
            float delta_intvl = dt - pce->mean;
            pce->mean += delta_intvl / pce->event_count;
            pce->M2 += delta_intvl * (dt - pce->mean);

            pce->time_start = 0;
        }
    }
    break;

	default:
		break;
	}
}

void perf_cancel(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			pce->time_start = 0;
		}
		break;

	default:
		break;
	}
}

void perf_reset(perf_counter_t handle)
{
	if (handle == NULL)
		return;

	switch (handle->type) {
        case PC_COUNT:
            ((struct perf_ctr_count *)handle)->event_count = 0;
            break;

        case PC_ELAPSED: {
            struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
            pce->event_count = 0;
            pce->time_start = 0;
            pce->time_total = 0;
            pce->time_least = 0;
            pce->time_most = 0;
            pce->mean = 0;
            break;
        }

        case PC_INTERVAL: {
            struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
            pci->event_count = 0;
            pci->time_event = 0;
            pci->time_last = 0;
            pci->time_least = 0;
            pci->time_most = 0;
            pci->mean = 0;
            break;
        }
	}
}

void perf_print_counter(perf_counter_t handle)
{
    char buf[200];
	if (handle == NULL)
		return;

	switch (handle->type) {
	case PC_COUNT:
        sprintf(buf,"%llu events", (uint64_t)((struct perf_ctr_count *)handle)->event_count);
		rt_kprintf("%s: %s\n",
		       handle->name,
		       buf);
		break;

	case PC_ELAPSED: {
		struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
        
		float rms = sqrtf(pce->M2 / (pce->event_count-1));
        sprintf(buf,"%llu events, %llu overruns, %lluus elapsed, %lluus avg, min %lluus max %lluus %5.3fus rms",
            (unsigned long long)pce->event_count,
			(unsigned long long)pce->event_overruns,
			(unsigned long long)pce->time_total,
			(unsigned long long)pce->time_total / pce->event_count,
			(unsigned long long)pce->time_least,
			(unsigned long long)pce->time_most,
            (double)(1e6f * rms));
		rt_kprintf("%s: %s\n",
			handle->name,
			buf);
		break;
	}

	case PC_INTERVAL: {
		struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;

		float rms = sqrtf(pci->M2 / (pci->event_count-1));
        sprintf(buf,"%llu events, %lluus avg, min %lluus max %lluus %5.3fus rms",
            (unsigned long long)pci->event_count,
			(unsigned long long)(pci->time_last - pci->time_first) / pci->event_count,
			(unsigned long long)pci->time_least,
			(unsigned long long)pci->time_most,
            (double)(1e6f * rms));
		rt_kprintf("%s: %s\n",
			handle->name,
			buf);
		break;
	}

	default:
		break;
	}
}

uint32_t perf_event_count(perf_counter_t handle)
{
	if (handle == NULL)
		return 0;

	switch (handle->type) {
	case PC_COUNT:
		return ((struct perf_ctr_count *)handle)->event_count;

	case PC_ELAPSED: {
		struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
		return pce->event_count;
	}

	case PC_INTERVAL: {
		struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
		return pci->event_count;
	}

	default:
		break;
	}
	return 0;
}

void perf_print_all(void)
{
    rt_kprintf("----------------------------------------------------------------------------------------------\n");
    if(!rt_list_isempty(&perf_counters))
    {
        perf_counter_t handle = (perf_counter_t)perf_counters.next;

        while ((handle != NULL) && (handle != (perf_counter_t)&perf_counters)) {
            perf_print_counter(handle);
            handle = (perf_counter_t)handle->link.next;
        }
        rt_kprintf("----------------------------------------------------------------------------------------------\n");
    }
}

void perf_reset_all(void)
{
    if(!rt_list_isempty(&perf_counters))
    {
        perf_counter_t handle = (perf_counter_t)&perf_counters.next;

        while ((handle != NULL) && (handle != (perf_counter_t)&perf_counters)) {
            perf_reset(handle);
            handle = (perf_counter_t)&handle->link.next;
        }
    }
}

void perf_print(char *name)
{
    if(!rt_list_isempty(&perf_counters))
    {
        perf_counter_t handle = (perf_counter_t)perf_counters.next;

        while ((handle != NULL) && (handle != (perf_counter_t)&perf_counters)) {
            if (!strcmp(handle->name, name)) {
                perf_print_counter(handle);
            }
            handle = (perf_counter_t)handle->link.next;
        }
    }
}

#ifdef RT_USING_FINSH
#include <finsh.h>

void perf_all(void)
{
    perf_print_all();
}
FINSH_FUNCTION_EXPORT(perf_all, Printf All perf)
 
void perf_res(void)
{
    perf_reset_all();
}
FINSH_FUNCTION_EXPORT(perf_res, Reset All perf)

void perf_view(char *name)
{
    perf_print(name);
}
FINSH_FUNCTION_EXPORT(perf_view, Printf Any perf)

#endif
