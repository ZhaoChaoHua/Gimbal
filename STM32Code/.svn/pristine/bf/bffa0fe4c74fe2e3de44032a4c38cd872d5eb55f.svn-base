/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Gimbal.h"
#include <stdio.h>

#define SCHEDULER_DEFAULT_LOOP_RATE 2000

int8_t Scheduler::current_task = -1;

struct rt_event Scheduler::Sys_Event;

// constructor
Scheduler::Scheduler(void)
{
    _loop_rate_hz = SCHEDULER_DEFAULT_LOOP_RATE;
}

// initialise the scheduler
void Scheduler::init(const Scheduler::Task *tasks, uint8_t num_tasks)
{
    _tasks = tasks;
    _num_tasks = num_tasks;
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _tick_counter = 0;
    rt_event_init(&Scheduler::Sys_Event,"sys",RT_IPC_FLAG_FIFO);
}

// one tick has passed
void Scheduler::tick(void)
{
    _tick_counter++;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void Scheduler::run(uint32_t time_available)
{
    uint32_t run_started_count = SysTick->VAL;
    uint32_t now = run_started_count;
    
    for (uint8_t i=0; i<_num_tasks; i++) {
        uint16_t dt = _tick_counter - _last_run[i];
        uint16_t interval_ticks = _loop_rate_hz / _tasks[i].rate_hz;
        if (interval_ticks < 1) {
            interval_ticks = 1;
        }
        if (dt >= interval_ticks) {
            // this task is due to run. Do we have enough time to run it?
            _task_time_allowed = _tasks[i].max_time_micros;

            if (dt >= interval_ticks*2) {
                // we've slipped a whole run of this task!
            }

            if (_task_time_allowed <= time_available) {
                // run it
                _task_time_started = now;
                current_task = i;
                rt_event_send(&Scheduler::Sys_Event, _tasks[i].events);
                current_task = -1;

                // record the tick counter when we ran. This drives
                // when we next run the event
                _last_run[i] = _tick_counter;

                // work out how long the event actually took
                now = SysTick->VAL;
                // 将计数值转换为时间
                uint32_t time_taken = (now > _task_time_started)?(SysTick->LOAD + _task_time_started - now) : (_task_time_started - now);
                time_taken = time_taken / US_T;
                if (time_taken > _task_time_allowed) {
                    // the event overran!
                }
                if (time_taken >= time_available) {
                    goto update_spare_ticks;
                }
                time_available -= time_taken;
            }
        }
    }

    // update number of spare microseconds
    _spare_micros += time_available;

update_spare_ticks:
    _spare_ticks++;
    if (_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }
}

/*
  return number of micros until the current task reaches its deadline
 */
uint16_t Scheduler::time_available_usec(void)
{
    uint32_t dt = (SysTick->VAL > _task_time_started)?(SysTick->LOAD + _task_time_started - SysTick->VAL) : (_task_time_started - SysTick->VAL);
    dt = dt / US_T;
    if (dt > _task_time_allowed) {
        return 0;
    }
    return _task_time_allowed - dt;
}

/*
  calculate load average as a number from 0 to 1
 */
float Scheduler::load_average(uint32_t tick_time_usec) const
{
    if (_spare_ticks == 0) {
        return 0.0f;
    }
    uint32_t used_time = tick_time_usec - (_spare_micros/_spare_ticks);
    return used_time / (float)tick_time_usec;
}
