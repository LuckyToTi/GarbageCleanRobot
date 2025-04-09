#ifndef __RUN_IN_NEW_THREAD_H_
#define __RUN_IN_NEW_THREAD_H_

#include "rtthread.h"

void RunInAThread(
        void (*entry)(void *parameter),
        rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick);

void RunInNewThread(
        void(*func_)(void),
        rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick);

#endif  // __RUN_IN_NEW_THREAD_H_
