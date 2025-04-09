#include "run_in_new_thread.h"

void RunInAThread(
        void (*entry)(void *parameter),
        rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick)
{
    rt_thread_t tmp_tid = rt_thread_create("tmp_t", entry, (void*)0, stack_size, priority, tick);
    if (tmp_tid != RT_NULL) rt_thread_startup(tmp_tid);
}

static void RunFuncInThread(void *parameter)
{
    void(*func_)(void);
    func_ = parameter;
    func_();
}

void RunInNewThread(
        void(*func_)(void),
        rt_uint32_t stack_size,
        rt_uint8_t  priority,
        rt_uint32_t tick)
{
    rt_thread_t tmp_tid = rt_thread_create("tmp_t", RunFuncInThread, (void*)func_, stack_size, priority, tick);
    if (tmp_tid != RT_NULL) rt_thread_startup(tmp_tid);
}
