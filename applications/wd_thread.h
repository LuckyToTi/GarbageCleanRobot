#ifndef APPLICATIONS_WD_THREAD_H_
#define APPLICATIONS_WD_THREAD_H_

#include <rtthread.h>

rt_thread_t* get_wd_tid();
void WaterdepthThreadEntry(void* parameter);
int32_t Get_Water_Depth(void);

#endif /* APPLICATIONS_WD_THREAD_H_ */
