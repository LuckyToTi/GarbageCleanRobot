/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-19     24144       the first version
 */
#ifndef APPLICATIONS_YESENS_THREAD_H_
#define APPLICATIONS_YESENS_THREAD_H_


#include "rtthread.h"
#include "stm32f4xx.h"
#include "userlib/analysis_data.h"

rt_thread_t* get_yesens_tid();
void YsensThreadEntry(void* parameter);

const protocol_info_t* get_yesens_data();


#endif /* APPLICATIONS_YESENS_THREAD_H_ */
