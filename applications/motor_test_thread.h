#ifndef APPLICATIONS_MOTOR_TEST_THREAD_H_
#define APPLICATIONS_MOTOR_TEST_THREAD_H_

#include <rtthread.h>

rt_thread_t* get_motor_test_tid();
void MotorTestThreadEntry(void* parameter);

#endif /* APPLICATIONS_MOTOR_TEST_THREAD_H_ */
