#include <modbus_thread.h>
#include "config.h"
#include <rtthread.h>
#include "mb.h"
#include "mb_m.h"
#include "mbconfig.h"
#include "user_mb_app.h"
#include "userlib/bytetransformers.h"
#include "gyro_thread.h"
#include "hardware/led.h"
#include "userlib/bytetransformers.h"
#include "config.h"

#define LOG_TAG "modbus_t"
#define LOG_LVL LOG_LVL_INFO
#include <ulog.h>

extern USHORT usMRegInBuf[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_INPUT_NREGS];
extern USHORT usMRegHoldBuf[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_HOLDING_NREGS];

#define SLAVE_ADDR        (0x01)
#define PORT_NUM          (4)
#define PORT_BAUDRATE     (get_config()->u4_485_config.baundrate)
#define PORT_PARITY       (MB_PAR_NONE)
#define MB_POLL_CYCLE_MS  (20)

struct PowerManageData PowerManage_msg;
inline const struct PowerManageData* get_PowerManage_msg()
{
    return &PowerManage_msg;
}

static void ParsePowerManage()
{
    PowerManage_msg.cell1 = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 0],LSB);
    PowerManage_msg.cell2 = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 1],LSB);
    PowerManage_msg.cell3 = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 2],LSB);
    PowerManage_msg.cell4 = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 3],LSB);

    PowerManage_msg.Totalvoltage = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 4],LSB);
    PowerManage_msg.extertemp1 = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 5],LSB) / 100.0;
    PowerManage_msg.extertemp2 = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 6],LSB) / 100.0;
    PowerManage_msg.Dieremp = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 7],LSB);

    PowerManage_msg.cadccurrent = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 8],LSB);
    PowerManage_msg.Fu11chgCaP = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 9],LSB);
    PowerManage_msg.Remaincap = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 10],LSB);
    PowerManage_msg.RSOC = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 11],LSB);

    PowerManage_msg.CycleCount = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 12],LSB);
    PowerManage_msg.Packstatus = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 13],LSB);
    PowerManage_msg.Batterystatus = get_u16(&usMRegHoldBuf[SLAVE_ADDR - 1][0x01 + 14],LSB);
    LOG_D("%d, %d, %d, %d",PowerManage_msg.cell1,PowerManage_msg.cell2,PowerManage_msg.cell3,PowerManage_msg.cell4);
    LOG_D("%d, %d, %d, %d, ",PowerManage_msg.Totalvoltage,PowerManage_msg.extertemp1,PowerManage_msg.extertemp2,PowerManage_msg.Dieremp);
    LOG_D("%d, %d, %d, %d, ",PowerManage_msg.cadccurrent,PowerManage_msg.Fu11chgCaP,PowerManage_msg.Remaincap,PowerManage_msg.RSOC);
    LOG_D("%d, %d, %d",PowerManage_msg.CycleCount,PowerManage_msg.Packstatus,PowerManage_msg.Batterystatus);
}

static void send_thread_entry(void *parameter) {
	eMBMasterReqErrCode error_code = MB_MRE_NO_ERR;
	static rt_uint16_t error_count = 0;
	rt_thread_mdelay(1000);
	for (;;) {
		rt_thread_mdelay(1000 / 50);
		/* 03 功能码 连续读取保持寄存器多个地址的数据 */
		error_code = eMBMasterReqReadHoldingRegister(SLAVE_ADDR, 0x01, 15, 0);//PowerManage Request
		/* Record the number of errors */
		if (error_code != MB_MRE_NO_ERR) {
			error_count++;
			LOG_D("error_code = %d, error_count = %d", error_code, error_count);
			error_code = MB_MRE_NO_ERR;
		} else {
			ParsePowerManage();
		}
	}
}

//modbus线程控制块
rt_thread_t modbus_tid = RT_NULL;
inline rt_thread_t* get_modbus_tid() {
    return &modbus_tid;
}

static void mb_master_poll(void *parameter) {
	eMBMasterInit(MB_RTU, PORT_NUM, PORT_BAUDRATE, PORT_PARITY);
	eMBMasterEnable();

	while (1) {
		eMBMasterPoll();
		rt_thread_mdelay(MB_POLL_CYCLE_MS);
	}
}

void modbus_thread(void* parameter) {
	rt_thread_t mb_tid1 = RT_NULL, mb_tid2 = RT_NULL;
	static rt_uint8_t is_init = 0;

	if (is_init > 0) {
		rt_kprintf("modbus is running\n");
		return;
	}

	mb_tid1 = rt_thread_create("md_m_poll", mb_master_poll, (void*) 0, 1024 + 512, 10, 10);
	if (mb_tid1 != RT_NULL) rt_thread_startup(mb_tid1);
	mb_tid2 = rt_thread_create("md_m_send", send_thread_entry, (void*) 0, 2048, 11, 10);
	if (mb_tid2 != RT_NULL) rt_thread_startup(mb_tid2);

	is_init = 1;
	return;
}
