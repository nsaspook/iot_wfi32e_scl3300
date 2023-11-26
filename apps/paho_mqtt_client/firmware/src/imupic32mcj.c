#include "imupic32mcj.h"

static uint32_t delay_freq = WDT_CAL;
volatile uint32_t NOPER = 0;

#ifdef __32MK0512MCJ048__
void qei_index_cb(QEI_STATUS, uintptr_t);

/*
 * on QEI index trigger function
 */
void qei_index_cb(QEI_STATUS status, uintptr_t context)
{

}
#endif

/*
 * configure the SPI port bit size for data transfers
 */
uint8_t set_imu_bits(void)
{
	uint8_t imu_bits = 32;

	return imu_bits;
}

/*
 * microsecond busy wait delay, 90 seconds MAX
 * used wdtwait in a cpu look so it must be calibrated
 */
void delay_us(uint32_t us)
{
	TP1_Set();
	// Convert microseconds us into how many delay ticks it will take
	us *= delay_freq;
	wdtdelay(us);
	TP1_Clear();
}

/*
 * start core-timer for delay_us
 */
void start_tick(void)
{

#ifdef __32MK0512MCJ048__
	TMR9_Start(); // IMU time-stamp counter
	QEI2_CallbackRegister(qei_index_cb, 0);
	QEI2_Start();
#endif
#ifdef __32MZ1025W104132__
	TMR2_Start(); // IMU time-stamp counter
#endif

#ifdef __32MZ1025W104132__
	cpu_serial_id = USERID & 0x1fffffff; // get CPU device 32-bit serial number and convert that to 29 - bit ID for CAN - FD
#else
	cpu_serial_id = DEVSN0 & 0x1fffffff; // get CPU device 32-bit serial number and convert that to 29 - bit ID for CAN - FD
#endif
}

void wdtdelay(const uint32_t delay)
{
	static uint32_t dcount;

	for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
		NOPER++;
	};
}