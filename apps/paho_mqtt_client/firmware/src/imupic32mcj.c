#include "imupic32mcj.h"
#include "timers.h"

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
 * does nothing for this port
 */
uint8_t set_imu_bits(void)
{
	uint8_t imu_bits = 32;

#ifdef SCA3300
	imu_bits = 32;
#endif

	return imu_bits;
}

/*
 * microsecond busy wait delay, 90 seconds MAX
 * uses wdtdelay in a cpu loop so it must be calibrated using delay_freq
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
 * start time-stamp counter
 */
void start_tick(void)
{
	TMR4_CallbackRegister(timer_ms_tick, 0);
	TMR4_Start(); // software timers counter

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