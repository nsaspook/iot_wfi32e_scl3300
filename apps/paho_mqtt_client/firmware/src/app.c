/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

/*
 * Testing example:  mosquitto_sub -t "mateq84/data/solar" -h 10.1.1.172
 */
#include <stdio.h>
#include "app.h"
#include "app_mqtt.h"
#include "timers.h"
#include "imupic32mcj.h"
#include "imu.h"
#include "sca3300.h"
#include "../../firmware/lcd_drv/lcd_drv.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
 */

#define BUFFER_SIZE	512
#define MAX_BBUF	BUFFER_SIZE-2

#define WDRV_PIC32MZW_MAC_ADDR_LEN              6

APP_DATA appData;
uint32_t counter = 0, ip_update = 0;
uint32_t count = 0;

static TCPIP_NET_HANDLE netHdl;

const char build_version[] = "MQTT WFI32E01 IoT     V1.000 ";
const char *build_date = __DATE__, *build_time = __TIME__;
void iot_version(void);

char buffer[BUFFER_SIZE];
bool wait = true, ip_show = true;
uint32_t board_serial_id = 0x35A, cpu_serial_id = 0x1957;
volatile double q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0; // quaternion of sensor frame relative to auxiliary frame
volatile double qa0 = 1.0, qa1 = 0.0, qa2 = 0.0, qa3 = 0.0; // quaternion of sensor frame relative to auxiliary frame

sSensorData_t accel = {
	.id = 1,
};

#ifdef SCA3300
/*
 * SCA3300-D01 instance
 */
imu_cmd_t imu0 = {
	.id = 3,
	.tbuf32[SCA3300_TRM] = SCA3300_SWRESET_32B,
	.online = false,
	.device = IMU_SCA3300, // device type
	.cs = IMU_CS, // chip select number
	.run = false,
	.crc_error = false,
	.log_timeout = SCA_LOG_TIMEOUT,
	.update = true,
	.features = false,
	.spi_bytes = 4,
	.op.info_ptr = &sca3300_version,
	.op.imu_set_spimode = &sca3300_set_spimode,
	.op.imu_getid = &sca3300_getid,
	.op.imu_getdata = &sca3300_getdata,
	.acc_range = range_15gl,
	.acc_range_scl = range_inc2,
	.angles = false,
	.locked = true,
	.warn = false,
	.down = false,
};
#endif

volatile uint16_t tickCount[TMR_COUNT];

TCPIP_SNTP_TIME_STAMP pTStamp;
TCPIP_SNTP_RESULT ntp_ret;
uint32_t pLastUpdate;
uint32_t pUTCSeconds, pMs;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
 */


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void)
{
	/* Place the App state machine in its initial state. */
	appData.state = APP_STATE_INIT;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void)
{
	TP1_Set();
	/* Check the application's current state. */
	switch (appData.state) {
		/* Application's initial state. */
	case APP_STATE_INIT:
	{
		bool appInitialized = true;
		APP_MQTT_Initialize();

		start_tick();

		lcd_version();
		init_lcd_drv(D_INIT);
		OledClearBuffer();
		eaDogM_WriteStringAtPos(0, 0, dis_buffer);
		imu0.op.info_ptr();
		eaDogM_WriteStringAtPos(2, 0, imu_buffer);
		iot_version();
		eaDogM_WriteStringAtPos(4, 0, imu_buffer);

		/*
		 * print the driver version
		 */

		imu0.op.imu_set_spimode(&imu0); // setup the IMU chip for SPI comms, X updates per second @ selected G range

		StartTimer(TMR_IMU, IMU_ID_DELAY);
		while (wait) {
			if (imu0.op.imu_getid(&imu0)) {
				wait = false;
				break;
			};
			LED_RED_Toggle();
			LED_GREEN_Toggle();
			if (TimerDone(TMR_IMU)) {
				LED_RED_On();
				LED_GREEN_Off();
				wait = false;
				snprintf(buffer, MAX_BBUF, "Unable to get IMU ID");
				eaDogM_WriteStringAtPos(11, 0, buffer);
				break;
			}
		}


		if (appInitialized) {
			appData.state = APP_STATE_SERVICE_TASKS;
			snprintf(buffer, MAX_BBUF, "Starting WFI");
			eaDogM_WriteStringAtPos(7, 0, buffer);
			snprintf(buffer, MAX_BBUF, "USERID 0x%X, IMUID 0x%X", cpu_serial_id, board_serial_id);
			eaDogM_WriteStringAtPos(14, 0, buffer);
		}
		OledUpdate();
		break;
	}

	case APP_STATE_SERVICE_TASKS:
		appData.state = APP_STATE_IMU;
		break;

	case APP_STATE_IMU:
		/*
		 * service the IMU data
		 */
		if (imu0.update) {
			imu0.op.imu_getdata(&imu0); // read data from the chip
			imu0.update = false;
			getAllData(&accel, &imu0); // convert data from the chip
			q0 = accel.x;
			q1 = accel.y;
			q2 = accel.z;
			qa0 = accel.xa;
			qa1 = accel.ya;
			qa2 = accel.za;

			ntp_ret = TCPIP_SNTP_TimeGet(&pUTCSeconds, &pMs);
			if (ntp_ret == SNTP_RES_OK) {
				TCPIP_SNTP_TimeStampGet(&pTStamp, &pLastUpdate);
				snprintf(buffer, MAX_BBUF, "SNTP UNIX time %d.%d        ", pUTCSeconds, pMs);
				UART3_Write((uint8_t*) buffer, strlen(buffer));
			} else {
				snprintf(buffer, MAX_BBUF, "SNTP, Waiting %d  ", ntp_ret);
				UART3_Write((uint8_t*) buffer, strlen(buffer));
			}
			eaDogM_WriteStringAtPos(9, 0, buffer);
			if (ip_show) {
				/*
				 * net functions in  tcpip_manager.h
				 */
				netHdl = TCPIP_STACK_NetHandleGet("PIC32MZW1");
				IPV4_ADDR ipAddr;
				ipAddr.Val = TCPIP_STACK_NetAddress(netHdl);
				if (ipAddr.Val) {
					snprintf(buffer, MAX_BBUF, "STA IP:%d.%d.%d.%d           ", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
					eaDogM_WriteStringAtPos(8, 0, buffer);
					ip_show = false;
				} else {
					snprintf(buffer, MAX_BBUF, "Waiting for IP Address ");
					eaDogM_WriteStringAtPos(8, 0, buffer);
				}
			} else {
				if (ip_update++ > IP_UPDATE_SPEED) {
					ip_update = 0;
					ip_show = true;
				}
			}

			OledUpdate();
			appData.state = APP_STATE_MQTT;
		}
		break;

	case APP_STATE_MQTT:
		/*
		 * convert IMU data to JSON string for MQTT publishing
		 */
		if (counter++ >= IMU_UPDATE_SPEED) {
			/*
			 * format data to JSON using printf formatting
			 */
			snprintf(buffer, MAX_BBUF, "{\r\n     \"name\": \"%s\",\r\n     \"Wsequence\": %u,\r\n     \"WUTC\": %u,\r\n     \"WUTCMs\": %u,\r\n     \"WX\": %f,\r\n     \"WY\": %f,\r\n     \"WZ\": %f,\r\n     \"WXA\": %f,\r\n     \"WYA\": %f,\r\n     \"WZA\": %f,\r\n     \"build_date\": \"%s\",\r\n     \"build_time\": \"%s\"\r\n}",
				build_version, count++, pMs, pUTCSeconds, q0, q1, q2, qa0, qa1, qa2, build_date, build_time);
			APP_MQTT_PublishMsg(buffer);
			counter = 0;
			imu0.update = true;
			/*
			 * send updates to the GLCD screen
			 */
			snprintf(buffer, MAX_BBUF, "X %7.3f,Y %7.3f,Z %7.3f", q0, q1, q2);
			eaDogM_WriteStringAtPos(11, 0, buffer);
			snprintf(buffer, MAX_BBUF, "XA%7.3f,YA%7.3f,ZA%7.3f", qa0, qa1, qa2);
			eaDogM_WriteStringAtPos(12, 0, buffer);
			snprintf(buffer, MAX_BBUF, "TOPIC %s", SYS_MQTT_DEF_PUB_TOPIC_NAME);
			eaDogM_WriteStringAtPos(15, 0, buffer);
			appData.state = APP_STATE_SERVICE_TASKS;
		}
		break;

		/* The default state should never be executed. */
	default:
		/* TODO: Handle error in application's state machine. */
		appData.state = APP_STATE_INIT;
		break;
	}
	TP1_Clear();
}

void iot_version(void)
{
	snprintf(imu_buffer, MAX_FBUF, "%s %s %s", build_version, build_date, build_time);
}
/*******************************************************************************
 End of File
 */
