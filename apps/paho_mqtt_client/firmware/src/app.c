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
#include "config/pic32mz_w1_curiosity/system/mqtt/sys_mqtt_paho.h"
#include "gfx.h"
#include "../../firmware/cjson/cJSON.h"

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

#define BUFFER_SIZE	2048
#define MAX_BBUF	BUFFER_SIZE-2

#define WDRV_PIC32MZW_MAC_ADDR_LEN              6

APP_DATA appData;
uint32_t counter = 0, ip_update = 0;
uint32_t count = 0;

static TCPIP_NET_HANDLE netHdl;

const char build_version[] = "MQTT WFI32E01 IoT     V1.100 ";
const char *build_date = __DATE__, *build_time = __TIME__;
char id_string[128], id_client[128], id_mqtt[128];
void iot_version(void);
int32_t APP_MQTT_PublishMsg_local(char *);
static void add_mqtt_id(char *);
cJSON *json;

char buffer[BUFFER_SIZE];
bool wait = true, ip_show = true;
uint32_t board_serial_id = 0x35A, cpu_serial_id = 0x1957;
volatile double q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0; // quaternion of sensor frame relative to auxiliary frame
volatile double qa0 = 1.0, qa1 = 0.0, qa2 = 0.0, qa3 = 0.0; // quaternion of sensor frame relative to auxiliary frame

extern SYS_MQTT_Handle g_asSysMqttHandle[1];
#define MQTT_DEVICE	"mateq84wfi"	// client base-name

extern SYS_MODULE_OBJ g_sSysMqttHandle;
#define SYS_MQTT_DEF_PUB_TOPIC_NAME_LOCAL	"mateq84/data/imu"

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
			snprintf(buffer, MAX_BBUF, "USERID 0x%X, IMUID 0x%X", cpu_serial_id, board_serial_id);
			eaDogM_WriteStringAtPos(14, 0, buffer);
			snprintf(id_string, 110, "IMUID 0x%X", board_serial_id);
			snprintf(id_mqtt, 110, "%X", board_serial_id);
			/*
			 * make unique client id from IMU serial number
			 */
			snprintf(id_client, 110, "%s%X", MQTT_DEVICE, board_serial_id);
			strcpy((char *) &g_asSysMqttHandle[0].sCfgInfo.sBrokerConfig.clientId, id_client);
			snprintf(buffer, MAX_BBUF, "Starting WFI %s", id_client);
			eaDogM_WriteStringAtPos(7, 0, buffer);
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
				snprintf(buffer, MAX_BBUF, "SNTP UNIX time %d.%d ", pUTCSeconds, pMs);
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
			ADCHS_ChannelConversionStart(ADCHS_CH21);
			/*
			 * format data to JSON using printf formatting
			 * create the json formatted data
			 */
			json = cJSON_CreateObject();
			add_mqtt_id("Wname"); // results in global buffer variable
			cJSON_AddStringToObject(json, buffer, build_version);
			add_mqtt_id("Wsequence");
			cJSON_AddNumberToObject(json, buffer, count);
			add_mqtt_id("WUTC");
			cJSON_AddNumberToObject(json, buffer, pUTCSeconds);
			add_mqtt_id("WUTCMs");
			cJSON_AddNumberToObject(json, buffer, pMs);
			add_mqtt_id("WX");
			cJSON_AddNumberToObject(json, buffer, q0);
			add_mqtt_id("WY");
			cJSON_AddNumberToObject(json, buffer, q1);
			add_mqtt_id("WZ");
			cJSON_AddNumberToObject(json, buffer, q2);
			add_mqtt_id("WXA");
			cJSON_AddNumberToObject(json, buffer, qa0);
			add_mqtt_id("WYA");
			cJSON_AddNumberToObject(json, buffer, qa1);
			add_mqtt_id("WZA");
			cJSON_AddNumberToObject(json, buffer, qa2);
			add_mqtt_id("Wbuild_date");
			cJSON_AddStringToObject(json, buffer, build_date);
			add_mqtt_id("Wbuild_time");
			cJSON_AddStringToObject(json, buffer, build_time);
			/*
			 * get the data string for publishing
			 */
			char *json_str = cJSON_Print(json);
			//			eaDogM_WriteStringAtPos(6, 0, json_str);
			APP_MQTT_PublishMsg_local(json_str);
			cJSON_free(json_str);
			cJSON_Delete(json);

			count++;
			counter = 0;
			imu0.update = true;
			/*
			 * send updates to the GLCD screen
			 */
			snprintf(buffer, MAX_BBUF, "X %7.3f,Y %7.3f,Z %7.3f", q0, q1, q2);
			eaDogM_WriteStringAtPos(11, 0, buffer);
			snprintf(buffer, MAX_BBUF, "XA%7.3f,YA%7.3f,ZA%7.3f", qa0, qa1, qa2);
			eaDogM_WriteStringAtPos(12, 0, buffer);
			snprintf(buffer, MAX_BBUF, "TOPIC %s", SYS_MQTT_DEF_PUB_TOPIC_NAME_LOCAL);
			eaDogM_WriteStringAtPos(15, 0, buffer);
			appData.state = APP_STATE_SERVICE_TASKS;

			/*
			 * visualize tilt values
			 */
			OledSetDrawColor(0); // clear all tilt display lines
			line_rot(0, GFX_Y_X, ccolOledMax, GFX_Y_X);
			line_rot(0, GFX_Y_Y, ccolOledMax, GFX_Y_Y);
			line_rot(0, GFX_Y_Z, ccolOledMax, GFX_Y_Z);

			OledSetDrawColor(1); // redraw tilt lines
			line_rot(GFX_X_MID, GFX_Y_X, GFX_X_MID + (int) qa0, GFX_Y_X);
			line_rot(GFX_X_MID, GFX_Y_Y, GFX_X_MID + (int) qa1, GFX_Y_Y);
			line_rot(GFX_X_MID, GFX_Y_Z, GFX_X_MID + (int) qa2, GFX_Y_Z);

			{
				int millivolt, temp, temp_raw;

				if (ADCHS_ChannelResultIsReady(ADCHS_CH21)) {
					millivolt = ADCHS_ChannelResultGet(ADCHS_CH21)* 3100 >> 12;
					temp_raw = millivolt;
					temp = (millivolt - 500) * (125 + 40) / (1300 - 500); /* 800 millivolt from -40 to +125 */
					temp = temp - 40; /* offset temperature */
					snprintf(buffer, MAX_BBUF, "WFI32 TEMP %dC Raw ADC %d  ", temp, temp_raw);
					eaDogM_WriteStringAtPos(13, 0, buffer);
				}
			}
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

int32_t APP_MQTT_PublishMsg_local(char *message)
{
	SYS_MQTT_PublishTopicCfg sMqttTopicCfg;
	int32_t retVal = SYS_MQTT_FAILURE;

	strcpy(sMqttTopicCfg.topicName, SYS_MQTT_DEF_PUB_TOPIC_NAME_LOCAL);
	sMqttTopicCfg.topicLength = strlen(SYS_MQTT_DEF_PUB_TOPIC_NAME_LOCAL);
	sMqttTopicCfg.retain = SYS_MQTT_DEF_PUB_RETAIN;
	sMqttTopicCfg.qos = SYS_MQTT_DEF_PUB_QOS;

	retVal = SYS_MQTT_Publish(g_sSysMqttHandle,
		&sMqttTopicCfg,
		message,
		strlen(message));
	if (retVal != SYS_MQTT_SUCCESS) {
		SYS_CONSOLE_PRINT("\nPublish_PeriodicMsg(): Failed (%d)\r\n", retVal);
	}
	return retVal;
}

/*
 * Append id in front of name string
 */
static void add_mqtt_id(char * name)
{
	strcpy(buffer, id_mqtt);
	strncat(buffer, name, MAX_BBUF);
}
/*******************************************************************************
 End of File
 */
