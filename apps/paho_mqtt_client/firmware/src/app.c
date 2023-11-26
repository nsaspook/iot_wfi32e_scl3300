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

#include <stdio.h>
#include "app.h"
#include "app_mqtt.h"
#include "imupic32mcj.h"
#include "imu.h"
#include "sca3300.h"

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

APP_DATA appData;
uint32_t counter = 0;
uint32_t count = 0;

const char build_version[] = "MQTT Test PIC32mz_w1";
const char *build_date = __DATE__, *build_time = __TIME__;

char buffer[256], opbuffer[24];
bool wait = true;
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



	/* TODO: Initialize your application's state machine and other
	 * parameters.
	 */
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void)
{

	/* Check the application's current state. */
	switch (appData.state) {
		/* Application's initial state. */
	case APP_STATE_INIT:
	{
		bool appInitialized = true;
		APP_MQTT_Initialize();

		start_tick();

		/*
		 * print the driver version
		 */
		imu0.op.info_ptr(); // print driver version on the serial port
		printf(imu_buffer);
		imu0.op.imu_set_spimode(&imu0); // setup the IMU chip for SPI comms, X updates per second @ selected G range

		while (wait) {
			if (imu0.op.imu_getid(&imu0)) {
				wait = false;
				break;
			};
			LED_RED_Toggle();
			LED_GREEN_Toggle();
		}


		if (appInitialized) {

			appData.state = APP_STATE_SERVICE_TASKS;
		}
		break;
	}

	case APP_STATE_SERVICE_TASKS:
	{

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
		}

		if (counter++ >= 20000) {
			/*
			 * format data to JSON
			 */
			snprintf(buffer, 250, "{\r\n     \"name\": \"%s\",\r\n     \"X\": %f,\r\n     \"Y\": %f,\r\n     \"Z\": %f,\r\n     \"XA\": %f,\r\n     \"YA\": %f,\r\n     \"ZA\": %f,\r\n     \"build_date\": \"%s\",\r\n     \"build_time\": \"%s\"\r\n}",
				build_version, q0, q1, q2, qa0, qa1, qa2, build_date, build_time);

			APP_MQTT_PublishMsg(buffer);
			counter = 0;
			imu0.update = true;
		}
		break;
	}

		/* TODO: implement your application state machine.*/


		/* The default state should never be executed. */
	default:
	{
		/* TODO: Handle error in application's state machine. */
		break;
	}
	}
}


/*******************************************************************************
 End of File
 */
