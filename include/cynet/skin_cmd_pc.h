#ifndef SKIN_CMD_PC_H
#define SKIN_CMD_PC_H

/*
 * This file contains the list of commands that can be exchanged between the PC
 * and the IHB.
 *
 * When adding commands, set its value to the maximum present plus 1 and make sure
 * not to change the other commands.
 */

enum SkinCmd
{
	SKIN_PATCH_CMD_MIN_CMD = 0x30,

	/**************************
	 * ST bootload operations *
	 **************************/
	/* send and program one row of ST module(s) */
	SKIN_PATCH_CMD_MODULE_PROGRAM = 0x30,
	/* end programming phase of ST modules */
	SKIN_PATCH_CMD_MODULE_PROGRAM_END = 0x31,

	/**************************
	 *  Execution operations  *
	 **************************/
	/* cold reboot the IHB.  It would go back to bootload phase */
	SKIN_PATCH_CMD_COLD_REBOOT = 0x32,
	/* warm reboot the IHB.  The skin application would restart */
	SKIN_PATCH_CMD_WARM_REBOOT = 0x33,
	/* cold reboot the STs.  The STs could be programmed again */
	SKIN_PATCH_CMD_MODULE_COLD_REBOOT = 0x34,
	/* warm reboot the STs.  The application on the ST would restart */
	SKIN_PATCH_CMD_MODULE_WARM_REBOOT = 0x35,

	/**************************
	 *     configurations     *
	 **************************/
	/* set grouping configuration of modules */
	SKIN_PATCH_CMD_SET_MODULE_GROUPING = 0x36,
	/* set whether SYNCs are required for each period, or IHB automatically handles the skin */
	SKIN_PATCH_CMD_SET_AUTO_SYNC = 0x37,
	/* if auto sync is selected, set the period of operation in milliseconds */
	SKIN_PATCH_CMD_SET_AUTO_SYNC_PERIOD = 0x38,
	/* SPI speed during normal operation (argument is valus in Mbps: 1, 2, 3, 4 or 8) */
	SKIN_PATCH_CMD_SET_COMM_SPEED = 0x39,
	SKIN_PATCH_CMD_GET_COMM_SPEED = 0x3A,

	/**************************
	 * application operations *
	 **************************/
	/* when sync is received, the IHB sends a command to the STs to start acquiring */
	SKIN_PATCH_CMD_SYNC = 0x3B,
	/* declare number of modules and number of sensors in each module */
	SKIN_PATCH_CMD_DECLARE = 0x3C,
	/* get details of the module, such as sensor groupings, which sensors are cut etc */
	SKIN_PATCH_CMD_MODULE_DETAILS = 0x3D,
	/* get IHB version */
	SKIN_PATCH_CMD_VERSION = 0x3E,
	/* get ST bootloader version */
	SKIN_PATCH_CMD_MODULE_BTLDR_VERSION = 0x3F,
	/* get ST application version */
	SKIN_PATCH_CMD_MODULE_APP_VERSION = 0x40,
	/* get unique id of the IHB */
	SKIN_PATCH_CMD_UID = 0x41,
	/* change CDC speed */
	SKIN_PATCH_CMD_MODULE_CDC_SPEED = 0x42,
	/* set cut sensors for a module */
	SKIN_PATCH_CMD_MODULE_SET_CUT_SENSORS = 0x47,

	/**************************
	 *  sensor data transfer  *
	 **************************/
	/*
	 * the sensor data will be divided up in fractions, since they don't all fit in one CANCMD message,
	 * and sending each module individually will also have considerable overhead.  The sensor data should
	 * thus be marked with two bytes in the beginning that say which module and which sensor in that module
	 * the first response in this chunk correspond to.
	 */
	SKIN_PATCH_CMD_SENSOR_DATA = 0x43,

	/**************************
	 *    debug operations    *
	 **************************/
	/* disable, disable excitation or enable CDC of a module, making it react to or ignore syncs */
	SKIN_PATCH_CMD_MODULE_CDC_CONTROL = 0x44,
	/* add a phase shift to triangle syncs equal to arg*(id-1)*32us */
	SKIN_PATCH_CMD_MODULE_PHASE_SHIFT = 0x45,

	/* other commands, if any */
	SKIN_PATCH_CMD_MAX_CMD = 0x46,
};

#endif
