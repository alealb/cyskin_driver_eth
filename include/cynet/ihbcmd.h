#ifndef IHBCMD_H
#define IHBCMD_H

/* generic */

#define CY_CAN_SWITCH_ST_EXT		18		/* need to shift by 18 places to get the standard or ext asm */
#define CY_CAN_ID				0x1FFFFFFF	/* mask of the 29 bits of the CAN identifier */
#define CY_CAN_ID_BITS			0x03FFFFFF	/* 26 bits of ID is actual ID */
#define CY_CAN_ASM_BITS			0x1C000000	/* 3 bits of ID is for assembly management */
#define CY_CAN_ASM_FIRST		0x04000000	/* First packet of a series */
#define CY_CAN_ASM_MID			0x08000000	/* Some packet in the middle of a series */
#define CY_CAN_ASM_LAST			0x10000000	/* Last packet of a series */
#define CY_CAN_ASM_SINGLE		(CY_CAN_ASM_FIRST | CY_CAN_ASM_LAST)
							/* A single packet is both first and last , it's 0x14000000 */

#define CY_CAN_ID_INVALID		0x00000000
#define CY_CAN_ID_BROADCAST		0x03FFFFFF

/* bootloader related (commands taken from Cypress bootloader datasheet) */

#define CY_BTLDR_SOP(p)			p[0]		/* Start byte of packet */
#define CY_BTLDR_CMD(p)			p[1]		/* Command byte of packet */
#define CY_BTLDR_RSP(p)			p[1]		/* Response byte of packet */
#define CY_BTLDR_SIZE_LO(p)		p[2]		/* Packet size low byte */
#define CY_BTLDR_SIZE_HI(p)		p[3]		/* Packet size high byte */
#define CY_BTLDR_DATA(p, o)		p[4 + (o)]	/* Packet data byte at offset `o` */
#define CY_BTLDR_CHK_LO(p)		p[4 + CY_BTLDR_SIZE(p)]	/* Packet checksum low byte */
#define CY_BTLDR_CHK_HI(p)		p[5 + CY_BTLDR_SIZE(p)]	/* Packet checksum high byte */
#define CY_BTLDR_EOP(p)			p[6 + CY_BTLDR_SIZE(p)]	/* End of packet byte with data length `l` */
#define CY_BTLDR_MIN_PKT_SIZE		7		/* The minimum number of bytes in a packet */
#define CY_BTLDR_PKT_SIZE(l)		(7 + (l))	/* The number of bytes in the packet with data length `l` */
#define CY_BTLDR_TAIL_SIZE		3		/* Size of checksum bytes + EOF */
#define CY_BTLDR_SIZE(p)		((uint16_t)(CY_BTLDR_SIZE_HI(p) << 8 | CY_BTLDR_SIZE_LO(p)))
#define CY_BTLDR_CHK(p)			((uint16_t)(CY_BTLDR_CHK_HI(p) << 8 | CY_BTLDR_CHK_LO(p)))

#define CY_BTLDR_SOP_DATA		0x01
#define CY_BTLDR_EOP_DATA		0x17

/* extensions to bootloader commands */
#define CY_CMD_BTLDR_PING 			0x91
#define CY_CMD_BTLDR_VERSION 		0x92
#define CY_CMD_BTLDR_GO_APP			0x93
#define CY_CMD_BTLDR_GET_SUBNETS	0x94
#define CY_CMD_BTLDR_GET_NODE		0x95
#define CY_CMD_BTLDR_START_HOST_PROGRAMMMING	0x96

/* application related */

#define CY_APP_CMD(p)			p[0]		/* Command byte of packet */
#define CY_APP_RSP(p)			p[0]		/* Response byte of packet */
#define CY_APP_SIZE(p)			p[1]		/* Packet size byte */
#define CY_APP_DATA(p, o)		p[2 + (o)]	/* Packet data byte at offset `o` */
#define CY_APP_CHK(p)			p[2 + CY_APP_SIZE(p)]	/* Packet checksum byte with data length `l` */
#define CY_APP_MIN_PKT_SIZE		3		/* The minimum number of bytes in a packet */
#define CY_APP_PKT_SIZE(l)		(3 + (l))	/* The number of bytes in the packet with data length `l` */

/*
 * if at the location of a command, a value below IHB_CMD_MIN_CMD is given, it is taken as
 * a sequence number instead.  This means that all commands in this file must have a greater
 * value than IHB_CMD_MIN_CMD (or equal to it).  Alternatively, commands larger than or equal to
 * IHB_CMD_MAX_CMD are taken as extensions and are left to the user to handle
 */
enum IHBCmd
{
	CY_CMD_MIN_CMD = 0x10,

	CY_CMD_DISCOVERY_PING = CY_CMD_MIN_CMD,	/* send unique id back */
	CY_CMD_DISCOVERY_RECV_ID,		/* receive a smaller id for CAN communication */
	CY_CMD_PING,				/* ping! */
	CY_CMD_ERROR,				/* error code (used as response) */
	/* other commands, if any */
	CY_CMD_MAX_CMD,
	/* place for extensions */
	CY_CMD_EXTENSION_START = 0x30,		/* for extensions, start from this code */
};

/* error codes for commands */
#define CY_CMD_ERROR_FAIL 0xE1		/* command failed */
#define CY_CMD_ERROR_NOSUP 0xE2		/* command not supported */
#define CY_CMD_ERROR_INVAL 0xE3		/* invalid argument */
#define CY_CMD_ERROR_NOCMD 0xE4		/* packet without command (command number below minimum) */
#define CY_CMD_ERROR_NOARG 0xE5		/* some argument missing */
#define CY_CMD_ERROR_FORMAT 0xE6	/* bad packet format */
#define CY_CMD_ERROR_CRC 0xE7		/* packet CRC error */
#define CY_CMD_ERROR_NOBUFS 0xE8	/* no buffer space for sending messages */
#define CY_CMD_ERROR_PHASE 0xE9		/* bad phase (e.g. bootload during application phase) */
#define CY_CMD_ERROR_BUSY 0xEA		/* must wait until previous operation finishes */
#define CY_CMD_ERROR_IO 0xEB		/* error communicating with a component */

#endif
