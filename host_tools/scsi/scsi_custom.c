/* Simple program to send custom scsi commands
 *
 * see http://tldp.org/HOWTO/archived/SCSI-Programming-HOWTO/ for details about
 * scsi interface.
 *
 * This code is not rigorously tested for suitability for any particular
 * application.  For example, there are known endianess issues.  Use at your own
 * risk.
 *
 * To compile: gcc -o scsi_custom scsi_custom.c; cp scsi_custom ~/bin
 */

#include <stddef.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <scsi/sg.h>

int fd;

/* Definitions of commands that the Universal PCApp expects to be able to send.
 *
 * These settings are from:
 * http://source.leapfrog.com/Middleware/Universal/UPCApp/TRUNK/DeviceIO/Src/ \
 * UsbMassStorage/Src/Device/Include/DeviceTypes.h
 *
 * Notes:
 * -- kDeviceCmdSetSetting is not supported over SCSI.
 * -- kDeviceCmdClearUploadData is not supported by lightning.
 * -- I added the standard INQUIRY command for testing purposes.
 * -- I added a undefined command for testing proper error responses.
 */
typedef unsigned char tDeviceCmd;
enum {
	kDeviceCmdInvalid = 0,
	kDeviceCmdInquiry = 0x12,
	kDeviceCmdLockDevice = 0xC1,
	kDeviceCmdUnlockDevice,
	kDeviceCmdGetSetting,
	kDeviceCmdSetSetting,
	kDeviceCmdClearUploadData,
	kDeviceCmdDisconnectOk,
	kNumDeviceCommands,
	kDeviceCmdUndefined = 0xFF
};

/* Definitions of settings that can be retrieved using the kDeviceCmdGetSetting
 * command.  Based on ongoing discussions, the only settings that can actually
 * be retrieved over SCSI are kSettingBatteryLevel and kSettingRTCCounter.
 */
typedef unsigned char tDeviceSetting;
enum {
	kSettingRTCOffset 		= 0,
	kSettingRTCCounter		= 1,
	kSettingBatteryLevel,
	kSettingSerialNumber,
	kSettingUndefined = 0xff,
};

/* The command formats are specified here.  Linux really wants the custom
 * commands to be 10 bytes.  I'm not exactly sure if this is a nuance of Linux,
 * or if it is specified by SCSI.
 */
struct inquiry_cmd {
	tDeviceCmd opcode;

	unsigned evpd:1;
	unsigned reserved0:4;
	unsigned lun_num:1;

	unsigned char page_code;
	unsigned char reserved1;
	unsigned char alloc_len;
	unsigned char control;
} __attribute__((packed));

struct custom_cmd {
	tDeviceCmd opcode;
	tDeviceSetting setting; /* only used for kDeviceCmdGetSetting */
	unsigned char unused[8];
} __attribute__((packed));

/* AT THIS TIME NO CARE IS TAKEN WITH REGARD TO THE ENDIANNESS OF CUSTOM
 * REPLYS!!  THEY ARE REQUIRED TO BE LITTLE ENDIAN, SO IF YOU'RE RUNNING THIS
 * CODE ON A BIG ENDIAN MACHINE, YOU MUST ALTER THE CODE!!
 *
 * The INQUIRY command has a reply of 96 bytes.
 */
struct inquiry_reply {
	unsigned peripheral_id:5;
	unsigned peripheral_qual:3;

	unsigned device_type_mod:7;
	unsigned rmb:1;

	unsigned ansi_version:3;
	unsigned ecma_version:2;
	unsigned iso_version:2;

	unsigned response_data_format:4;
	unsigned reserved0:2;
	unsigned trmiop:1;
	unsigned aenc:1;

	unsigned char additional_len;
	unsigned char reserved1;
	unsigned char reserved2;

	unsigned stfre:1;
	unsigned cmdque:1;
	unsigned reserved3:1;
	unsigned linked:1;
	unsigned sync:1;
	unsigned wbus16:1;
	unsigned wbus32:1;
	unsigned reladr:1;

	unsigned char vendor_id[8];
	unsigned char prod_id[16];
	unsigned char prod_level[4];
	unsigned char vendor_area[20];
	unsigned char reserved4[40];
} __attribute__((packed));

struct rtc_reply {
	unsigned long value;
} __attribute__((packed));

struct battery_reply {
	unsigned char value;
} __attribute__((packed));

char serial[256];

typedef unsigned char tBatteryLevelUSB;
enum {
	kBtLvlUnknown = 0,
	kBtLvlLow,
	kBtLvlMedium,
	kBtLvlHigh,
};

#define MAX_REPLY_LEN 4096

/* Print the usage message */
static void print_help(void)
{
	printf("\nscsi_custom [-c cmd [-v val]] <device>\n\n");
	printf("<device> is a generic scsi device such as /dev/sg2.  To figure out\n"
	       "which device you need, you can step through them using the inquiry\n"
	       "command, or you can use scsiinfo.\n\n");
	printf("options:\n");
	printf(" -h            Print this message.\n");
	printf(" -c cmd        Valid commands are inquiry, lock, unlock, get, disconnect,\n");
	printf("               and undefined.  If not specified, inquiry is performed.\n");
	printf(" -v val        The set and get commands require a value.  Valid values\n");
	printf("               include battery and rtc\n");
}

void print_reply(int cmd, int setting, unsigned char *reply)
{
	struct inquiry_reply *ir;
	struct sg_header *sgh = (struct sg_header *)reply;
	struct battery_reply *br;
	struct rtc_reply *rtcr;
	char vendor[9];
	char product[17];
	char *s;
	if(sgh->target_status) {
		printf("Command failed with status %d.\n", sgh->target_status);
		return;
	}

	switch(cmd) {
	case kDeviceCmdInquiry:
		ir = (struct inquiry_reply *)(reply + sizeof(struct sg_header));
		/* I'm not sure if SCSI requires the strings to be null-terminated, so I
		 * do it here.
		 */
		memcpy(vendor, &ir->vendor_id[0], 8);
		vendor[8] = 0;
		memcpy(product, &ir->prod_id[0], 16);
		product[16] = 0;
		printf("Vendor: %s\n", vendor);
		printf("Product: %s\n", product);
		break;

	case kDeviceCmdLockDevice:
	case kDeviceCmdUnlockDevice:
	case kDeviceCmdDisconnectOk:
		printf("Command succeeded.\n");
		break;

	case kDeviceCmdGetSetting:
		switch (setting) {
		case kSettingBatteryLevel:
			br = (struct battery_reply *)(reply + sizeof(struct sg_header));
			printf("0x%x\n", br->value);
			break;

		case kSettingRTCCounter:
			rtcr = (struct rtc_reply *)(reply + sizeof(struct sg_header));
			printf("0x%x\n", rtcr->value);
			break;

		case kSettingSerialNumber:
			s = (char *)(reply + sizeof(struct sg_header));
			printf("%s\n", s);
			break;

		case kSettingUndefined:
			s = (char *)(reply + sizeof(struct sg_header));
			printf("%s\n", s);
			break;

		default:
			printf("Unrecognized setting 0x%x\n", setting);
		}
		break;

	default:
		printf("Unrecognized command 0x%x\n", cmd);
	}
}

/* process a complete scsi cmd using the generic scsi interface. */
static int cmd_helper(unsigned cmd_len, unsigned char *cmd_buf,
		      unsigned rep_len, unsigned char *rep_buf)
{
	int status = 0;
	struct sg_header *sg_hd;

	/* generic scsi device header construction */
	sg_hd = (struct sg_header *)cmd_buf;
	memset(sg_hd, 0, sizeof(struct sg_header));
	memset(rep_buf, 0, sizeof(struct sg_header));
	sg_hd->reply_len = sizeof(struct sg_header) + rep_len;
	sg_hd->twelve_byte = cmd_len == 12;
	sg_hd->result = 0;

	/* send command */
	status = write(fd, cmd_buf, sizeof(struct sg_header) + cmd_len);
	if(status < 0 ||
	   status != sizeof(struct sg_header) + cmd_len || 
	   sg_hd->result) {
		/* some error happened */
		fprintf(stderr, "write result = 0x%x cmd = 0x%x\n",
			sg_hd->result, cmd_buf[sizeof(struct sg_header)] );
		perror("");
		return status;
	}
	
	/* retrieve result */
	status = read(fd, rep_buf, sizeof(struct sg_header) + rep_len);
	if(status < 0 ||
	   status != sizeof(struct sg_header) + rep_len ||
	   sg_hd->result)
		return status;

	/* Look if we got what we expected to get */
	if(status == sizeof(struct sg_header) + rep_len)
		status = 0;
	
	return status;
}

/* Assemble a scsi command with a struct sg_header attached to the front and
 * send it off.  Expect the reply prepended with a struct sg_header.
 */
int send_cmd(tDeviceCmd cmd, tDeviceSetting val, unsigned char *reply)
{
	int ret;
	unsigned char cmdbuf[sizeof(struct sg_header) + 256];
	unsigned char *ptr = cmdbuf + sizeof(struct sg_header);
	struct inquiry_cmd *inq_cmd;
	struct custom_cmd *custom_cmd;
	int reply_size;

	switch(cmd) {

	case kDeviceCmdInquiry:
		inq_cmd = (struct inquiry_cmd *)(cmdbuf + sizeof(struct sg_header));
		memset(inq_cmd, 0, sizeof(struct inquiry_cmd));
		inq_cmd->opcode = cmd;
		inq_cmd->alloc_len = sizeof(struct inquiry_reply);
		ret = cmd_helper(sizeof(struct inquiry_cmd), cmdbuf,
						 sizeof(struct inquiry_reply), reply);
		break;

	case kDeviceCmdLockDevice:
	case kDeviceCmdUnlockDevice:
	case kDeviceCmdDisconnectOk:
	case kDeviceCmdUndefined:
		custom_cmd = (struct custom_cmd *)(cmdbuf + sizeof(struct sg_header));
		memset(custom_cmd, 0, sizeof(struct custom_cmd));
		custom_cmd->opcode = cmd;
		ret = cmd_helper(sizeof(struct custom_cmd), cmdbuf,
				 0, reply);
		break;

	case kDeviceCmdGetSetting:
		switch (val) {
		case kSettingBatteryLevel:
			reply_size = sizeof(struct battery_reply);
			break;

		case kSettingRTCCounter:
			reply_size = sizeof(struct rtc_reply);
			break;

		case kSettingSerialNumber:
			reply_size = 256;
			break;

		case kSettingUndefined:
			reply_size = 2;
			break;

		default:
			/* unrecognized setting */
			return 1;
		}

		custom_cmd = (struct custom_cmd *)(cmdbuf + sizeof(struct sg_header));
		memset(custom_cmd, 0, sizeof(struct custom_cmd));
		custom_cmd->opcode = cmd;
		custom_cmd->setting = val;
		ret = cmd_helper(sizeof(struct custom_cmd), cmdbuf,
				 reply_size, reply);
		break;

	default:
		/* Unrecognized command */
		return 1;
	}
	return ret;
}

int main(int argc, char **argv) {
	
	char *device_name = NULL;
	int c, ret;
	unsigned char reply[sizeof(struct sg_header) + MAX_REPLY_LEN];
	int cmd = kDeviceCmdInquiry, val = -1;

	while ((c = getopt (argc, argv, "hc:v:")) != -1)
		switch (c)
		{
		case 'h':
			print_help();
			return 0;

		case 'c':
			if(strcmp(optarg, "inquiry") == 0)
				cmd = kDeviceCmdInquiry;

			else if(strcmp(optarg, "lock") == 0)
				cmd = kDeviceCmdLockDevice;

			else if(strcmp(optarg, "unlock") == 0)
				cmd = kDeviceCmdUnlockDevice;

			else if(strcmp(optarg, "disconnect") == 0)
				cmd = kDeviceCmdDisconnectOk;

			else if(strcmp(optarg, "get") == 0)
				cmd = kDeviceCmdGetSetting;

			else if(strcmp(optarg, "undefined") == 0)
				cmd = kDeviceCmdUndefined;

			else {
				printf("Invalid command %s\n", optarg);
				return 1;
			}
			break;

		case 'v':
			if(strcmp(optarg, "battery") == 0)
				val = kSettingBatteryLevel;

			else if(strcmp(optarg, "rtc") == 0)
				val = kSettingRTCCounter;

			else if(strcmp(optarg, "serial") == 0)
				val = kSettingSerialNumber;

			else if(strcmp(optarg, "undefined") == 0)
				val = kSettingUndefined;

			else {
				printf("Invalid value %s\n", optarg);
				return 1;
			}
			break;

		case '?':
		default:
			print_help();
			return 1;
		}
	
	if(optind != argc - 1) {
		printf("No scsi device specified\n");
		return 1;
	}
	device_name = argv[optind];

	if(cmd == kDeviceCmdGetSetting && val == -1) {
		printf("Must get command requires -v option.\n");
		return 1;
	}
	
	fd = open(device_name, O_RDWR);
	if(fd == -1) {
		perror("Failed to open scsi device.");
		return 1;
	}
	
	/* print some fields of the inquiry result */
	ret = send_cmd(cmd, val, reply);
	if(ret) {
		printf("Failed to send cmd 0x%x\n", cmd);
		return 1;
	}
	print_reply(cmd, val, reply);
	
	close(fd);
	return 0;
}
