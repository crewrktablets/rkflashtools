/* rkflashtool - for RockChip based devices.
 *               (RK2808, RK2818, RK2918, RK3066, RK3068 and RK3188)
 *
 * Copyright (C) 2010-2013 by Ivo van Poorten, Fukaumi Naoki, Guenter Knauf,
 *                            Ulrich Prinz, Steve Wilson
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Build with:
 *
 *      gcc -o rkflashtool rkflashtool.c -lusb-1.0 -O2 -W -Wall -s
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

/* hack to set binary mode for stdin / stdout on Windows */
#ifdef _WIN32
#include <fcntl.h>
int _CRT_fmode = _O_BINARY;
#endif

#define RKFLASHTOOL_VERSION_MAJOR      6
#define RKFLASHTOOL_VERSION_MINOR      1

/** USB Descriptor Definition
 *
 * Add all new Vendor-IDs or Product-IDs here that apply
 * to products supported by this software.
 */
#define USB_VID_RK         0x2207
#define USB_PID_RK2818     0x281a
#define USB_PID_RK2918     0x290a
#define USB_PID_RK3066     0x300a
#define USB_PID_RK3168     0x300b
#define USB_PID_RK3188     0x310b

#define RKFT_BLOCKSIZE      0x4000			/* must be multiple of 512 */
#define RKFT_IDB_BLOCKSIZE  0x210
#define RKFT_IDB_INCR       0x20
#define RKFT_MEM_INCR       0x80
#define RKFT_OFF_INCR       (RKFT_BLOCKSIZE>>9)

#ifndef RKFT_DISPLAY
#define RKFT_DISPLAY        0x100
#endif

/* TODO: Parameters flash area starts with PARM and is
 * available 8 times in a row at flash sector 0.
 */

typedef struct {
	char *name;			/**< Name of partition */
	int  start;			/**< Start address */
	int  size;			/**< Size of partition */
	void *next;			/** pointer to next part-entry */
} tPartition;

/* We keep it simple by declaring them plain instead
 * of making pointers and malloc things...
 */
static tPartition mtdparts[16];

/* empty buffers for FLASH should be filled with 0xFF
 * to avoid early wear-out.
 */
#define RKFT_FILLBYTE 	    0xff

typedef struct {
    uint16_t pid;
    char     name[8];
} t_pid;

const t_pid pidtab[] = {
    { USB_PID_RK2818, "RK2818" },
    { USB_PID_RK2918, "RK2918" },
    { USB_PID_RK3066, "RK3066" },
    { USB_PID_RK3168, "RK3168" },
    { USB_PID_RK3188, "RK3188" },
    { 0, "" },
};

typedef enum {
	RKFCMD_BLD_INIT		= 0x000600FD,		/* Found on RK2918 only */
	RKFCMD_BLD_HELLO	= 0x00060000,		/* Request hello from bootloader */
											/* USBS reply is 0x06 */
	RKFCMD_BLD_DATA		= 0x00061b00,		/* ?? Get Bootloader or Flash information */
											/* returns string B01321020311001V */

	RKFCMD_REBOOT		= 0x0006ff00,		/* Reboot */

	RKFCMD_FLASH_READ	= 0x000a1400,		/* Read from FLASH, USBS is 0x00 */
	RKFCMD_FLASH_WRITE	= 0x000a1500,		/* Write to FLASH, USBS is 0x00 */

	RKFCMD_DRAM_READ	= 0x000a1700,
	//RKFCMD_DRAM_WRITE

	RKFCMD_IDB_READ		= 0x000a0400,		/* Read IDB from FLASH */
	RKFCMD_IDB_WRITE	= 0x000a0500,		/* Write IDB to FLASH */
	RKFCMD_IDB_ERASE	= 0x000a0600,		/* ?? Erase IDB Sector */

} eSocCmd;

/** RK USB Handling
 *
 */

/* RK USB Endpoints
 *
 * Interface uses two endpoints
 */
#define EP_CMD			2		/* Endpoint for bootloader control */
#define EP_DATA 		1		/* Endpoint for bootloader data in / out */

/* Bootloader control message
 *
 */
#define RKFT_CID            4
#define RKFT_FLAG           12
#define RKFT_COMMAND        13
#define RKFT_OFFSET         17
#define RKFT_SIZE           23

/*
.----------------------------------------USBC: Command (out) or USBS Result (in)
|           .----------------------------CID:  Command ID, USBS retuns with same ID
|           |               .------------FLAG: Normally it just should toggle...
|           |               |          .-CMD:  Command for bootloader action
|           |               |          |       .--ADDRESS   SIZE--.
|           |               |          |       |                  |
.--. .---------.             .. .---------. .---------.       .---------.
0  3|4         8|           |12|13       16|17       20|     |23       26|
USBC CF 31 90 00 00 00 00 00 80 00 06 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
*/
typedef struct __attribute__((__packed__)) {
	uint32_t	token;		/**< Token is USBC 0x55534243 */
	uint32_t	cmdid;		/**< Command ID (sort of random number) */
	uint32_t	dummy1;		/**< Not used so far */
	uint8_t 	Flag;		/**< Sometimes set to 0x80, sometimes not */
	eSocCmd		cmd;		/**< The command */
	uint32_t	addr;		/**< address of data belonging to this command */
	uint16_t 	dummy2;		/**< Not used so far */
	uint32_t	size;		/**< size of data belonging to this command */
	uint32_t	dummy3;		/**< Not used so far */
} t_blcmd;

#define SETBE32(a, v) a  = ((v>> 0) & 0xff) << 24; \
                      a |= ((v>> 8) & 0xff) << 16; \
                      a |= ((v>>16) & 0xff) <<  8; \
                      a |= ((v>>24) & 0xff) <<  0

static uint8_t cmd[31] = { 'U', 'S', 'B', 'C', };
static uint8_t res[13];

static uint8_t buf[RKFT_BLOCKSIZE];
static uint32_t cid = 0xCF319000;
static int tmp;

static const char *const strings[3] = { "info", "fatal", "" };

static void info_and_fatal(const int s, char *f, ...) {
    va_list ap;
    va_start(ap,f);
    fprintf(stderr, "rkflashtool: %s: ", strings[s]);
    vfprintf(stderr, f, ap);
    va_end(ap);
    if (s) exit(s);
}

#define info(...)   info_and_fatal(0, __VA_ARGS__)
#define fatal(...)  info_and_fatal(1, __VA_ARGS__)
#define print(...)	info_and_fatal(2, __VA_ARGS__)

static void usage(void) {
	fprintf(stderr, "\n"
		"Usage is: rkflashtool [command] [options]\n"
    	  "FLASH Access Commands:\n"
    		"  r <offset> <size> > file   read flash to file\n"
    		"  w <offset> <size> < file   write flash from file\n"
    		"  e <offset> <size>          erase flash (fill with 0xff)\n"
            "  <offset> and <size> are in units of 512 bytes\n"
			"  Trailing bytes till sector end will be filled with 0xFF\n"
    	  "\nPartition Access Commands:\n"
    		"  R <partition> > file       read partition from flash\n"
			"  W <partition> < file       write to partition on flash\n"
			"  E <partition>              erase partition on flash\n"
			"  <partiton> must be defined in parameters section of flash.\n"
    	  "\nRAM Access\n"
			"  m <offset> <size> > file   read DRAM to file\n"
    	  "\nGeneral Device Support:\n"
    		"  i <offset> <blocks> > file read IDB flash\n"
            "  p [>file]                  fetch [or save] parameters\n"
            "  b                          reboot device\n"
			"  ?                          read SOC/NAND info\n"
          "\n");
}

static void send_cmd(libusb_device_handle *h, int e, uint8_t flag,
                     uint32_t command, uint32_t offset, uint8_t size)
{
	t_blcmd *usbcmd = malloc(sizeof(t_blcmd));
	if (usbcmd == NULL)
		fatal("Error no memory for cmd!");
	info("CMD is of size %d\n", sizeof(t_blcmd));
	memset( usbcmd, 0, sizeof(t_blcmd));
	usbcmd->token = 0x43425355;	/* USBC */
	SETBE32(usbcmd->cmdid, cid++);
	usbcmd->Flag = flag;
	SETBE32(usbcmd->cmd, command);
	SETBE32(usbcmd->addr, offset);
	SETBE32(usbcmd->size, size);
#if 0
	SETBE32(cmd[RKFT_CID ] = cid++;
    cmd[RKFT_FLAG] = flag;

    SETBE32(&cmd[RKFT_COMMAND], command);
    SETBE32(&cmd[RKFT_OFFSET ], offset );
    SETBE32(&cmd[RKFT_SIZE], size);
	//cmd[RKFT_SIZE] = size;
#endif
    libusb_bulk_transfer(h, e|LIBUSB_ENDPOINT_OUT, (uint8_t*)usbcmd, sizeof(t_blcmd), &tmp, 0);
    free(usbcmd);
}

#define send_buf(h,e,s) libusb_bulk_transfer(h, e|LIBUSB_ENDPOINT_OUT, \
                                             buf, s, &tmp, 0)

#define recv_res(h,e) libusb_bulk_transfer(h, e|LIBUSB_ENDPOINT_IN, \
                                           res, sizeof(res), &tmp, 0)

#define recv_buf(h,e,s) libusb_bulk_transfer(h, e|LIBUSB_ENDPOINT_IN, \
                                             buf, s, &tmp, 0)

#if PHASE_2
// TODO: How to put blocksizes? May be as fixed size>>9 ??
static int usb_tranceive( libusb_device_handle *h, eSocCmd command, uint32_t offset, void *buf, uint32_t size)
{
	int ret = -1;
	int flag = 0x00;
	uint8_t *rxbuf = NULL;
	uint8_t *txbuf = NULL;
	uint32_t rxsz = 0;
	uint32_t txsz = 0;

    switch (command) {
    /* Commands without buffer use */
    case RKFCMD_BLD_HELLO:
    case RKFCMD_REBOOT:
    	break;
    /* Commands receiving data from bootloader */
    case RKFCMD_BLD_DATA:
    case RKFCMD_FLASH_READ:

    case RKFCMD_DRAM_READ:
    case RKFCMD_IDB_READ:
    	if((buf == NULL) || (size == 0)) {
    		info( "%s(): Invalid buffer on READ command 0x%08x\n", __func__, command);
    		goto error_out;
    	}
    	rxbuf = (uint8_t*)buf;
    	rxsz = size;
    	break;

        /* Commands writing to bootloader */
    case RKFCMD_FLASH_WRITE:
    case RKFCMD_IDB_WRITE:
    	flag = 0x80;
    //case RKFCMD_DRAM_WRITE:
    	if((buf == NULL) || (size == 0)) {
    		info( "%s(): Invalid buffer on WRITE command 0x%08x\n", __func__, command);
    		goto error_out;
    	}
    	txbuf = (uint8_t*)buf;
    	txsz = size;
    	break;
    case RKFCMD_IDB_ERASE:
    	break;
    }

	/* setup command */
	cmd[RKFT_CID ] = cid++;
    cmd[RKFT_FLAG] = flag;

    SETBE32(&cmd[RKFT_COMMAND], command);
    SETBE32(&cmd[RKFT_OFFSET ], offset );
    SETBE32(&cmd[RKFT_SIZE], size);

    /* Send Command out */
    ret = libusb_bulk_transfer(h, EP_CMD|LIBUSB_ENDPOINT_OUT, cmd, sizeof(cmd), &tmp, 0);
    if (ret) {
    	info( "%s():%d Error Code %d received\n", __func__, __LINE__, ret);
    }

    /* Optional send or receive some data */
    if (txsz && txbuf) {
    	/* We have something to send to the bootloader */
    	ret = libusb_bulk_transfer( h, EP_DATA|LIBUSB_ENDPOINT_OUT , txbuf, txsz, &tmp, 0);
        if (ret) {
        	info( "%s():%d Error Code %d received\n", __func__, __LINE__, ret);
        }
    }
    else if (rxsz && rxbuf) {
    	/* We have something to receive from the bootloader */
    	ret = libusb_bulk_transfer( h, EP_DATA|LIBUSB_ENDPOINT_IN , rxbuf, rxsz, &tmp, 0);
        if (ret) {
        	info( "%s():%d Error Code %d received\n", __func__, __LINE__, ret);
        }
    }

    /* Request wait and receive status response from bootloader */
    ret = libusb_bulk_transfer( h, EP_DATA|LIBUSB_ENDPOINT_IN , rxbuf, rxsz, &tmp, 0);
    if (ret) {
    	info( "%s():%d Error Code %d received\n", __func__, __LINE__, ret);
    }

error_out:
    return ret;
}
#endif

/*************************************************************************
 *
 *  FLASH Works
 */

/** Read FLASH memory from SOC to file
 *
 * This function reads from FLASH memory of the SOC at a given address and
 * outputs the content to a file.
 */
int soc_flash_read( libusb_device_handle *h, int offset, int size)
{
	int ret = 0;
    while (size > 0)
    {
        info("reading flash memory at offset 0x%08x", offset);
#if PHASE_2
        usb_tranceive( h, RKFCMD_FLASH_READ, offset, RKFT_BLOCKSIZE)
#else
        send_cmd(h, 2, 0x00, RKFCMD_FLASH_READ, offset, RKFT_OFF_INCR);
        recv_buf(h, 1, RKFT_BLOCKSIZE);
        recv_res(h, 1);
#endif

        /* Bootloader result is returned in tmp buffer */
        info ( " = 0x%x\n", tmp);

        ret = write(1, buf, RKFT_BLOCKSIZE);
        if ( ret < 0) {
        	/* Info on error, but continue to close USB stack */
            info("Write error! Disk full?\n");
            break;
        }

        offset += RKFT_OFF_INCR;
        size   -= RKFT_OFF_INCR;
    }
//	fprintf(stderr, "\n");
    return ret;
}

/** Write file to FLASH memory of SOC
 *
 * This function reads a file and writes it via the bootloader
 * of the SOC to its FLASH memory.
 */
int soc_flash_write( libusb_device_handle *h, int offset, int size)
{
	int ret = 0;
    while ((size > 0) && ( ret > 0))
    {
        info("writing flash memory at offset 0x%08x", offset);

        /* fill unused bytes with 0xFF before flashing */
        memset(buf, RKFT_FILLBYTE, RKFT_BLOCKSIZE);
        ret = read(0, buf, RKFT_BLOCKSIZE);
        if ( ret < 0) {
        	/* Just stop further reading, but do write last sector */
            info("EOF reached.\n");
        }

        send_cmd(h, 2, 0x80, RKFCMD_FLASH_WRITE, offset, RKFT_OFF_INCR);
        send_buf(h, 2, RKFT_BLOCKSIZE);
        recv_res(h, 1);
        /* Bootloader result is returned in tmp buffer */
        info ( " = 0x%x\n", tmp);

        offset += RKFT_OFF_INCR;
        size   -= RKFT_OFF_INCR;
    }
//	fprintf(stderr, "\n");
    return ret;
}

/** Erase FLASH memory of SOC
 *
 * This function erases the FLASH via the bootloader.
 */
int soc_flash_erase( libusb_device_handle *h, int offset, int size)
{
	int ret = 0;
	memset(buf, RKFT_FILLBYTE, RKFT_BLOCKSIZE);
	while (size > 0) {
		if (offset % RKFT_DISPLAY == 0)
			info("erasing flash memory at offset 0x%08x", offset);

		send_cmd(h, 2, 0x80, RKFCMD_FLASH_WRITE, offset, RKFT_OFF_INCR);
		send_buf(h, 2, RKFT_BLOCKSIZE);
		recv_res(h, 1);
        /* Bootloader result is returned in tmp buffer */
        info ( " = 0x%x\r", tmp);

		offset += RKFT_OFF_INCR;
		size   -= RKFT_OFF_INCR;
	}
	fprintf(stderr, "\n");
	return ret;
}

/*************************************************************************
 *
 *  PARAMETER Works
 */

/** Read PARAMETERS from FLASH of SOC
 *
 * This function reads the Android PARAMETERS file content.
 * The function does not write it to disc, that is handled separately.
 * The content is saved in a buffer an can be used by other commands
 * to auto-evaluate addresses.
 *
 */
int soc_parameters_get( libusb_device_handle *h, uint8_t *buf, int *size)
{
	uint32_t *p = (uint32_t*)buf;

	*size = 0;
	info("reading parameters from SOC");

	send_cmd(h, 2, 0x00, RKFCMD_FLASH_READ, 0, RKFT_OFF_INCR);
	recv_buf(h, 1, RKFT_BLOCKSIZE);
	recv_res(h, 1);
    /* Bootloader result is returned in tmp buffer */
    info ( " = 0x%x\n", tmp);

    /* first field is checksum, second is size of parameters */
	info("rkcrc: 0x%08x\n", *p++);
	*size = *p;
	info("size:  0x%08x\n", *size);
	buf[*size+8] = 0;

	return 0;
}

int soc_add_partition( char *name, int start, int size)
{
	tPartition *cp = &mtdparts[0];
	int i = 16;

	/* find first empty entry */
	while (cp->name && i--) cp++;

	/* add new partition entry */
	cp->name = name;
	cp->start = start;
	cp->size = size;

	return 0;
}

int soc_find_partition( char *name, int *start, int *size)
{
	int ret = -1;
	int i = 16;
	tPartition *cp = &mtdparts[0];

	/* find first empty entry */
	while ((cp->name) && strncmp( cp->name, name, 20) && i--) cp++;
	if (i && cp->name) {
		info ("found [%s] at 0x%08x of size 0x%08x\n", cp->name, cp->start, cp->size);
		*start = cp->start;
		*size = cp->size;
		ret = 0;
	}
	return ret;
}
/*
 *
 * mtdparts=rk29xxnand:0x00002000@0x00002000(misc),0x00006000@0x00004000(kernel),0x00008000@0x0000A000(boot),0x00010000@0x00012000(recovery),0x00020000@0x00022000(backup),0x00040000@0x00042000(cache),0x00400000@0x00082000(userdata),0x00002000@0x00482000(kpanic),0x00100000@0x00484000(system),-@0x00584000(user)
 */
int soc_parameters_decode( uint8_t *buf, int size)
{
	char *cmtd = (char*)buf+8; /* skip crc, size */
	char *cmte = cmtd+size;
	char *csize, *cstart, *cname;
	int pstart, psize;

	info("decoding parameters...\n");

	/* Find mtdparts */
	cmtd = strstr( cmtd, "mtdparts=");
	if (cmtd == NULL)
	{
		info("Cannot find MTD in parameters\n");
		return -1;
	}

	/* check for rockchip type mtd partitions */
	cmtd = strstr( cmtd, "rk29xxnand:");
	if (cmtd == NULL)
	{
		info("Cannot find MTD in parameters\n");
		return -1;
	}

	/* Proceed past mtdname "rk29xxnand:" */
	cmtd += strlen( "rk29xxnand:");

	/* get first partiton entry and then bash string into pieces */
	csize = strtok( cmtd, ",");
	while (csize && (csize < cmte)) {
		// info("found partition [%s]\n", csize);
		cstart = strchr(csize, '@');
		*cstart++ = '\0';

		cname = strchr( cstart, '(');
		*cname++ = '\0';
		*strchr( cname, ')') = '\0';

		info ("  >> found [%s] at %s of size %s\n", cname, cstart, csize);

		psize = strtol( csize, NULL, 0);
		pstart = strtol( cstart, NULL, 0);

		if (psize == 0)
			psize = -1;

		if (pstart == 0) {
			info ("Wrong MTD partition address\n");
			return -1;
		}

		info ("  ## found [%s] at 0x%08x of size 0x%08x\n", cname, pstart, psize);

		soc_add_partition( cname, pstart, psize);

		csize = strtok( NULL, ",");
	}

	return 0;
}

int soc_autopart( libusb_device_handle *h, char *name, int *offset, int *size)
{
	int ret = soc_parameters_get( h, buf, size);
	if (ret) {
		info("Error reading parameters from SOC\n");
		return ret;
	}
	ret = soc_parameters_decode( buf, *size);
	if (ret) {
		info("Error decoding parameters from SOC\n");
		return ret;
	}
	ret = soc_find_partition( name, offset, size);
	if (ret) {
		info("Error partition %s unknown\n", name);
		return ret;
	}

	info("Accessing %s 0x%08x 0x%08x\n", name, offset, size);
	return ret;
}

/** Save PARAMETERS to Output
 *
 * Write parameters to given default output.
 */
int soc_parameters_save( uint8_t *buf, int size)
{
	int ret = write(1, (char*)buf, size);
	if (  ret <= 0) {
		info("Write error! Disk full?\n");
	}

	return ret;
}

/*************************************************************************
 *
 *  RAM Works
 */

int soc_dram_read( libusb_device_handle *h, int offset, int size)
{
	int ret = 0;
    while ((size > 0) && (ret > 0))
    {
        int sizeRead = size > RKFT_MEM_INCR ? RKFT_MEM_INCR : size;
        info("reading memory at offset 0x%08x size %x", offset, sizeRead);

        send_cmd(h, 2, 0x00, 0x000a1700, offset-0x60000000, sizeRead);
        recv_buf(h, 1, sizeRead);
        recv_res(h, 1);
        /* Bootloader result is returned in tmp buffer */
        info ( " = 0x%x\n", tmp);

        ret = write(1, buf, sizeRead);
        if ( ret <= 0) {
            fatal("Write error! Disk full?\n");
            size = 0;
        }
        offset += sizeRead;
        size -= sizeRead;
    }
    return ret;
}

// TODO: Add functions to read / write SRAM

/*************************************************************************
 *
 *  Support Functions
 */

int soc_bootloader( libusb_device_handle *h)
{
	int ret = 0;
	uint8_t ver[40];
	int i, n = 0;

	info("Detecting Hardware...\n");

	cid = 0xCF319000;
	send_cmd(h, EP_CMD, 0x80, RKFCMD_BLD_INIT, 0x00000000, 0x00);
	if (ret)
		return ret;
	ret = recv_res(h, EP_DATA);
	if (ret)
		return ret;
	fprintf(stderr, "  0x%08x: Init returned 0x%02x\n", RKFCMD_BLD_INIT, res[sizeof(res)-1]);

	if (res[sizeof(res)-1] == 0) {
		/* Try secondary info request for newer devices */
		cid = 0xCF319000;
		send_cmd(h, EP_CMD, 0x80, RKFCMD_BLD_DATA, 0x00000000, 0x00);
		recv_buf(h, EP_DATA, 16);
		recv_res(h, EP_DATA);
		info("Init returnd 0x%02x\n", res[sizeof(res)-1]);
		if (buf[0] == 'B') {
			buf[16] = 0;
			fprintf(stderr, "0x%08x: Got Tokens: %s\n", RKFCMD_BLD_DATA, (char*)buf);
		}
	}
	else {
		memset( ver, 0, sizeof(ver));
		i = 0;
		do {
			send_cmd(h, EP_CMD, 0x80, RKFCMD_BLD_HELLO, 0x00000000, 0x00);
			recv_res(h, EP_DATA);
			if (res[sizeof(res)-1] == 1) {
				n=res[sizeof(res)-3];
				ver[n]=res[sizeof(res)-2];
			}
			i++;
		} while ((res[sizeof(res)-1]==1) && (i<16));

		/* Print this as debug until we can decode it. */
		if (n) {
			int i;
			fprintf(stderr, "0x%08x: Got Tokens: ", RKFCMD_BLD_HELLO);
			for (i=0; i<=n; i++)
				fprintf(stderr, "%02x", ver[i]);
			fprintf(stderr, "\n");
		}
	}
    return ret;
}

int soc_reboot( libusb_device_handle *h)
{
	int ret = 0;

	info("rebooting device...\n");
	send_cmd(h, 2, 0x00, RKFCMD_REBOOT, 0x00000000, 0x00);
	recv_res(h, 1);

	//TODO: evaluate tmp[0] response
    return ret;
}

/*************************************************************************
 *
 *  MAIN
 */

#define NEXT do { argc--;argv++; }while(0)

int main(int argc, char **argv) {
    const t_pid *ppid = &pidtab[0];
    libusb_context *c;
    libusb_device_handle *h = NULL;
    int offset = 0, size = 0;
    int ret = 0;
    char action;
    char *pname = NULL;

    fprintf(stderr, "rkflashtool v%d.%d\n", RKFLASHTOOL_VERSION_MAJOR,
                                 RKFLASHTOOL_VERSION_MINOR);
    
    NEXT; if (!argc) usage();

    action = **argv; NEXT;

    switch(action) {
    case 'b':
        if (argc) usage(); 
        break;
    case 'e':
    case 'r': 
    case 'w': 
    case 'm':
    case 'i':
        if (argc != 2) usage();
        offset = strtoul(argv[0], NULL, 0);
        size   = strtoul(argv[1], NULL, 0);
        break;
    case 'p':
    case 'd':
        if (argc) usage();
        offset = 0;
        size = 1024;
        break; 
    case 'E':
    case 'R':
    case 'W':
    	if (argc != 1) usage();
    	pname = argv[0];
    	break;
    default:
        usage();
    }

    /* Initialize mtsparts */
    memset( mtdparts, 0, sizeof( mtdparts));

    /* Initialize libusb */
    
    if (libusb_init(&c)) fatal("cannot init libusb\n");

    libusb_set_debug(c, 3);
    
    /* Detect connected RockChip device */
    
    while ( !h && ppid->pid) {
        h = libusb_open_device_with_vid_pid(c, USB_VID_RK, ppid->pid);
        if (h) {
            info("Detected %s...\n", ppid->name);
            break;
        }
        ppid++;
    } 
    if (!h) fatal("cannot open device\n");

    /* Connect to device */
    
    if (libusb_kernel_driver_active(h, 0) == 1) {
        info("kernel driver active\n");
        if (!libusb_detach_kernel_driver(h, 0))
            info("driver detached\n");
    }

    if (libusb_claim_interface(h, 0) < 0)
        fatal("cannot claim interface\n");
        
    info("interface claimed\n");

    /* Initialize bootloader interface */
    ret = soc_bootloader( h);

    /* Check and execute command */
    switch(action)
    {
    case 'b':   /* Reboot device */
    	ret = soc_reboot( h);
        break;

    case 'R':	/* Read FLASH by Partiton */
    	if (soc_autopart( h, pname, &offset, &size))
    		break;
    case 'r':   /* Read FLASH by Address */
    	ret = soc_flash_read( h, offset, size);
        break;

    case 'W':	/* Write FLASH by Partiton */
    	if (soc_autopart( h, pname, &offset, &size))
    		break;
    case 'w':   /* Write FLASH */
    	ret = soc_flash_write( h, offset, size);
        break;

    case 'E':	/* Erase FLASH by Partiton */
    	if (soc_autopart( h, pname, &offset, &size))
    		break;
    case 'e':   /* Erase flash */
    	ret = soc_flash_erase( h, offset, size);
		break;

    case 'p':   /* Write parameters to file */
    	ret = soc_parameters_get( h, buf, &size);
    	ret = soc_parameters_decode( buf, size);
    	ret = soc_parameters_save( buf, size);
    	/* write size to parameters */
        break;

    case 'd':
    	ret = soc_parameters_get( h, buf, &size);
    	ret = soc_parameters_decode( buf, size);
    	break;

    case 'm':   /* Read RAM */
    	ret = soc_dram_read( h, offset, size);
        break;

    case 'i':   /* Read IDB */
        while (size > 0) 
        {
            int sizeRead = size > RKFT_IDB_INCR ? RKFT_IDB_INCR : size;
            info("reading IDB flash memory at offset 0x%08x\n", offset);
             
            send_cmd(h, 2, 0x80, 0x000a0400, offset, sizeRead);
            recv_buf(h, 1, RKFT_IDB_BLOCKSIZE * sizeRead);
            recv_res(h, 1);
             
            if ( write(1, buf, RKFT_IDB_BLOCKSIZE * sizeRead) <= 0) {
                fatal("Write error! Disk full?\n");
            }
            offset += sizeRead;
            size -= sizeRead;
        }
        break;

    case 'f':	/* Flash Chip Erase */
    	// TODO: Implement Flash Chuip Eraser and Bootloader update
    	break;

    default:
        break;
    }

    /* Disconnect and close all interfaces */

    libusb_release_interface(h, 0);
    libusb_close(h);
    libusb_exit(c);
    return 0;
}
