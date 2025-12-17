/*
 * ide_hw.h
 */

#ifndef _IDE_HW_H
#define _IDE_HW_H

#include <sys/types.h>
#include <sys/param.h>
#include <sys/buf.h>
#include <sys/kmem.h>
#include <sys/uio.h>
#include <sys/file.h>
#include <sys/cred.h>
#include <sys/conf.h>
#include <sys/ddi.h>
#include <sys/ipl.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/vtoc.h>
#include <sys/inline.h>
#include <sys/param.h>
#include <sys/fdisk.h>
#include <sys/mkdev.h>
#include <sys/varargs.h>
#include <sys/xdebug.h>
#include <sys/kdebugger.h>

/* Choose PIO Multiple policy: 1=max drive-supported, 0=cap at 8 */
#define ATA_USE_MAX_MULTIPLE 1
#define ATA_MAX_XFER_SECTORS 256 /* 128KiB per command */

#define ATA_MAX_RETRIES 3

#define ATA_SECSIZE	512
#define MAXNBLKS	256

/* Register offsets from io_base */
#define ATA_DATA         0x00
#define ATA_FEAT         0x01
#define ATA_ERROR        0x01
#define ATA_SECTCNT      0x02
#define ATA_SECTNUM      0x03
#define ATA_LBA0	 0x03
#define ATA_CYLLOW       0x04
#define ATA_LBA1	 0x04
#define ATA_CYLHIGH      0x05
#define ATA_LBA2	 0x05
#define ATA_DRIVEHD      0x06
#define ATA_COMMAND      0x07
#define ATA_STATUS       0x07

/* Control block (io_ctl) */
#define ATA_ALTSTATUS    0x00
#define ATA_DEVCTRL      0x00   /* write-only */

#define ATA_CMD_O(c)         (c->io_base + ATA_COMMAND)
#define ATA_FEAT_O(c)        (c->io_base + ATA_FEAT)
#define ATA_ERROR_O(c)       (c->io_base + ATA_ERROR)
#define ATA_STATUS_O(c)      (c->io_base + ATA_STATUS)
#define ATA_DATA_O(c)        (c->io_base + ATA_DATA)
#define ATA_SECTCNT_O(c)     (c->io_base + ATA_SECTCNT)
#define ATA_SECTNUM_O(c)     (c->io_base + ATA_SECTNUM)
#define ATA_CYLLOW_O(c)      (c->io_base + ATA_CYLLOW)
#define ATA_CYLHIGH_O(c)     (c->io_base + ATA_CYLHIGH)
#define ATA_DRVHD_O(c)       (c->io_base + ATA_DRIVEHD)
#define ATA_LBA0_O(c)        (c->io_base + ATA_LBA0)
#define ATA_LBA1_O(c)        (c->io_base + ATA_LBA1)
#define ATA_LBA2_O(c)        (c->io_base + ATA_LBA2)

#define ATA_ALTSTATUS_O(c)   (c->io_base+0x206 + ATA_ALTSTATUS)
#define ATA_DEVCTRL_O(c)     (c->io_base+0x206 + ATA_DEVCTRL)

/* Status bits */
#define ATA_SR_ERR   		0x01	/* Error */
#define ATA_SR_DRQ   		0x08	/* Data request RD/WR xfer sector */
#define ATA_SR_DSC   		0x10	/* Drive seek complete */
#define ATA_SR_DWF    		0x20	/* Drive Fault */
#define ATA_SR_DRDY  		0x40	/* Drive is ready for command */
#define ATA_SR_BSY   		0x80	/* Drive is busy */
#define ATA_ERR(ast)		((ast&(ATA_SR_ERR|ATA_SR_DWF)) != 0)

/* Devctl */
#define ATA_CTL_SRST 		0x04	/* Software Reset */
#define ATA_CTL_NIEN 		0x02	/* Disable INTRQ */
#define ATA_CTL_IRQEN 		0x00	/* Enable INTRQ */

/* Simple hd.c-style IRQ helpers */
#define ATA_IRQ_ON(ac)   do { \
		if (!AC_HAS_FLAG(ac,ACF_IRQ_ON)) { \
			outb(ATA_DEVCTRL_O(ac), ATA_CTL_IRQEN); \
			AC_SET_FLAG(ac,ACF_IRQ_ON); \
		} \
		BUMP(ac,irq_turnon); \
	} while (0)

#define ATA_IRQ_OFF(ac,force)  do { \
		if (AC_HAS_FLAG(ac,ACF_IRQ_ON) || force) { \
			outb(ATA_DEVCTRL_O(ac), ATA_CTL_NIEN); \
			AC_CLR_FLAG(ac,ACF_IRQ_ON); \
		} \
		BUMP(ac,irq_turnoff); \
	} while (0)

/* DRVHD bits */
#define ATA_DH_DRV		0x10
#define ATA_DHFIXED 		0xA0  /* 1,0,1,x, head3..0 */
#define ATA_DHLBA     		0x40

#define ATA_DH(d,lba,h4) \
		( (ATA_DHFIXED) | \
		  ((lba) ? ATA_DHLBA : 0) | \
		  (((d) & 1) << 4) | \
		  ((h4) & 0x0F) )
 
/* ATA/ATAPI commands */
#define ATA_CMD_IDENTIFY        0xEC
#define ATA_CMD_READ_SEC        0x20
#define ATA_CMD_READ_SEC_EXT    0x24
#define ATA_CMD_READ_MULTI	0xC4
#define ATA_CMD_READ_MULTI_EXT	0x29
#define ATA_CMD_WRITE_SEC       0x30
#define ATA_CMD_WRITE_SEC_EXT   0x34
#define ATA_CMD_WRITE_MULTI	0xC5
#define ATA_CMD_WRITE_MULTI_EXT	0x39
#define ATA_CMD_READ_SEC_RETRY  0x21
#define ATA_CMD_FLUSH_CACHE     0xE7
#define ATA_CMD_FLUSH_CACHE_EXT 0xEA
#define ATA_CMD_PACKET          0xA0
#define ATA_CMD_IDENTIFY_PKT    0xA1
#define ATA_CMD_SET_MULTI	0xC6

/* ATAPI CDB opcodes */
#define CDB_TEST_UNIT_READY     0x00
#define CDB_REQUEST_SENSE       0x03
#define CDB_INQUIRY             0x12
#define CDB_MODE_SENSE_6     	0x1A
#define CDB_MODE_SENSE_10       0x5A
#define CDB_START_STOP_UNIT     0x1B
#define CDB_PREVENT_ALLOW       0x1E
#define CDB_READ_10             0x28
#define CDB_WRITE_10            0x2A
#define CDB_READ_CAPACITY       0x25

#define CDB_READ_SUBCHANNEL     0x42
#define CDB_READ_TOC            0x43
#define CDB_PLAY_AUDIO_10       0x45
#define CDB_PLAY_AUDIO_MSF      0x47
#define CDB_PAUSE_RESUME        0x4B

#define ATAPI_MAX_RETRIES       3

#define ATAPI_VALID_CDB(len)	((len)==6||(len)==10||(len)==12||(len)==16)


/* ATAPI Interrupt Reason Bits (SECCNT in Packet mode) */
#define ATAPI_IR_COD	0x01	/* 1 = Command, 0 = Data */
#define ATAPI_IR_IO	0x02	/* 1 = Device->Host, 0 = Host->Device */

#endif /* _IDE_HW_H */
