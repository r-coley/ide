/*
 * ide.h
 *
 * IDE main header for SVR4 
 *
 *  ata_ctrl_t
 * +-----------
 * |
 * | ata_unit_t
 * |+----------
 * || Master
 * |+----------
 * || Slave
 * |+----------
 * |
 * | ata_ioque_t
 * |+----------
 * ||ata_req_t 
 * |+----------
 * +-----------
 */

#ifndef _IDE_H
#define _IDE_H

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

#ifndef TRUE
#define TRUE 	1
#define FALSE 	0
#endif

#define EOK	0

#define IKDB()	si86_call_demon()

#define U2CTRLNO(X)		((X)->ctrl - &ata_ctrl[0])

/* Minor layout (USL-style, extended) ----------
 * 15 14 12 12  11 10 09 08  07 06 05 04  03 02 01 00
 *                    +-+-+  +  +-+-+ +   +----+----+
 *                      |    |    |   |        |
 *                      |    |    |   |        +------- Slice
 *                      |    |    |   +---------------- Drive 
 *                      |    |    +-------------------- Controller
 *                      |    +------------------------- ABSDEV (Whole Disk)
 *                      +------------------------------ Partition
 * Only Root can open ABSDEV() to give rw to entire disk irrepective
 * of slices but confined still to the partion map
 */
#define BASEDEV(dev)	(dev_t)(((dev) & ~0x30F)|0xf)
#define ABSDEV(dev)	(BASEDEV(dev)|0x80)
#define ISABSDEV(dev)	((dev)&0x80)

#define ATA_WHOLE_PART_SLICE   0x0F
#define ATA_CTRL(m)       (((getminor(m)) >> 5) & 0x03)
#define ATA_DRIVE(m)      (((getminor(m)) >> 4) & 0x01)
#define ATA_SLICE(m)	  ((getminor(m)) & ATA_WHOLE_PART_SLICE)
#define ATA_PART(m)       (((getminor(m)) >> 8) & 0x03)

/* Unit helpers: unit = ctrl*2 + drive */
#define ATA_UNIT_FROM(c,d)   	(((c) << 1) | ((d) & 1))
#define ATA_CTRL_FROM_UNIT(u) 	((u) >> 1)
#define ATA_DRIVE_FROM_UNIT(u) 	((u) & 1)
#define ATA_UNIT(m)          	ATA_UNIT_FROM(ATA_CTRL(m), ATA_DRIVE(m))
#define ATA_DEV(c,d)		(ATA_UNIT_FROM(c,d)<<4)
#define ATA_DEV_UNIT(d)		ATA_UNIT(getminor(d))
#define ATA_DEV_SLICE(d)	ATA_SLICE(getminor(d))

/* ---------- Per-unit state ---------- */

#define BUMP(C,field)	(C)->counters->field++

/* Well known fdisk partition types */
#define EMPTY		0x00
#define FAT12		0x01	
#define FAT16		0x04	
#define EXTDOS0		0x05
#define NTFS		0x07	
#define DOSDATA0	0x56
#define OTHEROS0	0x62
#define SVR4		0x63	/* UNIX V.x partition */
#define UNUSED2		0x64
#define LINUXSWAP	0x82
#define LINUXNATIVE	0x83
#define BSD386		0xA5
#define OPENBSD		0xA6
#define NETBSD		0xA9
#define SOLARIS		0xBF

/*** Devtype ***/
#define DEV_UNKNOWN	0x0000
#define DEV_ATA		0x0010
#define DEV_ATAPI	0x0020
#define DEV_PARALLEL	0x0000
#define DEV_SERIAL	0x0001

#define ATA_PDLOC	29	/* Sector location of PDINFO */

#define ATA_XFER_BUFSZ	(64*1024)	/* 64k */

#define XFERINC(R) \
	do { \
	(R)->xptr     += ATA_SECSIZE; \
	(R)->xfer_off += ATA_SECSIZE; \
	if ((R)->chunk_left >= 0) (R)->chunk_left   -= 1; \
	else			  (R)->chunk_left    = 0; \
	if ((R)->sectors_left >= 0) (R)->sectors_left -= 1; \
	else			    (R)->sectors_left  = 0; \
	} while (0)

#define DDI_INTR_UNCLAIMED	0
#define DDI_INTR_CLAIMED	1
#define	splbio	spl5

#define insw(port,addr,count)	linw(port,addr,count)
#define outsw(port,addr,count)	loutw(port,addr,count)

/* ---------- Controller limits ---------- */
#define ATA_MAX_CTRL   	4		
#define ATA_MAX_DRIVES	2
#define ATA_MAX_UNITS	(ATA_MAX_CTRL*ATA_MAX_DRIVES)

#define ATA_NPART 16

/* --- Unified controller flags --- */
#define ACF_NONE          0x0000  
#define ACF_PRESENT       0x0001  /* interrupts enabled path */
#define ACF_INTR_MODE     0x0002  /* interrupts enabled path */
#define ACF_IN_ISR        0x0004  /* currently in ISR context */
#define ACF_POLL_RUNNING  0x0008  /* poll engine active */
#define ACF_PENDING_KICK  0x0010  /* deferred kick requested */
#define ACF_BUSY  	  0x0020  /* busy */
#define ACF_CLOSING  	  0x0040  /* busy */
#define ACF_IRQ_ON	  0x0080  /* Interupts Enabled */
#define ACF_SYNC_DONE     0x0100  /* current sync request done */

#define AC_HAS_FLAG(ac,f)   (((ac)->flags & (f)) != 0)
#define AC_SET_FLAG(ac,f)   ((ac)->flags |= (f))
#define AC_CLR_FLAG(ac,f)   ((ac)->flags &= ~(f))

#define ATA_RF_NEEDCOPY	0x0001
#define ATA_RF_DONE	0x0002
#define ATA_RF_CDB_SENT	0x0004

/* --- Unified device flags --- */
#define UF_PRESENT		0x0001
#define UF_ATAPI		0x0002
#define UF_CDROM		0x0004
#define UF_MOZIP		0x0008
#define UF_HASMEDIA		0x0010
#define UF_REMOVABLE		0x0020
#define UF_ATAPI_NEEDS_SENSE	0x0080
#define UF_ABORT		0x8000

#define U_HAS_FLAG(u,f)	(((u)->flags & (f)) != 0)
#define U_SET_FLAG(u,f)	((u)->flags |= (f))
#define U_CLR_FLAG(u,f)	((u)->flags &= ~(f))

typedef enum {
	SEL_CHS = 0,
	SEL_LBA28
} sel_state_t;

/* Simple states for interrupt engine */
typedef enum {
	AS_IDLE = 0,
	AS_PRIMING,
	AS_PRIMED,
	AS_XFER,
	AS_WAIT,
	AS_DONE,
	AS_RESET,
	AS_ERROR
} ata_state_t;

enum intr_trigger {
	INTR_TRIGGER_INVALID 	= -1,
	INTR_TRIGGER_CONFORM 	= 0,
	INTR_TRIGGER_EDGE 	= 1,
	INTR_TRIGGER_LEVEL 	= 2
};

typedef enum {
	ATAPI_PHASE_IDLE	= 0,
	ATAPI_PHASE_WAIT_PKT_DRQ,
	ATAPI_PHASE_SEND_PACKET,
	ATAPI_PHASE_WAIT_DATA,
	ATAPI_PHASE_PIO_IN,
	ATAPI_PHASE_PIO_OUT,
	ATAPI_PHASE_DMA_XFER,
	ATAPI_PHASE_WAIT_COMPLETE,
	ATAPI_PHASE_ERROR
} atapi_phase_t;

typedef enum {
	ATAPI_DIR_NONE = 0,
	ATAPI_DIR_READ,
	ATAPI_DIR_WRITE
} atapi_dir_t;
	
struct ata_ctrl;
struct ata_ioque;
struct ata_unit;
struct ata_req;
struct ata_counters;
typedef struct ata_ctrl ata_ctrl_t;
typedef struct ata_ioque ata_ioque_t;
typedef struct ata_unit ata_unit_t;
typedef struct ata_req ata_req_t;
typedef struct ata_counters ata_counters_t;

typedef struct ata_part {
	u8_t	active;
	u32_t	base_lba;
	u32_t	nsectors;
	u8_t	systid;
	int	vtoc_valid;
	struct partition slice[ATA_NPART];
} ata_part_t;

/* ---------- I/O port base + helpers ---------- */
struct ata_ctrl {
	u16_t	io_base;   	/* base for data..status */
	u8_t	irq;       	/* wired IRQ (14/15 typical) */
	u32_t	flags;

	int	idx;
	int 	nreq;
	ata_ioque_t *ioque;
	ata_unit_t *drive[ATA_MAX_DRIVES];

	/*** watchdog/timeout ***/
	int	tmo_id;
	int	tmo_ticks;

	int	sel_drive;
	u8_t	sel_hi4;
	sel_state_t	sel_mode;
	
	u8_t	pio_multi;
	int	multi_set_ok; /* 1 if SET MULTIPLE accepted */

	/*** Counters ***/
	ata_counters_t *counters;
} ;

struct ata_ioque {
	/*** Submission Queue ***/
	ata_req_t *q_head;
	ata_req_t *q_tail;
	/*** Inflight request ***/
	ata_req_t *cur;
	ata_state_t state;
	/*** IRQ Policy/close gating ***/
	/*** sync/wakeup for raw/uio and close drain ***/
	
	int last_err;
	/*** staging buffer (lifetime: first open / last close ***/
	/*caddr_t	xptr;*/
	caddr_t xfer_buf;
	u32_t	xfer_bufsz;
	int	open_count;
} ;

/* Request representing a hardware chunk derived from a struct buf */
struct ata_req {
	struct ata_req *next;
	struct buf   *bp;        /* original request */

	dev_t	dev;		/* Original dev, udriv could be deduced */
	int	drive;		/* Drive: 0 master 1 slave */
	u32_t	reqid;

	int	flags;
	int 	is_write;  /* 1=write, 0=read */
	int	await_drq_ticks;

	/*** Chunking/progress ***/
	caddr_t	addr;     	/* current kernel addr within bp */
	u32_t	lba;       	/* device LBA(512B for ATA, 2048B for ATAPI) */
	u32_t	nsec;
	u32_t	lba_cur;
	u32_t	sectors_left;	/* How many sectors remain in entire request */
	u16_t	chunk_left;	/* How many sectors remain in current burst */
	u32_t 	chunk_bytes;	/* How many bytes remain in current burst */
	u32_t	xfer_off;
	u16_t	prev_chunk_left;
	u16_t	prev_sectors_left;
	int	wdog_stuck;
	caddr_t	xptr;

	u8_t	cmd;
	u16_t	atapi_bytes;
	u8_t	cdb[12];
	int	cdb_len;

	atapi_phase_t atapi_phase;
	atapi_dir_t atapi_dir;
	int	atapi_use_dma;
	u8_t	sense[18];

	u8_t	ast;
	u8_t	err;
};

struct ata_unit {
	u32_t	flags;

	int 	lba_ok;              	/* 1 if LBA28 supported */
	u32_t 	nsectors;  		/* total 512B sectors (ATA only) */
	int	pio_multi;

	ata_part_t fd[4];
	int	fdisk_valid;

	char 	model[41];
	int  	ioctl_warned;
	int  	read_only;          	/* 1 if media/device RW locked */

	char	vendor[9];
	char 	product[17];
	u16_t	devtype;
	u8_t	rmb,
		pdt;

	u32_t	atapi_blocks;
	u32_t	atapi_blksz;
	u16_t	lbsize;
	u8_t	lbshift;

	u8_t	cdb[16];
	u8_t	sense[18];
	caddr_t	atapi_bounce;

	int	drive;
};

struct ata_counters
{
	u32_t	polled_reads;
	u32_t	polled_write;
	u32_t	polled_chunks;
	u32_t	polled_sectors;
	u32_t	reads;
	u32_t	writes;
	u32_t	loops;
	u32_t	sectors;
	u32_t	wait_drq_ok;
	u32_t	wait_drq_to;
	u32_t	wait_bsy_to;
	u32_t	err;
	u32_t	irq_turnon;
	u32_t	irq_turnoff;
	u32_t	irq_seen;
	u32_t	irq_handled;
	u32_t	irq_spurious;
	u32_t	irq_no_cur;
	u32_t	irq_drq_service;
	u32_t	irq_eoc;
	u32_t	irq_err;
	u32_t	irq_bsy_skipped;
	u32_t	irq_atapi_ignored;
	u32_t	lost_irq_rescued;
	u32_t	wd_fired;
	u32_t	wd_serviced;
	u32_t	wd_rekicked;
	u32_t	wd_arm;
	u32_t	wd_cancel;
	u32_t	wd_resets;
	u32_t	wd_chunk;
	u32_t	eoc_polled;
	u32_t	softresets;
} ;

#include "ide_hw.h"
#include "ide_funcs.h"

#define CDB8(X) 	((u8_t)((X) & 0xff))
#define CDB16_H(X) 	((u8_t)(((X) >> 8) & 0xff))
#define CDB16_L(X) 	((u8_t)((X) & 0xff))
#define CDB32_B3(X) 	((u8_t)(((X) >> 24) & 0xff))
#define CDB32_B2(X) 	((u8_t)(((X) >> 16) & 0xff))
#define CDB32_B1(X) 	((u8_t)(((X) >>  8) & 0xff))
#define CDB32_B0(X) 	((u8_t)((X)         & 0xff))

/*
 * Private ATAPI CD-ROM ioctls for TOC/audio control.
 * These are driver-specific and can be mirrored in a user header.
 */
#define CDIOC_BASE	 ('C'<<8)
#define CDIOC_READTOC    (CDIOC_BASE | 0x01) /* read raw TOC into user buffer */
#define CDIOC_PLAYMSF    (CDIOC_BASE | 0x02) /* play audio from MSF range */
#define CDIOC_PAUSE      (CDIOC_BASE | 0x03) /* pause audio playback */
#define CDIOC_RESUME     (CDIOC_BASE | 0x04) /* resume audio playback */
#define CDIOC_EJECT      (CDIOC_BASE | 0x05) /* eject media (tray open) */
#define CDIOC_LOAD       (CDIOC_BASE | 0x06) /* load media (tray close) */
#define CDIOC_SUBCHANNEL (CDIOC_BASE | 0x07) /* load media (tray close) */

/*
 * Argument for CDIOC_READTOC
 *
 * toc_buf   - user-space buffer pointer
 * toc_len   - IN: max bytes user can accept (<= 4096)
 *             OUT: actual bytes returned
 * msf       - 0 = return addresses as LBA, 1 = MSF
 * format    - READ TOC format (0 for standard TOC)
 * track     - starting track/session number (usually 0 or 1)
 */
typedef struct cd_toc_io {
	u32_t	toc_buf;
	u16_t	toc_len;
	u8_t	msf;
	u8_t	format;
	u8_t	track;
	u8_t	reserved;
} cd_toc_io_t;

/*
 * Argument for CDIOC_PLAYMSF
 * Start and end positions in MSF (minutes/seconds/frames).
 */
typedef struct cd_msf_io {
	u8_t	start_m;
	u8_t	start_s;
	u8_t	start_f;
	u8_t	end_m;
	u8_t	end_s;
	u8_t	end_f;
} cd_msf_io_t;

/*
 * Argument for CDIOC_SUBCHANNEL
 *
 * audio_status	- low-level audio status from device
 * track	- current track number
 * index	- current index number (usually 1)
 * rel_*	- track-relative position (MSF)
 * abs_*	- absolute position from start of disc (MSF)
 */ 
typedef struct cd_subchnl_io {
	u8_t	audio_status;
	u8_t	track;
	u8_t	index;
	u8_t	_pad;
	u8_t	rel_m, rel_s, rel_f;
	u8_t	abs_m, abs_s, abs_f;
} cd_subchnl_io_t;

#endif /* _IDE_H */
