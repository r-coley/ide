#include "ide.h"
#include <stdarg.h>
#include <sys/cmn_err.h>

extern void ata_service_irq(ata_ctrl_t *ac, ata_req_t *r, u8_t st);

#define BS	0x08

extern char 	putbuf[];
extern int	putbufsz;
extern int	putbufndx;
extern int	ata_debug_console;
extern short	prt_where;

extern int  dbg_getchar();
extern void dbg_putchar(int);

u32_t	req_seq=0;

void
ATADEBUG(int lvl, char *fmt, ...) 
{
	va_list ap;
	char buf[256];
	int i;

	if (atadebug < lvl || fmt == NULL)
		return;

	va_start(ap, fmt);
	(void)vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	/* Ensure newline termination for /dev/osm consumption */
	for (i = 0; i < (int)sizeof(buf) && buf[i] != '\0'; i++)
		;
	if (i == 0 || buf[i - 1] != '\n') {
		if (i < (int)sizeof(buf) - 1) {
			buf[i++] = '\n';
			buf[i] = '\0';
		} else {
			buf[sizeof(buf) - 2] = '\n';
			buf[sizeof(buf) - 1] = '\0';
		}
	}

	/*
	 * Write to putbuf directly so debug is available via /dev/osm
	 * without relying on prt_where routing (avoids interleaving issues).
	 * One wakeup per message.
	 */
	{
		int sp = splhi();
		for (i = 0; buf[i] != '\0'; i++)
			putbuf[putbufndx++ % putbufsz] = buf[i];
		splx(sp);
	}

	wakeup((caddr_t)putbuf);

	/* Optional console mirroring */
	if (ata_debug_console)
		printf("%s", buf);
}

void
CopyTbl(ata_part_t *fp,struct ipart *ipart)
{
	fp->active   = ipart->bootid;
	fp->base_lba = (u32_t)ipart->relsect;
	fp->nsectors = (u32_t)ipart->numsect;
	fp->systid   = (int)ipart->systid;
}

char *
getstr(char *ptr, int len, int swap, int blanks,int stop)
{
static	char	buf[50];
	int	i;
	
	for(i=0;i<len;i++) {
		buf[i]=ptr[i];
		if (swap && i%2) {
			buf[i]   = ptr[i-1];
			buf[i-1] = ptr[i];
		}
	}
	if (blanks) {
		i=len;
		while (i > 0 && buf[i-1] == ' ') i--;
		buf[i] = 0;
	}
	if (stop) {
		for(i=0;i<len && buf[i] && buf[i] != ' ';i++);
		buf[i] = 0;
	}
	return buf;
} 

char *
Cstr(ata_ctrl_t *ac) 
{
static	char	buf[20];
	int	d=ac->sel_drive;
	int	c=ac->idx;
	ata_unit_t *u = ac->drive[d];
	dev_t	dev=ATA_DEV(c,d);
	char 	*suffix=ISABSDEV(dev) ? " <ABSDEV>" : "";

	sprintf(buf,"c%dd%d%s",c,d,suffix);
	return buf;
}

char *
Dstr(dev_t dev)
{
static	char	buf[50];
	char 	*suffix=ISABSDEV(dev) ? " <ABSDEV>" : "";
	int 	ctrl = ATA_CTRL(dev), 
		unit = ATA_UNIT(dev), 
		driv = ATA_DRIVE(dev), 
		fdisk = ATA_PART(dev),
		slice = ATA_SLICE(dev);
	sprintf(buf,"c%dd%dp%ds%d%s",ctrl,driv,fdisk,slice,suffix);
	return buf;
}

char *
Istr(int cmd)
{
	switch (cmd) {
	case V_CONFIG:	 return "V_CONFIG";
	case V_REMOUNT:  return "V_REMOUNT";
	case V_GETPARMS: return "V_GETPARMS";
	case V_FORMAT: 	 return "V_FORMAT";
	case V_PDLOC: 	 return "V_PDLOC";
	case V_RDABS: 	 return "V_RDABS";
	case V_WRABS:	 return "V_WRABS";
	case V_VERIFY:	 return "V_VERIFY";
	case CDIOC_READTOC: return "CDIOC_READTOC";
	case CDIOC_PLAYMSF: return "CDIOC_PLAYMSF";
	default:	 return "V_default";
	}
}

void
reset_queue(ata_ctrl_t *ac,int hard)
{
	ata_ioque_t *q=ac->ioque;

	ATADEBUG(2,"reset_queue()\n");
	if (ac->tmo_id) {
    		untimeout(ac->tmo_id);
    		ac->tmo_id = 0;
	}
	/* Soft reset of the channel engine; do NOT free xfer_buf here. */
	q->cur       = 0;
	q->state     = AS_IDLE;
	AC_CLR_FLAG(ac, ACF_BUSY);
}

void
ata_attach(int ctrl)
{
	ata_ctrl_t *ac= &ata_ctrl[ctrl];
	int 	i, drive, rc=-1;

	ATADEBUG(1,"ata_attach(%d)\n",ctrl);

	if (!AC_HAS_FLAG(ac,ACF_PRESENT)) return;

	if (ata_intr_mode || atapi_intr_mode) {
		RegisterIRQ(ac->irq,&ataintr, SPL5, INTR_TRIGGER_EDGE);
		AC_SET_FLAG(ac,ACF_INTR_MODE);
	}
	if (ata_intr_mode) printf("ATA in intr mode\n");
	if (atapi_intr_mode) printf("ATAPI in intr mode\n");
	ata_softreset_ctrl(ac);
	for (drive = 0; drive <= 1; drive++) {
		ata_unit_t *u = ac->drive[drive];

		bzero((caddr_t)u,sizeof(*u));
		u->pio_multi=1;		/* 1 sector xfers */
		u->atapi_blocks = 0;
		u->atapi_blksz  = 0;
		ata_probe_unit(ac,drive);
	}
}

int 
ata_read_vtoc(dev_t dev,int part)
{
	int 	slice = ATA_SLICE(dev),
		ctrl  = ATA_CTRL(dev),
		drive = ATA_DRIVE(dev);
	ata_ctrl_t *ac = &ata_ctrl[ctrl];
	ata_unit_t *u=ac->drive[drive];
	u32_t 	base, lba, off;
	caddr_t k = 0;
	struct pdinfo *pd;
	struct vtoc *v;
	ata_part_t *fp = &u->fd[part];
	int 	s;
	int rootu = ATA_DEV_UNIT(dev);
	int swapu = ATA_DEV_UNIT(swapdev);

	ATADEBUG(1,"ata_read_vtoc(%s dev=%x) rootu=%x swapu=%x\n",
		Dstr(dev),BASEDEV(dev),rootu,swapu);

	if (U_HAS_FLAG(u,UF_ATAPI)) return 0;

	/* Only attempt on UNIX partitions with a size. */
	if (fp->systid != UNIXOS || fp->nsectors == 0) return 0;

	base = fp->base_lba;

	/* pdinfo + vtoc live in the same sector at (unix_base + VTOC_SEC). */
	lba = base + (u32_t)VTOC_SEC;
	if (!(k = kmem_alloc(DEV_BSIZE, KM_SLEEP))) return 0;

        if (ata_getblock(ABSDEV(dev),lba,(caddr_t)k,DEV_BSIZE) != 0) {
		kmem_free(k, DEV_BSIZE);
		return EIO;
	}

	pd = (struct pdinfo *)k;
	if (pd->sanity != VALID_PD || pd->version != 1) {
		kmem_free(k, DEV_BSIZE);
		return 0;
	}

	/*
	 * vtoc is embedded within this same 512B sector at offset 
	 * dp.vtoc_ptr*dp_secsiz 
	 */
        off = (u32_t)pd->vtoc_ptr % DEV_BSIZE;
        if (off + sizeof(struct vtoc) > DEV_BSIZE) {
            kmem_free(k, DEV_BSIZE);
            return 0;
        }
        v = (struct vtoc *)((char*)k + off);

	if (v->v_sanity != VTOC_SANE || v->v_version != V_VERSION) {
		kmem_free(k, DEV_BSIZE);
		return 0;
	}

	/* Copy slices from vtoc into our driver table. */
	for (s = 0; s < V_NUMPAR; ++s) {
		fp->slice[s].p_tag = v->v_part[s].p_tag;
		fp->slice[s].p_flag = v->v_part[s].p_flag;
		fp->slice[s].p_start = v->v_part[s].p_start;
		fp->slice[s].p_size  = v->v_part[s].p_size;
		if (fp->slice[s].p_tag == V_SWAP &&
		    fp->slice[s].p_flag & V_VALID) {
				if (swapdev != NODEV && rootu == swapu) {
					nswap = fp->slice[s].p_size;
				}
			}
	}

	/* Whole-fdisk pseudo-slice: full partition range. */
	fp->slice[ATA_WHOLE_PART_SLICE].p_start = 0;
	fp->slice[ATA_WHOLE_PART_SLICE].p_size  = fp->nsectors;

	kmem_free(k, DEV_BSIZE);
	return 1;
}

void
ata_copy_model(u16_t *id, char *dst)
{
	int 	i;
	char 	*p = dst;
	for (i = 27; i < 27 + 20; i++) {
		u16_t w = id[i];
		*p++ = (char)(w >> 8);
		*p++ = (char)(w & 0xFF);
	}
	*p = 0;
	for (i = (int)strlen(dst)-1; i >= 0; --i) {
		if (dst[i] == ' ') dst[i] = 0; 
		else break;
	}
}

int
ata_read_signature(ata_ctrl_t *ac, u8_t drive,u16_t *type)
{
	u8_t st, lc=0, hc=0;
	u16_t	dev;

	ATADEBUG(1,"read_signature(drive=%d)\n",drive);

	if (ata_sel(ac,drive,0) != 0) return EIO;

	/*** Send an CMD_IDENTIFY to force the signature info on the bus ***/
	outb(ATA_CMD_O(ac), ATA_CMD_IDENTIFY);

	/* Wait for not-BSY */
	if (ata_wait(ac, 0, ATA_SR_BSY, 500000L, 0, 0) != 0) return EIO;

	lc = inb(ATA_CYLLOW_O(ac));
	hc = inb(ATA_CYLHIGH_O(ac));
	ATADEBUG(1,"read_signature() lc=%02x hc=%02x\n",lc,hc);

	switch ((hc<<8)|lc)
	{
	case 0x0000: dev = DEV_ATA|DEV_PARALLEL; 	break;
	case 0x0800: dev = DEV_ATA|DEV_PARALLEL; 	break;
	case 0xC33C: dev = DEV_ATA|DEV_SERIAL; 		break;
	case 0xEB14: dev = DEV_ATAPI|DEV_PARALLEL; 	break;
	case 0x9669: dev = DEV_ATAPI|DEV_SERIAL; 	break;
	default:     dev = DEV_UNKNOWN;			break;
	}
	*type = dev;
	return (dev == DEV_UNKNOWN) ? -1 : 0;
}

int
ata_probe_unit(ata_ctrl_t *ac, u8_t drive)
{
	ata_unit_t *u = ac->drive[drive];
	u8_t 	lc, hc, st;
	u32_t	type;
	char 	*klass;

	ATADEBUG(3,"ata_probe_unit(%d)\n",drive);
	if (!AC_HAS_FLAG(ac,ACF_PRESENT)) return ENXIO;

	if (ata_read_signature(ac,drive,&u->devtype) != 0) return ENXIO;
	if (u->devtype & DEV_ATAPI) U_SET_FLAG(u,UF_ATAPI);

	if (ata_identify(ac, drive) != 0) return ENXIO;
 
	ac->tmo_id    = 0;
	ac->tmo_ticks = drv_usectohz(2000000); /* 2s is sane for PIO */

	U_SET_FLAG(u,UF_PRESENT);
	if (U_HAS_FLAG(u,UF_ATAPI)) {
		u32_t 	blocks=0, blksz=0;
		char	*med="";

		/*
		 * Populate inquiry fields into ata_unit[] 
		 * Set CDROM, MOZIP and model etc
		 */
		(void)atapi_inquiry(ac, drive);

		if ((atapi_read_capacity(ac,drive,&blocks,&blksz) == 0) &&
			blocks && blksz) {
			/*printf("blocks=%ld, blksz=%ld\n",
				blocks,blksz);*/
			;
		}
		if (blocks==0 || blksz == 0) {
			u->atapi_blocks = 0;
			u->atapi_blksz  = 0;
		}

		u->lbsize = (blksz ? blksz : 2048);
		if (u->lbsize < 512) u->lbsize=512;

		atapi_test_unit_ready(ac,drive);

		if (U_HAS_FLAG(u,UF_CDROM) || U_HAS_FLAG(u,UF_MOZIP)) {
			med = (U_HAS_FLAG(u,UF_HASMEDIA)) ? "Inserted"
							  : "Empty";
		}

		klass = atapi_class_name(u);

		printf("%s: ATAPI %s, model=\"%s\" %s (atapi_blksz=%ld)\n",
                       Cstr(ac),klass,u->model,med,u->atapi_blksz);

	} else { /* ATA disk branch */
		char 	*lba28 = u->lba_ok ? "LBA28" : "";
		unsigned long nsec = u->nsectors;

		ulong_t mib = nsec >> 11; 
		ulong_t gib_i = nsec / 2097152UL;
		ulong_t gib_tenths = ((nsec % 2097152UL) * 10UL) / 2097152UL;

		printf("%s: ATA disk, model=\"%s\" %s, (%lu.%u GiB)\n",
			Cstr(ac), u->model, lba28, gib_i,gib_tenths);

		u->lbsize=512;
	}
	u->lbshift=(u8_t)((u->lbsize >> 9) 
			? (u->lbsize==512 ?0:(u->lbsize==1024 ? 1 : 2)) : 0);

	ata_negotiate_pio_multiple(ac,drive);

	return 0;
}

void
ata_region_from_dev(dev_t dev, u32_t *out_base, u32_t *out_len)
{
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_unit_t *u = &ata_unit[ATA_UNIT(dev)];
	int	part  = ATA_PART(dev);
	int	slice = ATA_SLICE(dev);
	u32_t	base=0, len=0, bsz512;
	ata_part_t *fp = &u->fd[part];

	ATADEBUG(2,"ata_region_from_dev(%s base=%lu, start=%lu) ABSDEV=%d\n",
		Dstr(dev), (u32_t)fp->base_lba, 
		(u32_t)fp->slice[slice].p_start,
		ISABSDEV(dev));

	if (U_HAS_FLAG(u,UF_ATAPI)) {
        	bsz512 = (u->lbsize >> 9) ? (u->lbsize >> 9) : 1;
        	base = 0;
        	len  = u->atapi_blocks * bsz512;
	} else {
		if (ISABSDEV(dev)) {
			base = 0;
			len  = u->nsectors;
		} else {
			base = fp->base_lba;
			len  = fp->nsectors;

			if (fp->systid == UNIXOS) {
				if (slice != 0 && 
				    slice != ATA_WHOLE_PART_SLICE) {
					base += fp->slice[slice].p_start-1;
					len    = fp->slice[slice].p_size;
				}
			}
		}
	}
	ATADEBUG(1,"region_from_dev: %s part_base=%lu slice_start=%lu final_base=%lu\n",
		Dstr(dev),fp->base_lba,fp->slice[slice].p_start,base);

	*out_base = base;
	*out_len  = len;
	return;
}

int
ata_pdinfo(dev_t dev)
{
	int 	part  = ATA_PART(dev),
		slice = ATA_SLICE(dev),
		ctrl  = ATA_CTRL(dev),
		drive = ATA_DRIVE(dev);
	ata_ctrl_t *ac = &ata_ctrl[ctrl];
	ata_unit_t *u = ac->drive[drive];
	struct mboot *mboot;
	struct ipart *ip;
	struct buf *bp;
	ata_part_t *fp;
	int	i, s, rc;

	ATADEBUG(1,"ata_pdinfo(%s, dev=%x) drive=%d part=%d\n",	
		Dstr(dev),dev,drive,part);

	if (U_HAS_FLAG(u,UF_ATAPI)) return 0;

	mboot = (struct mboot *)kmem_alloc(DEV_BSIZE,KM_SLEEP);
	if (!mboot) return ENOMEM;

        if (ata_getblock(ABSDEV(dev),0,(caddr_t)mboot,DEV_BSIZE) != 0) {
		kmem_free((caddr_t)mboot,DEV_BSIZE);
		return EIO;
	}

	if (mboot->signature != MBB_MAGIC) {
		kmem_free((caddr_t)mboot, DEV_BSIZE);
		fp = &u->fd[ part ];
		ATADEBUG(1,"Part %d: start=0 p_size=%lu\n",
			part,fp->nsectors);
		fp->slice[ATA_WHOLE_PART_SLICE].p_start = 0;
		fp->slice[ATA_WHOLE_PART_SLICE].p_size  = fp->nsectors;
		return 0;
	}

	/*** Now copy the others in sequence ***/
	ip = (struct ipart *)&mboot->parts;
	for(i=0; i<FD_NUMPART; ip++) {
		fp = &u->fd[ i ];
		if (ip->systid == EMPTY) continue;
		CopyTbl(fp,ip);
		if (ip->systid == UNIXOS) {
			fp->slice[ATA_WHOLE_PART_SLICE].p_start = 0;
			fp->slice[ATA_WHOLE_PART_SLICE].p_size  = fp->nsectors;
			if (fp->nsectors > 0) {
				fp->vtoc_valid = ata_read_vtoc(dev, i);
			}
		}
		i++;
	}
	kmem_free((caddr_t)mboot,DEV_BSIZE);
	return 0;
}

int 	
ata_getblock(dev_t dev, daddr_t blkno, caddr_t buf, u32_t count)
{
	int	rc;
	struct buf *bp; 

	ATADEBUG(1,"ata_getblock(%x,%lu,%x,%lu)\n",dev,blkno,buf,count);

	if (!(bp = geteblk())) return EIO;

	bp->b_dev    = dev;
	bp->b_edev   = dev;
	bp->b_error  = 0;
	bp->b_resid  = 0;
	bp->b_flags  = B_READ | B_BUSY;
	bp->b_blkno  = blkno;
	bp->b_bcount = count;

	atastrategy(bp);
	iowait(bp);

	rc = (bp->b_flags & B_ERROR) ? EIO : 0;
	bcopy(bp->b_un.b_addr,buf,count);
	brelse(bp);

	return rc;
}

int 	
ata_putblock(dev_t dev, daddr_t blkno, caddr_t buf, u32_t count)
{
	int rc;
	struct buf *bp; 

	ATADEBUG(1,"ata_putblock(%x,%lu,%x,%lu)\n",dev,blkno,buf,count);

	if (!(bp = geteblk())) return EIO;

	bp->b_dev    = dev;
	bp->b_edev   = dev;
	bp->b_error  = 0;
	bp->b_resid  = 0;
	bp->b_flags  = B_WRITE | B_BUSY;
	bp->b_blkno  = blkno;
	bp->b_bcount = count;

	bcopy(buf, bp->b_un.b_addr, count);
	atastrategy(bp);
	iowait(bp);

	rc = (bp->b_flags & B_ERROR) ? EIO : 0;
	brelse(bp);
	return rc;
}

int
berror(struct buf *bp, int resid, int err)
{
	char *str = ISABSDEV(bp->b_edev) ? "<ABSDEV>" : "";

	ATADEBUG(3,"berror(%s)\n",str);
	bp->b_flags |= B_ERROR; 
	bp->b_error = err; 
	bp->b_resid = resid;
	biodone(bp); 
	return 0;
}

int
bok(struct buf *bp, int resid)
{
	char *str = ISABSDEV(bp->b_edev) ? "<ABSDEV>" : "";

	ATADEBUG(3,"bok(%s)\n",str);
	bp->b_flags &= ~B_ERROR; 
	bp->b_resid = resid;
	if (ISABSDEV(bp->b_edev) && !(bp->b_flags & B_READ)) {
		ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(bp->b_edev)];
		u8_t drive = ATA_DRIVE(bp->b_edev);
		bflush(bp->b_edev);
		ata_flush_cache(ac,drive);
	}
	biodone(bp);
	return 0;
}

void
ide_poll_engine(ata_ctrl_t *ac)
{
	ata_ioque_t *q;
	ata_req_t   *r;
	u8_t	ast;
	int		poll_burst;
	int		burst_done;

	q = ac ? ac->ioque : 0;
	r = q ? q->cur : 0;

	if (!r)
		return;

	/* Never run the poll engine in interrupt mode */
	if (AC_HAS_FLAG(ac, ACF_INTR_MODE))
		return;

	AC_SET_FLAG(ac, ACF_POLL_RUNNING);

	/* Max number of sectors to service per poll entry (avoid watchdog-only progress) */
	poll_burst = 32;
	burst_done = 0;

	/* Drive the state machine until we either need to yield or finish */
	for (;;) {
		/* Tiny yield to avoid a hot loop if the device is still busy */
		drv_usecwait(2);

		ata_err(ac, &ast, 0);
		ATADEBUG(5, "poll: ST=%02x\n", ast);

		if (ast & ATA_SR_BSY)
			continue;

		if (ast & ATA_SR_ERR) {
			ata_finish_current(ac, EIO, __LINE__);
			ide_kick(ac);
			break;
		}

		if (ast & ATA_SR_DRQ) {
			/* Progress: reset DRQ wait budget */
			r->await_drq_ticks = HZ * 2;

			if (ata_data_phase_service(ac, r) < 0) {
				ata_finish_current(ac, EIO, __LINE__);
				ide_kick(ac);
				break;
			}

			/* Count one sector serviced */
			burst_done++;

			/*
			 * If we have finished the programmed block (chunk_left == 0),
			 * handle end-of-chunk bookkeeping now (DRQ may remain asserted
			 * until status settles). Do not yield early on burst budget in
			 * this case, or we can leave the device in a DRQ state and trip
			 * the DRQ-clear check later.
			 */
			if (r->chunk_left == 0) {
				/* End of programmed block: require BSY|DRQ to drop before next cmd */
				{
					u8_t st2, er2;
					if (ata_wait(ac, 0, (ATA_SR_BSY|ATA_SR_DRQ), 50000, &st2, &er2) != 0) {
						cmn_err(CE_WARN, "%s: DRQ did not clear after data phase (st=%02x err=%02x)",
							Cstr(ac), st2, er2);
						ata_finish_current(ac, EIO, __LINE__);
						ide_kick(ac);
						break;
					}
				}

				if (r->sectors_left > 0) {
					ata_program_next_chunk(ac, r, HZ/8);
				} else {
					ata_finish_current(ac, 0, __LINE__);
					ide_kick(ac);
					break;
				}
				continue;
			}

			/* Still inside this programmed block; yield after a burst */
			if (burst_done >= poll_burst)
				break;

			/* PIO MULTIPLE: keep servicing while chunk_left > 0 */
			continue;
		}
		/* Inter-sector gap in multi-sector PIO (command still active) */
		if (r->chunk_left > 0 && r->sectors_left > 0) {
			if (r->await_drq_ticks > 0) {
				r->await_drq_ticks--;
				continue;
			}
			cmn_err(CE_WARN,
			    "%s: POLL DRQ timeout (st=%02x) lba=%lu secleft=%lu chunkleft=%lu",
			    Cstr(ac), ast, (u32_t)r->lba_cur, (u32_t)r->sectors_left,
			    (u32_t)r->chunk_left);
			ata_finish_current(ac, EIO, __LINE__);
			ide_kick(ac);
			break;
		}

		/* Waiting for first DRQ after issuing a command */
		if (r->sectors_left > 0 && r->chunk_left == 0) {
			if (r->await_drq_ticks > 0) {
				r->await_drq_ticks--;
				continue;
			}
			cmn_err(CE_WARN,
			    "%s: POLL no progress (st=%02x) lba=%lu secleft=%lu",
			    Cstr(ac), ast, (u32_t)r->lba_cur, (u32_t)r->sectors_left);
			ata_finish_current(ac, EIO, __LINE__);
			ide_kick(ac);
			break;
		}

		/* Completion edge in POLL mode */
		if (r->sectors_left == 0 && r->chunk_left == 0) {
			ata_finish_current(ac, 0, __LINE__);
			ide_kick(ac);
			break;
		}

		/*
		 * POLL mode needs to emulate the "completion interrupt" edge
		 * for cases where commands complete with no DRQ/data phase.
		 */
		ata_service_irq(ac, r, ast);
		continue;
	}

	if (AC_HAS_FLAG(ac, ACF_PENDING_KICK) &&
	    !AC_HAS_FLAG(ac, ACF_INTR_MODE)) {
		AC_CLR_FLAG(ac, ACF_PENDING_KICK);
		ide_kick_internal(ac);
	}

	AC_CLR_FLAG(ac, ACF_POLL_RUNNING);
}

/*
 * Dump the kernel message ring buffer (putbuf[]) to the console
 * in chronological order, without appending to putbuf again.
 */
void
dump_putbuf(void)
{
	int	s;
	int	ndx, sz;
	int	start, n, i;
	int	lines = 0;

	/*
	 * Block interrupts while we snapshot indices so they don't move
	 * underneath us. use whatever your tree uses for "block all"
	 */
	s = splhi();
	ndx = putbufndx;
	sz = putbufsz;
	splx(s);

	if (sz <= 0) return;

	/* How many valid characters are there? */
	if (ndx < sz) {
		/* Buffer has never wrapped. Valid range: [0 .. ndx-1] */
		start = 0;
		n = ndx;
	} else {
		/* Buffer has wrapped. Oldest bytes is at ndx % sz and there 
		 * are sz bytes of valid data
		 */
		start = ndx % sz;
		n = sz;
	}

	for(i=0; i<n; i++) {
		char c = putbuf[(start+i) % sz];

		/* Early boot messages may have embedded NULLs; you can either
		 * skip them or turn them into newlines. 
		 */
		if (c == '\0') continue;
		dbg_putchar(c);

		if (c == '\n') {
			int ch;

			if (lines++ < 24) continue;
			dbg_printf("--More--");

			ch = dbg_getchar() & 0x7f;
			dbg_putchar(BS); dbg_putchar(BS);	
			dbg_putchar(BS); dbg_putchar(BS);	
			dbg_putchar(BS); dbg_putchar(BS);	
			dbg_putchar(BS); dbg_putchar(BS);	
			dbg_putchar('\n');	

			switch(ch) {
			case '\r':
			case '\n':
				lines=23;
				break;
			case 'q':
			case 'Q':
				dbg_putchar('\n');
				return;
			default:
				lines=0;	
				break;
			}
		}
	}
}

char *
get_sysid(u8_t systid)
{
	switch(systid) {
	case EMPTY:		return "Empty";
	case UNUSED:		return "Unused";
	case FAT12:		return "FAT12";
	case PCIXOS:		return "PCIXOS";
	case FAT16:		return "FAT16";
	case EXTDOS:		return "EXTDOS";
	case NTFS:		return "NTFS";
	case DOSDATA:		return "DOSDATA";
	case OTHEROS:		return "OTHEROS";
	case SVR4:		return "Unix SVR4";
	case LINUXSWAP:		return "Linux Swap";
	case LINUXNATIVE:	return "Linux";
	case BSD386:		return "BSD386";
	case OPENBSD:		return "OpenBSD";
	case NETBSD:		return "NetBSD";
	case SOLARIS:		return "Solaris";
	}
	return "??";
}

void
ata_dump_fdisk(int ctrl, u8_t drive)
{
	ata_ctrl_t *ac= &ata_ctrl[ctrl];
	ata_unit_t *u=ac->drive[drive];
	int	fdisk, s;

	ATADEBUG(1,"ata_dump_fdisk(%s)\n",Cstr(ac));

	if (!U_HAS_FLAG(u,UF_PRESENT)) return;
	if (U_HAS_FLAG(u,UF_ATAPI)) return;

	for(fdisk=0; fdisk<4; fdisk++) {
		ata_part_t *fp=&u->fd[fdisk];

		u32_t 	base   = fp->base_lba;
		u32_t 	nsec   = fp->nsectors;
		int 	valid  = fp->vtoc_valid;
		u8_t	active = fp->active;
		char	*systid;
		int	any;

		for(s=0; s<ATA_NPART; s++) {
			if (fp->slice[s].p_size != 0) {
				any=1;
				break;
			}
		}
		if (!any && base == 0 && nsec == 0 && !valid) continue;

		systid = get_sysid(u->fd[fdisk].systid);
		printf(" %cPart=%d: Type=%-10s base_lba=%lu size=%lu %s\n",
			active ? '+' : ' ',
			fdisk, systid,
			(ulong_t)base, (ulong_t)nsec,
			valid ? "vtoc=VALID" : "");

		for(s=0;s<ATA_NPART; s++) {
			u32_t ps = fp->slice[s].p_start;
			u32_t sz = fp->slice[s].p_size;
			if (sz == 0) continue;

			printf("   %cs%02d: start=%8lu size=%8lu\n",
				(s==ATA_WHOLE_PART_SLICE)?'*':' ',
				s,(ulong_t)ps,(ulong_t)sz);
		}
	}
}
