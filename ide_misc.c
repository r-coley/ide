/*
 * ide_misc.c
 */
#include "ide.h"
#include <stdarg.h>
#include <sys/cmn_err.h>

extern int ata_major;

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
	char 	buf[256];
	int 	i, s;

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

	/* Optional console mirroring */
	if (ata_debug_console) {
		printf("%s", buf);
		return;
	}

	/*
	 * Write to putbuf directly so debug is available via /dev/osm
	 * without relying on prt_where routing (avoids interleaving issues).
	 * One wakeup per message.
	 */
	s = splhi();
	for (i = 0; buf[i] != '\0'; i++)
		putbuf[putbufndx++ % putbufsz] = buf[i];
	splx(s);

	wakeup((caddr_t)putbuf);
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
static	char	buf[64];
	char	*suffix;

	if (ac == NULL) {
		sprintf(buf,"c?d?");
		return buf;
	}
	if (ac->sel_drive < 0 || ac->sel_drive > 1) {
		sprintf(buf,"c%dd?",ac->idx);
		return buf;
	}

	suffix=ISABSDEV(ATA_DEV(ac->idx,ac->sel_drive)) ? " <ABSDEV>" : "";
	sprintf(buf,"c%dd%d%s",ac->idx,ac->sel_drive,suffix);
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
	wakeup((caddr_t)ac->ioque);
}

void
ata_attach(int ctrl)
{
	ata_ctrl_t *ac= &ata_ctrl[ctrl];
	int 	drive;

	ATADEBUG(1,"ata_attach(%d)\n",ctrl);

	if (!AC_HAS_FLAG(ac,ACF_PRESENT)) return;

	if (ata_intr_mode || atapi_intr_mode) {
		RegisterIRQ(ac->irq,&ataintr, SPL5, INTR_TRIGGER_EDGE);
		AC_SET_FLAG(ac,ACF_INTR_MODE);
	}
	ata_softreset_ctrl(ac);
	for (drive = 0; drive <= 1; drive++) {
		ata_unit_t *u = ac->drive[drive];

		bzero((caddr_t)u,sizeof(*u));
		u->pio_multi=1;		/* 1 sector xfers */
		u->atapi_blocks = 0;
		u->atapi_blksz  = 0;
		u->drive=drive;
		ata_probe_unit(ac,drive,&u->devtype);
		/*printf("ctrl=%d drive=%d type=%x\n",ctrl,drive,u->devtype);*/
	}

	for (drive = 0; drive <= 1; drive++) {
		ata_unit_t *u = ac->drive[drive];

		if (u->devtype == DEV_UNKNOWN) continue;
		if (u->devtype & DEV_ATAPI) U_SET_FLAG(u,UF_ATAPI);
		if (ata_id_unit(ac,u) != 0) {
			U_CLR_FLAG(u,UF_PRESENT);
			u->devtype = DEV_UNKNOWN;
		}
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
	int	isataroot = (getmajor(rootdev) == ata_major) ? 1 : 0;
	int	isataswap = (getmajor(swapdev) == ata_major) ? 1 : 0;

	ATADEBUG(1,"ata_read_vtoc(%s dev=%x) isataroot=%d isataswap=%d\n",
		Dstr(dev),BASEDEV(dev),isataroot,isataswap);

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
		if (isataroot && isataswap &&
		    fp->slice[s].p_tag == V_SWAP &&
		    fp->slice[s].p_flag & V_VALID) {
				int 	devu = ATA_DEV_UNIT(dev);
				int 	swapu = ATA_DEV_UNIT(swapdev);

				if (devu == swapu && nswap == 0) 
					nswap = fp->slice[s].p_size;
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
ata_probe_unit(ata_ctrl_t *ac, u8_t drive,u16_t *type)
{
	u8_t 	st, lc=0, hc=0, sc, sn;
	u16_t	dev=DEV_UNKNOWN;

	*type = dev;
	ATADEBUG(1,"ata_probe_unit(drive=%d) lbolt=%ld\n",drive,lbolt);
	if (!AC_HAS_FLAG(ac,ACF_PRESENT))
		return EIO;

	if (ata_sel(ac,drive,0) != 0)
		return EIO;

	/* Wait for not-BSY */
	if (ata_wait(ac, 0, ATA_SR_BSY, 500000L, &st, 0) != 0)
		return EIO;

	sc = inb(ATA_SECTCNT_O(ac));
	sn = inb(ATA_SECTNUM_O(ac));
	lc = inb(ATA_CYLLOW_O(ac));
	hc = inb(ATA_CYLHIGH_O(ac));
	/* printf("read_signature() sc=%02x sn=%02x lc=%02x hc=%02x\n",
		sc,sn,lc,hc);*/

	/*** No device check - floating bus ***/
	if (sc == 0xff && sn == 0xff && lc == 0xff && hc == 0xff) {
		printf("Floating BUS\n");
		return ENXIO;
	}

	if (sc == 0x01 && sn == 0x01) {
		switch ((hc<<8)|lc)
		{
		case 0x0000: dev = DEV_ATA|DEV_PARALLEL; 	break;
		case 0x0800: dev = DEV_ATA|DEV_PARALLEL; 	break;
		case 0xC33C: dev = DEV_ATA|DEV_SERIAL; 		break;
		case 0xEB14: dev = DEV_ATAPI|DEV_PARALLEL; 	break;
		case 0x9669: dev = DEV_ATAPI|DEV_SERIAL; 	break;
		default:     dev = DEV_UNKNOWN;			break;
		}
	}
	*type = dev;
	return (dev == DEV_UNKNOWN) ? -1 : 0;
}

int
ata_id_unit(ata_ctrl_t *ac, ata_unit_t *u)
{
	char 	*klass;

	ATADEBUG(3,"ata_id_unit(%d) lbolt=%ld\n",u->drive,lbolt);
	if (!AC_HAS_FLAG(ac,ACF_PRESENT)) return ENXIO;
	if (ata_identify(ac, u->drive) != 0) return ENXIO;
 
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
		(void)atapi_inquiry(ac, u->drive);

		if ((atapi_read_capacity(ac,u->drive,&blocks,&blksz) == 0) &&
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

		if (atapi_test_unit_ready(ac,u->drive) != 0)
			ATADEBUG(1,"atapi_test_unit_ready failed\n");

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

		ulong_t gib_i = nsec / 2097152UL;
		ulong_t gib_tenths = ((nsec % 2097152UL) * 10UL) / 2097152UL;

		printf("%s: ATA disk, model=\"%s\" %s, (%lu.%u GiB)\n",
			Cstr(ac), u->model, lba28, gib_i,gib_tenths);

		u->lbsize=512;
	}
	u->lbshift=(u8_t)((u->lbsize >> 9) 
			? (u->lbsize==512 ?0:(u->lbsize==1024 ? 1 : 2)) : 0);

	ata_negotiate_pio_multiple(ac,u->drive);

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
					/*** -1 is on purpose ***/
					base += fp->slice[slice].p_start-1;
					len   = fp->slice[slice].p_size;
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
	ata_ctrl_t *ac;
	ata_unit_t *u;
	struct mboot *mboot;
	struct ipart *ip;
	struct buf *bp;
	ata_part_t *fp;
	int	i, s, rc;

	if (ctrl < 0 || ctrl>ATA_MAX_CTRL) return ENODEV;
	ac = &ata_ctrl[ctrl];
	u = ac->drive[drive];

	ATADEBUG(1,"ata_pdinfo(%s, dev=%x) drive=%d part=%d\n",	
		Dstr(dev),dev,drive,part);

	if (U_HAS_FLAG(u,UF_ATAPI)) return 0;

	/* Assume fdisk table is not valid unless we successfully read and
	 * recognize an mboot. ataopen/ataclose print this for debugging
	 */
	u->fdisk_valid = 0;

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

	/*** We successfully read and recognized an mboot ***/
	u->fdisk_valid = 1;

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
	static int busy = 0;
	int	s, rc;
	struct buf *bp; 

	ATADEBUG(1,"ata_getblock(%x,%lu,%x,%lu)\n",dev,blkno,buf,count);

	/*
	 * Serialize ata_getblock() to prevent recursive entry.
	 * This is required on SVR4 because completion/wakeup paths
	 * can re-enter while the original call has not unwound
	 */
	s=splbio();
	while (busy)
		sleep((caddr_t)&busy,PRIBIO);
	busy=1;
	splx(s);

	if (!(bp = geteblk()))
		rc=EIO;
	else {
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
		if (!rc)
			bcopy(bp->b_un.b_addr,buf,count);
		brelse(bp);
	}

	s = splbio();
	busy=0;
	wakeup((caddr_t)&busy);
	splx(s);

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
		ata_unit_t *u=ac->drive[drive];

		bflush(bp->b_edev);
		/* FLUSH CACHE (0xE7) is ATA-Only; ATAPI will ABRT it */
		if (!U_HAS_FLAG(u,UF_ATAPI)) {
			if (ata_flush_cache(ac,drive) != 0) 
				cmn_err(CE_WARN,"ide: flush cache failed ctrl=%d drive=%d\n",ATA_CTRL(bp->b_edev),drive);
		}
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

	/* Prevent recursive entry (watchdog/kick/copyin paths) */
	if (AC_HAS_FLAG(ac, ACF_POLL_RUNNING)) {
		AC_SET_FLAG(ac, ACF_PENDING_KICK);
		return;
	}

	AC_SET_FLAG(ac, ACF_POLL_RUNNING);

	/*
	 * Max number of sectors to service per poll entry (avoid 
	 * watchdog-only progress) 
	 */
	poll_burst = 32;
	burst_done = 0;

	/* Drive the state machine until we either need to yield or finish */
	for (;;) {
		/* Tiny yield to avoid a hot loop if the device is still busy */
		drv_usecwait(2);

		ata_err(ac, &ast, 0);
		ATADEBUG(5, "poll: ST=%02x\n", ast);

		if (ast & ATA_SR_BSY) continue;

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
			 * If we have finished the programmed block 
			 * (chunk_left == 0), handle end-of-chunk bookkeeping 
			 * now (DRQ may remain asserted until status settles). 
			 * Do not yield early on burst budget in this case, or 
			 * we can leave the device in a DRQ state and trip
			 * the DRQ-clear check later.
			 */
			if (r->chunk_left == 0) {
				u8_t st2, er2;

				/*
				 * End of programmed block: require BSY|DRQ 
				 * to drop before next cmd 
				 */
				if (ata_wait(ac, 0, (ATA_SR_BSY|ATA_SR_DRQ), 50000, &st2, &er2) != 0) {
					cmn_err(CE_WARN, "%s: DRQ did not clear after data phase (st=%02x err=%02x)",
						Cstr(ac), st2, er2);
					ata_finish_current(ac, EIO, __LINE__);
					ide_kick(ac);
					break;
				}

				ata_copyback_chunk_if_needed(ac,r);

				if (r->sectors_left > 0) {
					if (ata_program_next_chunk(ac, r, HZ/8) != 0) {
						ata_finish_current(ac, EIO, __LINE__);
						ide_kick(ac);
					}
				} else {
					ata_finish_current(ac, 0, __LINE__);
					ide_kick(ac);
					break;
				}
				continue;
			}

			/* Still inside programmed block; yield after burst */
			if (burst_done >= poll_burst) break;

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
