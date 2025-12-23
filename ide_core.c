#include "ide.h"

ata_unit_t ata_unit[ATA_MAX_UNITS]; /* up to 2 drives per controller */

void
ataprint(dev_t dev, char *str)
{
	printf("ata: %s %s\n", str ? str : "", Dstr(dev));
}

int
ataopen(dev_t *devp, int flags, int otyp, cred_t *crp)
{
	dev_t 	dev = *devp;
	int 	fdisk = ATA_PART(dev),
		slice = ATA_SLICE(dev),
		ctrl  = ATA_CTRL(dev),
		drive = ATA_DRIVE(dev);
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_ioque_t *q = ac->ioque;
	ata_unit_t *u = ac->drive[drive];
	ata_part_t *fp=&u->fd[fdisk];
	int	s;

	ATADEBUG(1,"ataopen(%s) present=%d dev=%x part=%d ctrl=%d driv=%d slice=%d\n",
		Dstr(dev),
		U_HAS_FLAG(u,UF_PRESENT),
		dev,fdisk,ctrl,drive,slice);

	if (!U_HAS_FLAG(u,UF_PRESENT)) return ENXIO;
	if ((u->read_only || 
	     U_HAS_FLAG(u,UF_CDROM)) && (flags & FWRITE)) return EROFS;

	if (ISABSDEV(dev)) return ENXIO;

	s=splbio();
	if (AC_HAS_FLAG(ac, ACF_CLOSING)) {
		splx(s);
		return EBUSY;
	}

	/* Channel-scoped first-open allocation of bounce buffer */
	if (q->open_count == 0) {

		/*** Read the partition table ***/
		if (ata_pdinfo(ABSDEV(dev)) != 0) {
			splx(s);
			return ENXIO;
		}

    		if (q->xfer_buf == 0) {
        		q->xfer_buf = (caddr_t)kmem_zalloc(ATA_XFER_BUFSZ, KM_SLEEP);
        		if (q->xfer_buf == 0)
            			return ENOMEM;
        		q->xfer_bufsz = ATA_XFER_BUFSZ;
			ATADEBUG(5,"%s: xfer_buf=%lx\n",Cstr(ac),q->xfer_buf);
    		}
		AC_CLR_FLAG(ac,ACF_BUSY); 
		q->state	= AS_IDLE;
		q->cur		= NULL; /* opencount==0 */
		ac->tmo_id	= 0;
	}
	
	AC_CLR_FLAG(ac,ACF_CLOSING); 
	splx(s);

	if (U_HAS_FLAG(u,UF_ATAPI)) {
		int	rc;
		u32_t	blocks, blksz;

		if (!U_HAS_FLAG(u,UF_HASMEDIA)) {
			printf("No media in drive\n");
			return ENXIO;
		}

		rc=atapi_read_capacity(ac,drive,&blocks,&blksz);
		if (rc != 0) {
			if (blksz == 0) blksz = 2048;
			if (blocks == 0) blocks = 0;
		}
		u->atapi_blksz=(blksz ? blksz : 2048);
		u->atapi_blocks=blocks;
		u->nsectors = (u32_t)(blocks*(u->atapi_blksz >> 9));
		ATADEBUG(1,"ataopen() atapi_blksz=%ld atapi_blocks=%ld\n",
			 u->atapi_blksz, u->atapi_blocks);

		if (slice == 0 || slice == ATA_WHOLE_PART_SLICE) goto ok;
		return ENXIO;
	}

	if (!fp->vtoc_valid) {
		if (fp->systid == UNIXOS) {
			if (fp->nsectors > VTOC_SEC) goto ok;
		}
		if (slice == 0 || slice == ATA_WHOLE_PART_SLICE) goto ok;
		return ENXIO;
	}

	if (slice < 0 || slice > ATA_NPART-1) {
		return ENXIO;
	}

	if (!fp->vtoc_valid || 
	     fp->slice[slice].p_size == 0) {
		return ENXIO;
	}

ok:
	q->open_count++;
	return 0;
}

int
ataclose(dev_t dev, int flags, int otyp, cred_t *crp)
{
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_ioque_t *q = ac->ioque;
	ata_unit_t *u = ac->drive[ATA_DRIVE(dev)];
	ata_part_t *fp=&u->fd[ATA_PART(dev)];
	int	s;

	ATADEBUG(1,"ataclose(%s) present=%d fdisk_valid=%d vtoc_valid=%d\n",
		Dstr(dev),
		U_HAS_FLAG(u,UF_PRESENT),
		u->fdisk_valid,fp->vtoc_valid);

	if (!U_HAS_FLAG(u,UF_PRESENT)) return ENODEV;

	s=splbio();
	AC_SET_FLAG(ac, ACF_CLOSING);
	while (AC_HAS_FLAG(ac,ACF_BUSY) || q->q_head) {
		ATADEBUG(2,"busy=%d q_head=%lx\n",AC_HAS_FLAG(ac,ACF_BUSY),q->q_head);
		sleep((caddr_t)ac->ioque,PRIBIO);
	}

	if (q->open_count > 0) q->open_count--;

	if (q->open_count == 0) {
		if (q->xfer_buf) {
			ATADEBUG(5,"ataclose(%s: free %lx\n",
				Cstr(ac),q->xfer_buf);
			kmem_free(q->xfer_buf,q->xfer_bufsz);
			q->xfer_buf  = 0;
			q->xfer_bufsz = 0;
		}
		u->fdisk_valid=0;
		reset_queue(ac,0);
	}
	AC_CLR_FLAG(ac,ACF_CLOSING);
	splx(s);
	return 0;

}

void
atabreakup(struct buf *bp)
{
	pio_breakup(atastrategy, bp, MAXNBLKS);
}

int
atastrategy(struct buf *bp)
{
	int	dev=bp->b_edev, is_wr, s, do_kick;
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_unit_t *u = ac->drive[ATA_DRIVE(dev)];
	ata_ioque_t *q = ac->ioque;
	ata_req_t *r;
	u32_t	base, len, lba, left;
	char	*addr;

        ATADEBUG(1,"atastrategy(%s) %s lba=%lu nsec=%lu flags=%x\n", 
		Dstr(dev), 
		(bp->b_flags&B_READ)?"READ":"WRITE",
		bp->b_blkno, 
		(bp->b_bcount>>DEV_BSHIFT),
		bp->b_flags);

	ata_region_from_dev(dev,&base,&len);
	
	r = (ata_req_t *)kmem_zalloc(sizeof(*r),KM_SLEEP);
	if (!r) return berror(bp,0,ENOMEM);

	r->is_write = (bp->b_flags & B_READ) ? 0 : 1;
	r->reqid    = req_seq++;
	r->drive    = ATA_DRIVE(dev);
	r->addr     = (char *)bp->b_un.b_addr;
	r->bp 	    = bp;

	if (U_HAS_FLAG(u,UF_ATAPI)) {
		u32_t blksz = u->atapi_blksz ? u->atapi_blksz : 2048;
		u32_t bsz512 = blksz >> 9;
		if (bsz512 == 0) bsz512=1;

		r->lba          = base / bsz512 + (u32_t)bp->b_blkno / bsz512;
		r->lba_cur      = r->lba;
		r->nsec         = (u32_t)(bp->b_bcount / blksz);
		r->sectors_left = r->nsec;
		r->cmd		= ATA_CMD_PACKET;
		r->atapi_phase	= ATAPI_PHASE_WAIT_PKT_DRQ;
		r->atapi_dir	= r->is_write ?ATAPI_DIR_WRITE :ATAPI_DIR_READ;
		r->atapi_use_dma= 0;
		ATADEBUG(1,"r->lba=%ld r->nsec=%ld\n",r->lba, r->nsec);
	} else {
		r->lba          = base + (u32_t)bp->b_blkno;
		r->lba_cur      = r->lba;
		r->nsec         = (u32_t)(bp->b_bcount >> 9);
		r->sectors_left = r->nsec;
		r->cmd 		= multicmd(ac,r->is_write,bp->b_blkno,r->nsec);
	}

	ata_pushreq(ac,r);
	return 0;
}

int
ataread(dev_t dev, struct uio *uiop, cred_t *crp)
{
	int 	fdisk = ATA_PART(dev),
		slice = ATA_SLICE(dev);
	ata_unit_t *u = &ata_unit[ATA_UNIT(dev)];

	ATADEBUG(1,"ataread(%s)\n",Dstr(dev));

	return physiock(atabreakup, 
			NULL, 
			dev, 
			B_READ, 
			(daddr_t)(u->fd[fdisk].slice[slice].p_size),
			uiop);
}

int
atawrite(dev_t dev, struct uio *uiop, cred_t *crp)
{
	int 	fdisk = ATA_PART(dev),
		slice = ATA_SLICE(dev);
	ata_unit_t *u = &ata_unit[ATA_UNIT(dev)];

	ATADEBUG(1,"atawrite(%s)\n",Dstr(dev));

	return physiock(atabreakup, 
			NULL, 
			dev, 
			B_WRITE, 
			(daddr_t)(u->fd[fdisk].slice[slice].p_size),
			uiop);
}

int
ataioctl(dev_t dev, int cmd, caddr_t arg, int mode, cred_t *crp, int *rvalp)
{
	ata_ctrl_t *ac = &ata_ctrl[ATA_CTRL(dev)];
	ata_unit_t *u = ac->drive[ATA_DRIVE(dev)];
	int 	fdisk = ATA_PART(dev);
	int	drive = u->drive;
	ata_part_t *fp=&u->fd[fdisk];

	ATADEBUG(1,"ataioctl(%s,%s)\n",Dstr(dev),Istr(cmd));

	if (!U_HAS_FLAG(u,UF_PRESENT)) return ENODEV;

	switch (cmd) {
	case V_CONFIG: 		/* VIOC | 0x01 */
		return 0;

	case V_REMOUNT: /* VIOC | 0x02 */
		if (fp->nsectors > 0 && fp->systid == UNIXOS) 
			fp->vtoc_valid = ata_read_vtoc(dev,fdisk);
		return 0;
	
	case V_GETPARMS: {	/* VIOC | 0x04 */
		struct disk_parms dp;

		/*
		 * We dont have geometry; supply logical CHS compatible 
		 * with 63/16 
		 */
		dp.dp_type   = 1;
		dp.dp_heads  = 16;
		dp.dp_sectors= 63;
		dp.dp_cyls   = (u16_t)(u->nsectors/(dp.dp_heads*dp.dp_sectors));
		dp.dp_secsiz = DEV_BSIZE;
		dp.dp_ptag   = 0;
		dp.dp_pflag  = 0;
		/*dp.dp_pstartsec = 0;*/
		dp.dp_pstartsec = 1;
		dp.dp_pnumsec  = u->nsectors;

		if (copyout((caddr_t)&dp, arg, sizeof(dp)) < 0)
			return EFAULT;
		return 0;
	    }

	case V_FORMAT: 		/* VIOC | 0x05 */
		return 0;

	case V_PDLOC: {		/* VIOC | 0x06 */
		u32_t	vtocloc = VTOC_SEC;

		if (copyout((caddr_t)&vtocloc, arg,sizeof(vtocloc)) != 0)
			return EFAULT;
		return 0;
	    }
	
	case V_RDABS: {		/* VIOC | 0x0A */
        	struct absio ab;
		caddr_t	ptr;

		printf("copyin\n");
        	if (copyin(arg, (caddr_t)&ab, sizeof(ab)) != 0) return EFAULT;
        	if (ab.abs_buf == 0) return EINVAL;

		if (!(ptr = (caddr_t)kmem_alloc(ATA_SECSIZE,KM_SLEEP))) {
			return ENOMEM;
		}
		printf("call ata_getblock(%d)\n",ab.abs_sec);
    		if (ata_getblock(ABSDEV(dev),ab.abs_sec,ptr,ATA_SECSIZE)) {
			kmem_free(ptr,ATA_SECSIZE);
			return EFAULT;
		}
		printf("call copyout()\n");
                if (copyout(ptr,ab.abs_buf,ATA_SECSIZE) != 0) {
			kmem_free(ptr,ATA_SECSIZE);
			return EFAULT; 
		}
		kmem_free(ptr,ATA_SECSIZE);
                return 0;
	    }
	
	case V_WRABS: {		/* VIOC | 0x0B */
        	struct absio ab;
		caddr_t	ptr;

        	if (copyin(arg,(caddr_t)&ab,sizeof(ab)) != 0) return EFAULT;
        	if (ab.abs_buf == 0) return EINVAL;

		if (!(ptr = (caddr_t)kmem_alloc(ATA_SECSIZE,KM_SLEEP))) {
			return ENOMEM;
		}
                if (copyin((caddr_t)ab.abs_buf,ptr,ATA_SECSIZE) != 0) { 
			kmem_free(ptr,ATA_SECSIZE);
			return EFAULT; 
		} 
    		if (ata_putblock(ABSDEV(dev),ab.abs_sec,ptr,ATA_SECSIZE) != 0) {
			kmem_free(ptr,ATA_SECSIZE);
			return EIO;
		}
		kmem_free(ptr,ATA_SECSIZE);
                return 0;
	    }

	case V_VERIFY: {	/* VIOC | 0x0C */
    		union vfy_io vfy;

		vfy.vfy_out.err_code=0;
		if (copyout((caddr_t)&vfy,arg,sizeof(vfy)) < 0)
			return EFAULT;
		return 0;
	    }

	case V_GETTYPE: {
		struct v_gettype gt;

		gt.flags = u->flags & UF_USER_MASK;
		gt.model[0] = '0';
		strncpy(gt.model, u->model, sizeof(gt.model) -1);
		gt.model[sizeof(gt.model)-1]=0;
		strncpy(gt.product, u->product, sizeof(gt.product) -1);
		gt.product[sizeof(gt.product)-1]=0;
		strncpy(gt.vendor, u->vendor, sizeof(gt.vendor) -1);
		gt.vendor[sizeof(gt.vendor)-1]=0;
		
		if (copyout((caddr_t)&gt,arg,sizeof(gt)) != 0) return EFAULT;
		return 0;
	    }

	/* --- Private ATAPI CD-ROM TOC / audio controls --- */
	case CDIOC_READTOC: {
		cd_toc_io_t tio;
		u8_t tocbuf[4096];
		int rc;
		ushort maxlen, actual, dlen;

		if (!U_HAS_FLAG(u,UF_ATAPI) || !U_HAS_FLAG(u,UF_CDROM))
			return ENOTTY;

		if (copyin(arg, (caddr_t)&tio, sizeof(tio)) != 0)
			return EFAULT;

		maxlen = tio.toc_len;
		if (maxlen == 0 || maxlen > sizeof(tocbuf))
			maxlen = sizeof(tocbuf);

		rc = atapi_read_toc(ac, (u8_t)drive,
				 (int)tio.msf,
				 tio.format,
				 tio.track,
				 tocbuf, maxlen);
		if (rc != 0)
			return EIO;

		/* TOC length is stored in first two bytes (big-endian). */
		dlen = (tocbuf[0] << 8) | tocbuf[1];
		/* Total bytes available = dlen + 2 header bytes. */
		actual = dlen + 2;
		if (actual > maxlen)
			actual = maxlen;

		if (copyout((caddr_t)tocbuf, (caddr_t)tio.toc_buf, actual) != 0)
			return EFAULT;

		/* Return actual length to caller. */
		tio.toc_len = actual;
		if (copyout((caddr_t)&tio, arg, sizeof(tio)) != 0)
			return EFAULT;

		return 0;
	}

	case CDIOC_PLAYMSF: {
		cd_msf_io_t msf;

		if (!U_HAS_FLAG(u,UF_ATAPI) || !U_HAS_FLAG(u,UF_CDROM))
			return ENOTTY;

		if (copyin(arg, (caddr_t)&msf, sizeof(msf)) != 0)
			return EFAULT;

		if (atapi_play_audio_msf(ac, (u8_t)drive,
				 msf.start_m, msf.start_s, msf.start_f,
				 msf.end_m,   msf.end_s,   msf.end_f) != 0)
			return EIO;

		return 0;
	}


	case CDIOC_PAUSE: {
		if (!U_HAS_FLAG(u,UF_ATAPI) || !U_HAS_FLAG(u,UF_CDROM))
			return ENOTTY;

		if (atapi_pause_resume(ac, (u8_t)drive, 0) != 0)
			return EIO;

		return 0;
	}

	case CDIOC_RESUME: {
		if (!U_HAS_FLAG(u,UF_ATAPI) || !U_HAS_FLAG(u,UF_CDROM))
			return ENOTTY;

		if (atapi_pause_resume(ac, (u8_t)drive, 1) != 0)
			return EIO;

		return 0;
	}

	case CDIOC_EJECT: {
		if (!U_HAS_FLAG(u,UF_ATAPI) || !U_HAS_FLAG(u,UF_CDROM))
			return ENOTTY;

		/* LoEj=1, Start=0 -> eject / tray open */
		if (atapi_start_stop(ac, (u8_t)drive, 0, 1) != 0)
			return EIO;

		return 0;
	}

	case CDIOC_LOAD: {
		if (!U_HAS_FLAG(u,UF_ATAPI) || !U_HAS_FLAG(u,UF_CDROM))
			return ENOTTY;

		/* LoEj=1, Start=1 -> load / tray close and spin up */
		if (atapi_start_stop(ac, (u8_t)drive, 1, 1) != 0)
			return EIO;

		return 0;
	}

	case CDIOC_SUBCHANNEL: {
		cd_subchnl_io_t sci;

		if (!U_HAS_FLAG(u,UF_ATAPI) || !U_HAS_FLAG(u,UF_CDROM))
			return ENOTTY;

		if (atapi_read_subchnl(ac, (u8_t)drive, 1, &sci) != 0)
			return EIO;

		if (copyout((caddr_t)&sci, (caddr_t)arg, sizeof(sci)) != 0)
			return EFAULT;
	
		return 0;
	}

	default:
		printf("Unknown IOCTL %x\n",cmd);
		return ENOTTY;
	}
}

int
atasize(dev_t dev)
{
	int 	fdisk = ATA_PART(dev),
		slice = ATA_SLICE(dev);
	ata_unit_t *u = &ata_unit[ATA_UNIT(dev)];

	if (!U_HAS_FLAG(u,UF_PRESENT)) return -1;

	if (U_HAS_FLAG(u,UF_ATAPI) || !u->fd[fdisk].vtoc_valid)
		return (int)u->nsectors;

	return (int)u->fd[fdisk].slice[slice].p_size;
}

int
atainit(void)
{
	int 	ctrl; 
	ata_ioque_t *q;
	ata_ctrl_t *ac;
	ata_counters_t *counters;

	ATADEBUG(1,"atainit()\n");

	/*** Hack - we mustn't run before bio subsystem has initialised ***/
	if (bfreelist.av_forw == NULL) binit();

	bzero((caddr_t)&ata_unit[0],sizeof(ata_unit_t)*ATA_MAX_UNITS);
	for (ctrl = 0; ctrl < ATA_MAX_CTRL; ctrl++) {
		ac = &ata_ctrl[ctrl];
		ac->idx = ctrl;
		ac->multi_set_ok = 0; /* disable MULTIPLE until proven */

		q = (ata_ioque_t *)kmem_zalloc(sizeof(ata_ioque_t),KM_SLEEP);
		if (!q) return -1;
		counters = (ata_counters_t *)kmem_zalloc(sizeof(ata_counters_t),KM_SLEEP);
		if (!counters) return -1;

		ac->ioque = q;
		ac->counters = counters;

		ac->sel_drive = -1;
		ac->sel_hi4 = 0xff;
		ac->sel_mode = 0;

		ac->drive[ 0 ] = &ata_unit[ATA_UNIT_FROM(ctrl,0)];
		ac->drive[ 1 ] = &ata_unit[ATA_UNIT_FROM(ctrl,1)];

		ac->drive[0]->drive = 0;
		ac->drive[1]->drive = 1;

		if (AC_HAS_FLAG(ac,ACF_PRESENT)) ata_attach(ctrl);

		if (AC_HAS_FLAG(ac,ACF_INTR_MODE)) {
			/*
			 * If in Interrupt mode as opposed POLL mode
			 * ACF_INTR_MODE will be set. Force Clear the flag
			 * that is used to cache whether interrupts are on
			 * and then Enable Interrupts
			 */
			AC_CLR_FLAG(ac,ACF_IRQ_ON); 
			ATA_IRQ_ON(ac);
		}
	}
	return 0;
}

ata_ctrl_t *
atafindctrl(int irq)
{
	ata_ctrl_t *ac;
	int 	ctrl;

	for (ctrl = 0; ctrl < ATA_MAX_CTRL; ctrl++) {
		ac=&ata_ctrl[ctrl];
		if (ac->irq == irq && AC_HAS_FLAG(ac,ACF_PRESENT)) return ac;
	}
	return DDI_INTR_UNCLAIMED;
}

/* ---------- interrupt handler (ATA PIO, per-sector handshakes) ---------- */
int
ataintr(int irq)
{
	int 	ctrl;
	ata_ctrl_t *ac;
	ata_ioque_t *q;
	ata_req_t *r;
	ata_unit_t *u;
	int	drive, er;
	u8_t 	st, ast, err, drvs;
	u32_t	lba;

	ATADEBUG(1,"ataintr(%d)\n",irq);

	if ((ac=atafindctrl(irq)) == (ata_ctrl_t *)0)
		return DDI_INTR_UNCLAIMED;

	q = ac->ioque;
	r = q ? q->cur : NULL;	/* ataintr() */

	BUMP(ac,irq_seen);
	st=inb(ATA_STATUS_O(ac)); /* ataintr() */
 	err=(st & (ATA_SR_ERR|ATA_SR_DWF)) ? inb(ATA_ERROR_O(ac)) : 0;
	if (err) {
		printf("Drive ERR:DWF\n");
	}

	if (!AC_HAS_FLAG(ac, ACF_INTR_MODE)) {
		BUMP(ac,irq_spurious);
		return DDI_INTR_UNCLAIMED;
	}

	if (atapi_force_polling) {
		inb(ATA_ALTSTATUS_O(ac));
		BUMP(ac,irq_atapi_ignored);
		return DDI_INTR_CLAIMED;
	}

	if (!r) { 
		ast=inb(ATA_ALTSTATUS_O(ac));
		drvs=inb(ATA_DRVHD_O(ac));
		u = ac->drive[ (drvs & ATA_DH_DRV) ? 1 : 0 ];

		BUMP(ac,irq_no_cur);
		if (U_HAS_FLAG(u,UF_ATAPI)) {
 			if (ast & ATA_SR_ERR) 
				U_SET_FLAG(u,UF_ATAPI_NEEDS_SENSE);
			return DDI_INTR_CLAIMED;
		}
		
		printf("STRAY %s: ST=%02x ERR=%02x AST=%02x q=%p cur=%p last: cmd=%02x lba=%ld sc=%d dh=%02x reqid=%d age=%lu\n",
			Cstr(ac),st,err,ast,
			q,q?q->cur:NULL,
			ac->lc.cmd,ac->lc.lba,ac->lc.sc,ac->lc.dh,ac->lc.reqid,
			(lbolt-ac->lc.tick));

		return DDI_INTR_CLAIMED;
	}

	AC_CLR_FLAG(ac,ACF_PENDING_KICK);
	AC_SET_FLAG(ac,ACF_IN_ISR);
	BUMP(ac, irq_handled);

	/* ATAPI IRQ dispatch */
    	if (r->cmd == ATA_CMD_PACKET) 
		atapi_service_irq(ac, r, st); 
	else
		ata_service_irq(ac, r, st);

	AC_CLR_FLAG(ac,ACF_IN_ISR);
	if (AC_HAS_FLAG(ac,ACF_PENDING_KICK)) {
		AC_CLR_FLAG(ac,ACF_PENDING_KICK);
		ide_kick_internal(ac);
	}
	return DDI_INTR_CLAIMED;
}
