#include "ide.h"

int
ata_sel(ata_ctrl_t *ac, int drive, u32_t lba)
{
	ata_unit_t *u=ac->drive[drive];
	u8_t ast, erc; 

	if (!u) return EIO;
	if (u->lba_ok) {
		u8_t	hi4 = (u8_t)((lba>>24) & 0x0f);

		if (ac->sel_drive != drive || ac->sel_mode != SEL_LBA28 || 
						   ac->sel_hi4 != hi4) {
			ata_err(ac, &ast, &erc);
			ATADEBUG(9,"ata_sel(%d,%lu) ST=%02x ER=%02x: ",
				drive,lba,ast,erc);
			ATADEBUG(1,"LBA %02x\n",ATA_DH(drive,1,hi4));
			outb(ATA_DRVHD_O(ac), ATA_DH(drive,1,hi4));
			ata_delay400(ac);
			ac->sel_drive = drive;
			ac->sel_mode  = SEL_LBA28;
			ac->sel_hi4   = hi4;
		}
	} else {
		if (ac->sel_drive != drive || ac->sel_mode != SEL_CHS || 
						   ac->sel_hi4 != 0) {
			ata_err(ac, &ast, &erc);
			ATADEBUG(9,"ata_sel(%d,%lu) ST=%02x ER=%02x: ",
				drive,lba,ast,erc);
			ATADEBUG(1,"CHS %02x\n",ATA_DH(drive,0,0));
			outb(ATA_DRVHD_O(ac), ATA_DH(drive,0,0));
			ata_delay400(ac);
			ac->sel_drive = drive;
			ac->sel_mode  = SEL_CHS;
			ac->sel_hi4   = 0;
		}
	}
	return 0;
}

void
ata_delay400(ata_ctrl_t *c)
{
	(void)inb(ATA_ALTSTATUS_O(c));	/*Force delay*/
	(void)inb(ATA_ALTSTATUS_O(c));	/*Force delay*/
	(void)inb(ATA_ALTSTATUS_O(c));	/*Force delay*/
	(void)inb(ATA_ALTSTATUS_O(c));	/*Force delay*/
}

/*
 * Drain final command status after last PIO data transfer.
 * Some devices leave DRQ asserted briefly after the final data word.
 * We must not treat the request as complete until we observe !BSY && !DRQ
 * (or an error).
 *
 * Returns 0 on success, -1 on error/timeout.
 */
static int
ata_drain_final_status(ata_ctrl_t *ac)
{
	u8_t st;
	int i;

	/* 400ns settle */
	(void)inb(ATA_ALTSTATUS_O(ac));

	for (i = 0; i < 1000; i++) {
		st = inb(ATA_STATUS_O(ac));

		if (st & (ATA_SR_ERR | ATA_SR_DWF))
			return -1;

		if (!(st & ATA_SR_BSY) && !(st & ATA_SR_DRQ))
			return 0;

		drv_usecwait(1);
	}
	return -1;
}


int
ata_wait(ata_ctrl_t *ac, u8_t must_set, u8_t must_clear, long usec, u8_t *st, u8_t *err)
{
	u8_t s = 0;
	long i;

	if (err)
		*err = 0;

	for (i = 0; i <= usec; ++i) {
		s = inb(ATA_ALTSTATUS_O(ac));
		if (((s & must_set) == must_set) && ((s & must_clear) == 0)) {
			if (st)
				*st = s;
			if ((s & ATA_SR_ERR) && err)
				*err = inb(ATA_ERROR_O(ac));
			return 0;
		}
		drv_usecwait(1);
	}

	if (st)
		*st = s;
	if ((s & ATA_SR_ERR) && err)
		*err = inb(ATA_ERROR_O(ac));
	return -1;
}


int
ata_identify(ata_ctrl_t *ac, int drive)
{
	ata_unit_t *u=ac->drive[drive];
	ata_req_t rb, *r=&rb;
	u16_t 	id[256];
	int 	i;

	ATADEBUG(1,"ata_identify(%s): io=%x\n",Cstr(ac),ac->io_base);

	if (!AC_HAS_FLAG(ac,ACF_PRESENT)) return ENODEV;

	ATA_IRQ_OFF(ac,1);

	bzero((caddr_t)r,sizeof(r));
	r->drive	= drive;
	r->cmd		= (u->devtype & DEV_ATAPI) ? ATA_CMD_IDENTIFY_PKT 
						   : ATA_CMD_IDENTIFY;
	r->lba_cur	= 0;
	r->sectors_left	= 0;
	r->chunk_left	= 0;
	r->is_write	= 0;
	ata_program_taskfile(ac,r);

	if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_BSY, 500000, 0, 0) != 0) {
		/* no ATA device mark not present */
		U_CLR_FLAG(u,UF_PRESENT);
		return ENODEV;
	}

	insw(ATA_DATA_O(ac),id,256);

	/* Extract model string (word-swapped ASCII) */
	strcpy(u->model,getstr(&((char *)id)[54],40,TRUE,TRUE,TRUE));

	/* LBA28 capacity in words 60 61 */
	u->nsectors = ((u32_t)id[61] << 16) | (u32_t)id[60];
	u->lba_ok   = (id[49] & (1<<9)) ? 1 : 0;
	U_SET_FLAG(u,UF_PRESENT);
	u->read_only = 0;
	return 0;
}

int
ata_flush_cache(ata_ctrl_t *ac,u8_t drive)
{
	ata_req_t rb, *r=&rb;

	ATADEBUG(1,"ide_flush_cache(%s)\n",Cstr(ac));

	bzero((caddr_t)r,sizeof(r));
	r->drive	= drive;
	r->is_write	= 0;
	r->cmd		= ATA_CMD_FLUSH_CACHE;
	ata_program_taskfile(ac,r);
	
	if (ata_wait(ac, ATA_SR_DRDY, 
			 ATA_SR_BSY|ATA_SR_DRQ, 2000000, 0, 0) != 0)
		return EIO;
	return 0;
}

void
ata_quiesce_ctrl(ata_ctrl_t *ac)
{
	int	pass;
	u8_t 	ast, st;
	u16_t	words, bc, chunk, scratch[256];

	ATADEBUG(9,"ide_quiesce_ctrl(%s)\n",Cstr(ac));
	ata_wait(ac, 0, ATA_SR_BSY, 500000, 0, 0);

	for(pass=0; pass<2; pass++) {
		ast = inb(ATA_ALTSTATUS_O(ac));
		if (ast & ATA_SR_BSY) break;
		if (!(ast & ATA_SR_DRQ)) break;

		bc = (inb(ATA_CYLHIGH_O(ac)) << 8) |	
		      inb(ATA_CYLLOW_O(ac));
		if (bc == 0) bc=65536;

		words = bc>>1;
		while (words) {
			u16_t chunk = (words > 256) ? 256 : words;
			insw(ATA_DATA_O(ac),scratch,chunk);
			words -= chunk;
		}

		inb(ATA_ALTSTATUS_O(ac));
	}
	ATA_IRQ_OFF(ac,1);
	ata_delay400(ac);
}

void	
ata_dump_stats(void)
{
	int	ctrl, driv;

	for(ctrl=0; ctrl<ATA_MAX_CTRL; ctrl++) {
		ata_ctrl_t *ac= &ata_ctrl[ctrl];
		ata_unit_t *u0 = ac->drive[0];
		ata_unit_t *u1 = ac->drive[1];

		if (!AC_HAS_FLAG(ac,ACF_PRESENT)) continue;
			
		printf("ATA%d[%08x]: intrmode=%d flag=%08x present(%d,%d) turnon=%lu turnoff=%lu\n",
			ctrl, ac, AC_HAS_FLAG(ac,ACF_INTR_MODE),ac->flags,
			u0 ? U_HAS_FLAG(u0,UF_PRESENT) : 0,
			u1 ? U_HAS_FLAG(u1,UF_PRESENT) : 0,
			ac->counters->irq_turnon,
			ac->counters->irq_turnoff);
 		printf("      IRQ: seen=%lu handled=%lu spurious=%lu nocur=%lu bsy=%lu drq_service=%lu\n",
			ac->counters->irq_seen, 
			ac->counters->irq_handled, 
			ac->counters->irq_spurious,
			ac->counters->irq_no_cur,
			ac->counters->irq_bsy_skipped,
			ac->counters->irq_drq_service);
 		printf("      eoc=%lu eoc_polled=%lu lost_irq_rescued=%lu softresets=%lu\n",
			ac->counters->irq_eoc, 
			ac->counters->eoc_polled,
			ac->counters->lost_irq_rescued,
			ac->counters->softresets);
 		printf("      WD: arm=%lu cancel=%lu fired=%lu service=%lu rekicked=%lu chunk=%ld\n",
			ac->counters->wd_arm,
			ac->counters->wd_cancel,
			ac->counters->wd_fired,
			ac->counters->wd_serviced, 
			ac->counters->wd_rekicked,
			ac->counters->wd_chunk);
	}
}

void
ata_softreset_ctrl(ata_ctrl_t *ac)
{
	int	i, was_enabled = AC_HAS_FLAG(ac,ACF_IRQ_ON);

	ATADEBUG(1,"ata_softreset_ctrl(%d)\n",ac->idx);
	BUMP(ac,softresets);
	AC_CLR_FLAG(ac,ACF_IRQ_ON); /* Clear the flag */
	outb(ATA_DEVCTRL_O(ac), ATA_CTL_SRST | ATA_CTL_NIEN);
	for(i=0;i<16;i++) ata_delay400(ac);
	outb(ATA_DEVCTRL_O(ac), ATA_CTL_NIEN); /* deassert SRST */
	(void)inb(ATA_ALTSTATUS_O(ac));

	if (was_enabled) ATA_IRQ_ON(ac);
}

int
ata_enable_pio_multiple(ata_ctrl_t *ac, u8_t drive, u8_t multi)
{
	u8_t	ast, err;

	if (!multi || multi <= 1) return 0;

	ata_sel(ac,drive,0);

	outb(ATA_SECTCNT_O(ac),multi);
	outb(ATA_CMD_O(ac),ATA_CMD_SET_MULTI);
	ata_delay400(ac);

	if (ata_wait(ac, 0, ATA_SR_BSY, 1000000, &ast, &err) != 0) return -1;
	if (ast & (ATA_SR_ERR|ATA_SR_DWF))  return -1;

	return 0;
}

/* Negotiate a valid multi-sector count: try 16/8/4/2 (or 8/4/2) based on policy. */
void
ata_negotiate_pio_multiple(ata_ctrl_t *ac, u8_t drive)
{
    ata_unit_t *u;
    u8_t target = (ATA_USE_MAX_MULTIPLE ? 16 : 8);
    u8_t n;

    /*
     * IMPORTANT:
     *  - POLL mode chunking uses per-unit u->pio_multi (see ata_request()).
     *  - Some other paths consult ac->pio_multi.
     *
     * So, when SET MULTIPLE MODE succeeds we must update BOTH.
     */
    u = (ac && drive < 2) ? ac->drive[drive] : NULL;
    if (target > 16) target = 16;
    if (target < 2)  target = 2;

    for (n = target; n >= 2; n >>= 1) {
	if (ata_enable_pio_multiple(ac,drive,n) == 0) {
            if (u) u->pio_multi = n;
            ac->pio_multi = n;
            return;
        }
    }
    if (u) u->pio_multi = 1;
    ac->pio_multi = 1;
    ATADEBUG(1, "%s: PIO multiple not supported, using single-sector\n",
		Cstr(ac));
}

int
ata_err(ata_ctrl_t *ac, u8_t *ast, u8_t *err)
{
	u8_t	x, y;

	x = inb(ATA_ALTSTATUS_O(ac));
	*ast = x;
	if ((x & (ATA_SR_BSY|ATA_SR_DRQ)) == 0) {
		y = inb(ATA_ERROR_O(ac));
		if (err) *err=y;
		return (u8_t)(x & (ATA_SR_ERR | ATA_SR_DWF)) ? EIO : 0;
	}
	return 0;
}

void
ata_rescue(int ctrl)
{
	ata_ctrl_t *ac= &ata_ctrl[1];
	ata_unit_t *u;

	if (ctrl < 0 || ctrl > 3) {
		printf("Invalid ctrl\n");
		return;
	}
	ac= &ata_ctrl[ctrl];
	if (!AC_HAS_FLAG(ac,ACF_PRESENT)) return;

	u=ac->drive[0];
	U_SET_FLAG(u,UF_ABORT);
	u=ac->drive[1];
	U_SET_FLAG(u,UF_ABORT);


	ata_rescueit(ac);
}

void
ata_rescueit(ata_ctrl_t *ac)
{
	int	s;
	ata_ioque_t *q;

	if (!ac) return;

	s = splbio();
	q = ac->ioque;
	if (!q || !q->cur) { splx(s); return; }

	if (ac->tmo_id) {
		untimeout(ac->tmo_id);
		ac->tmo_id = 0;
	}
	splx(s);

	ata_finish_current(ac,EIO,__LINE__);
	ide_kick(ac);
}

int
pio_one_sector(ata_ctrl_t *ac, ata_req_t *r)
{
	u16_t 	*p16 = (u16_t *)r->xptr;
	int	i;
	
	if (r->is_write) {
		ATADEBUG(2,"pio_one_sector: WRITE lba=%lu addr=%08x\n",
			(u32_t)r->lba_cur, p16);

		for(i=0; i<(ATA_SECSIZE/2); i++)
			outw(ATA_DATA_O(ac), p16[i]);
	} else {
		ATADEBUG(2,"pio_one_sector: READ lba=%lu addr=%08x\n",
			(u32_t)r->lba_cur, p16);

		for(i=0; i<(ATA_SECSIZE/2); i++)
			p16[i] = inw(ATA_DATA_O(ac));
	}
	r->xptr     += ATA_SECSIZE;
	r->xfer_off += ATA_SECSIZE;
	if (r->chunk_left >= 0)   r->chunk_left--;
	if (r->sectors_left >= 0) r->sectors_left--;
/*	ATADEBUG(3,"pio_one_sector done: xfer_off=%08x chunk_left=%d sectors_left=%d\n", r->xfer_off,r->chunk_left,r->sectors_left);*/
	return 0;
}

void
ata_service_irq(ata_ctrl_t *ac, ata_req_t *r, u8_t st)
{
	ata_ioque_t *q = ac ? ac->ioque : 0;
	int	i;
	u8_t ast, err;

	if (!r) {
		BUMP(ac, irq_no_cur);
		return;
	}

	ata_err(ac,&ast,&err);
	/*printf("ata_service_irq: ST=%02x ER=%02x DRQ=%d BSY=%d reqid=%d lba=%ld flags=%x\n",
		ast,err,!!(ast&ATA_SR_DRQ),!!(ast&ATA_SR_BSY),
		r->reqid,r->lba_cur,ac->flags);		*/

	/* ---- (1) Error Handling ---- */
	if (st & (ATA_SR_ERR | ATA_SR_DWF)) {
		u8_t er = inb(ATA_ERROR_O(ac));
		r->ast = st;
		r->err = er;
		ata_finish_current(ac,EIO,__LINE__); 
		ide_kick(ac); /*NEW*/
		return;
	}

	/* ---- (2) Data phase: DRQ asserted transfer one sector ---- */
	if (st & ATA_SR_DRQ) {
		if (ata_data_phase_service(ac,r) < 0) {
			ata_finish_current(ac, EIO, __LINE__);
			ide_kick(ac); /*NEW*/
			return;
		}
		if (r->chunk_left) return;

		if (!r->is_write && (r->flags & ATA_RF_NEEDCOPY) &&
		    q->xfer_buf && r->chunk_bytes) {
			caddr_t dst = (caddr_t)((u8_t *)r->addr + (r->xfer_off - r->chunk_bytes));
			if (valid_usr_range((addr_t)dst, r->chunk_bytes)) {
				bcopy((caddr_t)q->xfer_buf,dst,(size_t)(r->chunk_bytes));
			} else {
				r->err = EFAULT;
			}
			r->flags &= ~ATA_RF_NEEDCOPY;
		}
		
		/* No more sectors? we're done */
		if (r->sectors_left == 0) {
			if (!r->is_write) {
				/* READ completes at last data phase; ensure DRQ/BSY have dropped */
				if (ata_drain_final_status(ac) < 0) {
					ata_finish_current(ac, EIO, __LINE__);
					ide_kick(ac); /*NEW*/
				} else {
					ata_finish_current(ac, EOK, __LINE__);
					ide_kick(ac); /*NEW*/
				}
			}
			/* 
			 * For writes do not finish here; we expect a 
			 * final completion interrupt once the device is
			 * fully idle (BSY=0, DRQ=0). That inteerupt will
			 * be handled in the "final completion" branch below
			 */
			return;
		}

		/* Otherwise: if we're still in the current ATA command (chunk_left > 0),
		 * do NOT issue a new command. Just wait for the next DRQ edge.
		 * When chunk_left reaches 0, the current command should be complete; drain
		 * final status, then either start the next chunk or finish the request.
		 */
		if (r->chunk_left > 0) {
			return;
		}
		{
			u8_t st2 = 0, er2 = 0;
			/* ensure command completion before issuing a new one */
			(void)ata_wait(ac, 0, ATA_SR_BSY|ATA_SR_DRQ, 200000, &st2, &er2);
		}
		/* Start next chunk (new command) for remaining sectors */
		ata_program_next_chunk(ac, r, HZ/8);
		return;
	}	

	/* ---- (3) Final completion drive idle, no error ---- */
	if (!(st & ATA_SR_BSY) &&
	    !(st & ATA_SR_DRQ)) {
		if (r->sectors_left == 0) {
			ATADEBUG(2,"Calling finish as !ATA_SR_BSY && !ATA_SR_DRQ ST=%02x flags=%08x\n",st,ac->flags);
			ata_finish_current(ac, EOK, __LINE__);
			ATADEBUG(2,"Now calling ide_kick flags=%08x\n",
				ac->flags);
			ide_kick(ac); /*NEW*/
			return;
		}
	}

	ide_arm_watchdog(ac,HZ/8);
}

void
ata_program_taskfile(ata_ctrl_t *ac, ata_req_t *r)
{
	u32_t lba    = r->lba_cur;
	u8_t  drive  = (r->drive & 0x1);
	u8_t  cmd    = r->cmd;
	u16_t todo   = (r->nsec == 0) ? 256 : ((r->nsec>256) ? 256 : r->nsec);
	u8_t  sc     = (todo == 256) ? 0 : todo;
	u8_t 	ast, err, dh;
	int	er;

	ata_wait(ac,0,ATA_SR_BSY,500000,0,0);
	ata_sel(ac, drive, lba);
	er=ata_err(ac,&ast,&err);
	r->flags &= ~ATA_RF_CDB_SENT;

	/* Cache last command */
	ac->lc.cmd   = cmd;
	ac->lc.sc    = sc;
	ac->lc.dh    = drive;
	ac->lc.lba   = lba;
	ac->lc.err   = err;
	ac->lc.reqid = r->reqid;
	ac->lc.tick  = lbolt;

	switch (cmd) {
	case ATA_CMD_READ_SEC:
	case ATA_CMD_READ_SEC_EXT:
	case ATA_CMD_READ_MULTI:
		outb(ATA_SECTCNT_O(ac), sc);
		outb(ATA_LBA0_O(ac),    (u8_t)(lba      ));
		outb(ATA_LBA1_O(ac),    (u8_t)(lba >>  8));
		outb(ATA_LBA2_O(ac),    (u8_t)(lba >> 16));
		outb(ATA_CMD_O(ac), cmd);
		break;

	case ATA_CMD_WRITE_SEC:
	case ATA_CMD_WRITE_SEC_EXT:
	case ATA_CMD_WRITE_MULTI:
		outb(ATA_SECTCNT_O(ac), sc);
		outb(ATA_LBA0_O(ac),    (u8_t)(lba      ));
		outb(ATA_LBA1_O(ac),    (u8_t)(lba >>  8));
		outb(ATA_LBA2_O(ac),    (u8_t)(lba >> 16));
		outb(ATA_CMD_O(ac), cmd);
		break;

	case ATA_CMD_IDENTIFY:
	case ATA_CMD_IDENTIFY_PKT:
		outb(ATA_SECTCNT_O(ac), 0);
		outb(ATA_LBA0_O(ac),    0);
		outb(ATA_LBA1_O(ac),    0);
		outb(ATA_LBA2_O(ac),    0);
		outb(ATA_CMD_O(ac), cmd);
		break;

	case ATA_CMD_PACKET:
		outb(ATA_FEAT_O(ac),    0x00);
		outb(ATA_SECTCNT_O(ac), 0x00);
		outb(ATA_LBA0_O(ac),    0x00);
		outb(ATA_LBA1_O(ac),    (u8_t)(r->atapi_bytes      & 0xFF));
		outb(ATA_LBA2_O(ac),    (u8_t)((r->atapi_bytes>>8) & 0xFF));
		outb(ATA_CMD_O(ac), cmd);
		break;

	case ATA_CMD_FLUSH_CACHE:
		outb(ATA_CMD_O(ac), cmd);
		break;

	default:
		outb(ATA_SECTCNT_O(ac), 0);
		outb(ATA_LBA0_O(ac),    0);
		outb(ATA_LBA1_O(ac),    0);
		outb(ATA_LBA2_O(ac),    0);
		outb(ATA_CMD_O(ac), cmd);
		break;
	}

	ata_delay400(ac); 
	if (AC_HAS_FLAG(ac,ACF_INTR_MODE)) {
		drv_usecwait(20);
		if (cmd == ATA_CMD_WRITE_SEC ||
		    cmd == ATA_CMD_WRITE_SEC_EXT ||
		    cmd == ATA_CMD_WRITE_MULTI) {
			if (ata_wait(ac,ATA_SR_DRQ,ATA_SR_BSY,200000,0,0) == 0)
				ata_prime_write(ac,r);
			else
				ATADEBUG(2,"%s: DRQ not ready for write ST=%02x\n",
					Cstr(ac),inb(ATA_ALTSTATUS_O(ac)));
		}
	}
}

int 
ata_program_next_chunk(ata_ctrl_t *ac, ata_req_t *r,int arm_ticks)
{
	ATADEBUG(5,"ata_program_next_chunk(%s)\n",Cstr(ac));
	if (r->cmd == ATA_CMD_PACKET)
		return atapi_request(ac,r,arm_ticks);
	else
		return ata_request(ac,r,arm_ticks);
}

int 
ata_request(ata_ctrl_t *ac,ata_req_t *r,int arm_ticks)
{
	u32_t 	n;
	size_t 	bytes;
	ata_ioque_t *q = ac->ioque;
	ata_unit_t  *u = ac->drive[r->drive];
	u8_t	ast;
	int	s, er, multi_ok;
	caddr_t	user_ptr;

	ATADEBUG(2,"ata_request(Reqid=%ld)\n",r ? r->reqid : 0);
	if (!r) return;

	if (AC_HAS_FLAG(ac,ACF_INTR_MODE)) {
		n = (r->sectors_left > 256U) ? 256U : r->sectors_left;
		if (n == 0) return;

		/* Cap sectors to bounce-buffer capacity in interrupt mode */
		if (q->xfer_buf) {
			u32_t maxsecs = (q->xfer_bufsz >> 9);
			if ((u32_t)n > maxsecs) n = (int)maxsecs;
		}
	} else {
		n = r->sectors_left;
		if (n > (u32_t)u->pio_multi) n = (u32_t)u->pio_multi;
		if (n == 0) return;
		/* POLL mode: allow multi-sector PIO up to u->pio_multi */
	}

	bytes = (size_t)n << 9; /* * 512U */

	r->lba_cur 	= r->lba + (r->xfer_off >> 9);
	r->nsec 	= (u16_t)n;
	r->chunk_left   = (u16_t)n;
	r->chunk_bytes  = (u32_t)bytes;
	r->cmd          = multicmd(ac, r->is_write,r->lba_cur,n);
	r->flags       &= ~ATA_RF_NEEDCOPY;

	if (r->is_write) {
		/* Write: prefer bounce buffer for IRQ path; copy from user if valid */
		if (q->xfer_buf && valid_usr_range((addr_t)r->addr, bytes)) {
			bcopy((caddr_t)r->addr + r->xfer_off, q->xfer_buf, bytes);
			r->xptr = q->xfer_buf;
		} else {
			/* kernel buffers only */
			r->xptr = (caddr_t)r->addr + r->xfer_off;
		}
	} else {
		/* Read: receive into bounce buffer when available; copy back later if user VA */
		if (q->xfer_buf) {
			r->xptr = q->xfer_buf;
			if (valid_usr_range((addr_t)r->addr, bytes))
				r->flags |= ATA_RF_NEEDCOPY;
		} else {
			r->xptr = (caddr_t)r->addr + r->xfer_off;
		}
	}

	ATADEBUG(5,"%s: ata_program_next_chunk(%s) blk=%lu count=%lu\n",
		Cstr(ac),r->is_write?"Write":"Read",r->lba_cur,n);

	s=splbio();
	q->state = AS_XFER;
	AC_SET_FLAG(ac,ACF_BUSY);
	q->cur  = r;
	splx(s);

	ata_program_taskfile(ac, r);

	/* reset DRQ wait budget for this chunk */
	r->await_drq_ticks = HZ * 2;

	if (arm_ticks) ide_arm_watchdog(ac,arm_ticks);

	if (!AC_HAS_FLAG(ac,ACF_INTR_MODE)) ide_kick(ac);
}

void 
ata_finish_current(ata_ctrl_t *ac, int err,int place)
{
	ata_ioque_t *que;
	ata_req_t  *r;
	buf_t      *bp = NULL;
	int 	s;
	size_t bytes_done;
	u32_t 	resid;

	ATADEBUG(2,"ata_finish_current(err=%d place=%d)\n",err,place);
	if (!ac) {
		ATADEBUG(2,"ata_finish_current() ac NULL\n");
		return;
	}
	ide_cancel_watchdog(ac);

	que = ac->ioque;
	if (!que) {
		ATADEBUG(2,"ata_finish_current() que NULL\n");
	}
s = splbio();
r  = que ? que->cur : NULL;
if (!r) {
    ATADEBUG(9,"ata_finish(reqid: None)\n");
    if (que) {
        que->last_err = err;
        que->state = AS_IDLE;
        AC_CLR_FLAG(ac, ACF_BUSY);
        que->cur = NULL;
    }
    splx(s);
    return;
}
ATADEBUG(9,"ata_finish(reqid: %lu)\n",r->reqid);
	r->err = err;
	if (r->flags & ATA_RF_DONE) { splx(s); return; }
	r->flags |= ATA_RF_DONE;
	splx(s);

	bp = r->bp;
	bytes_done = r->xfer_off;

	if (bp) {
		if (bytes_done > bp->b_bcount) bytes_done = bp->b_bcount;
		resid = bp->b_bcount - bytes_done;
	} else {
		bytes_done=0;
		resid=0;
	}

	/*** Xfer done - now copy the buffer if needed ***/
	if (!r->err && 
	    !r->is_write && 
	    (r->flags & ATA_RF_NEEDCOPY) && 
	    que->xfer_buf && r->chunk_bytes) { 
		caddr_t dst = (caddr_t)((char *)r->addr + (r->xfer_off - r->chunk_bytes));
		if (valid_usr_range((addr_t)dst, r->chunk_bytes))
			bcopy(que->xfer_buf, dst, r->chunk_bytes);
		else
			r->err = EFAULT;

		r->flags &= ~ATA_RF_NEEDCOPY;
	}

	s=splbio();
AC_CLR_FLAG(ac,ACF_BUSY);
que->cur = NULL;
que->state = AS_IDLE;
que->last_err = r->err;
splx(s);
if (bp) {
		if (err) berror(bp,resid,EIO);
		else     bok(bp,resid);
	}

	if (r) kmem_free(r,sizeof(*r));
	AC_SET_FLAG(ac,ACF_PENDING_KICK);
}

int
ata_data_phase_service(ata_ctrl_t *ac, ata_req_t *r)
{
	u8_t ast;
	int rc = 0;

	/* point xptr at the current transfer offset */
	r->xptr = r->addr + r->xfer_off;

	/* Wait briefly for BSY to clear and DRQ to assert */
	if (ata_wait(ac, ATA_SR_DRQ|ATA_SR_DRDY, ATA_SR_BSY, 10000, &ast, 0)) {
		ATADEBUG(2,"ata_data_phase: DRQ wait timeout %02x\n",ast);
		r->err = ast;
		return -1;
	}

	/* small settle delay */
	drv_usecwait(10);

	/* Consume exactly ONE 512B data phase per call in POLL mode.
	 * Multi-sector PIO commands re-assert DRQ per sector; we must not
	 * assume we can read multiple sectors back-to-back without waiting
	 * for the next DRQ edge.
	 */
	if (pio_one_sector(ac, r) != 0) {
		ATADEBUG(1,"ata_data_phase: pio_one_sector failed\n");
		rc = -1;
	}

	/* derive from bytes already transferred to avoid drift */
	r->lba_cur = r->lba + (r->xfer_off >> 9);

	return rc;
}


void
ata_prime_write(ata_ctrl_t *ac, ata_req_t *r)
{
	ata_ioque_t *q = ac->ioque;
	u8_t	ast, err;
	u8_t	drive = r->drive & 1;
	int	er;

	if (ata_wait(ac,ATA_SR_DRQ,ATA_SR_BSY,1000000,&ast,&err) != 0) {
		if (!(ast & ATA_SR_DRQ)) {
			ATADEBUG(2,"ata_prime_write(): wait DRQ fail ST=%02x ER=%02x\n",ast,err);
			return;
		}
	}

	if (!q) printf("que is null\n");
	if (!r->xptr) printf("r->xptr is null\n");
	
	if (pio_one_sector(ac,r) != 0) {
		/* Error */
	}
	return;
}

int
ata_pushreq(ata_ctrl_t *ac, ata_req_t *r)
{
    ata_ioque_t *que = ac ? ac->ioque : NULL;
    struct buf  *bp  = r ? r->bp : NULL;
    int s;

    ATADEBUG(1, "ata_pushreq(%s: r->id=%ld flags=%08x)\n",
        Cstr(ac), r ? r->reqid : 0L, ac ? ac->flags : 0);

    if (!ac || !que || !r || !bp) {
        /* Internal callers should always provide a buf-backed request. */
        return EIO;
    }

    /*
     * Queue and kick under splbio(), then wait using the kernel buf mechanism.
     * This matches the vanilla hd driver pattern (strategy + iowait/biodone),
     * and avoids controller-global SYNC_DONE races.
     */
    s = splbio();
    ide_q_put(ac, r);

    if (AC_HAS_FLAG(ac, ACF_INTR_MODE) || !AC_HAS_FLAG(ac, ACF_POLL_RUNNING))
        ide_kick(ac);

    splx(s);

    iowait(bp);
    return (bp->b_flags & B_ERROR) ? bp->b_error : 0;
}

int
multicmd(ata_ctrl_t *ac, int is_write, u32_t lba, u32_t nsec)
{
	int	use_ext = 0; /* (lba > 0xffffffff) && ac->lba48_ok;*/
	int	multi_ok = (nsec>1) && (ac->pio_multi>1) && ac->multi_set_ok;

	if (is_write) {
		if (use_ext) return multi_ok ? ATA_CMD_WRITE_MULTI_EXT
					     : ATA_CMD_WRITE_SEC_EXT;
		return multi_ok ? ATA_CMD_WRITE_MULTI
				: ATA_CMD_WRITE_SEC;
	} else {
		if (use_ext) return multi_ok ? ATA_CMD_READ_MULTI_EXT
					     : ATA_CMD_READ_SEC_EXT;
		return multi_ok ? ATA_CMD_READ_MULTI
				: ATA_CMD_READ_SEC;
	}
}
