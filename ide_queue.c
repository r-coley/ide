/*
 * ide_queue.c 
 */

#include "ide.h"

void 
ide_arm_watchdog(ata_ctrl_t *ac, int ticks)
{
	ATADEBUG(5,"%s: ide_arm_watchdog(%d)\n", Cstr(ac), ticks);

	if (ticks <= 0)
		ticks = (ac->tmo_ticks ? ac->tmo_ticks : 2*HZ);
	if (ac->tmo_id) {
		untimeout(ac->tmo_id);
		ac->tmo_id = 0;
	}
	BUMP(ac,wd_arm);
	ac->tmo_id = timeout(ide_watchdog, (caddr_t)ac, ticks);
}

void 
ide_cancel_watchdog(ata_ctrl_t *ac)
{
	ATADEBUG(5,"%s: ide_cancel_watchdog()\n", Cstr(ac));

	if (ac->tmo_id) {
		BUMP(ac,wd_cancel);
		untimeout(ac->tmo_id);
		ac->tmo_id = 0;
	}
}

void
ide_watchdog(caddr_t arg)
{
	ata_ctrl_t *ac = (ata_ctrl_t *)arg;
	ata_ioque_t *q = ac ? ac->ioque : NULL;
	ata_req_t   *r = q ? q->cur : NULL;
	int	s, er, progress=0;
	u8_t 	ast, err;

	ATADEBUG(3,"ide_watchdog(r=%08x chunk_left=%x sectors_left=%x)\n",
		r, r ? r->chunk_left : -1, r ? r->sectors_left : -1);

	if (q && q->cur == NULL) 
		printf("WARNING: q->cur while command still active\n");

	if (!r) { 
		ac->tmo_id=0; 
		return; 
	}

	BUMP(ac,wd_fired);

	er = ata_err(ac,&ast,&err); 	/*** Check for Error ***/

	if (r->prev_chunk_left != r->chunk_left ||
	    r->prev_sectors_left != r->sectors_left) {
		r->wdog_stuck=0;
		r->prev_chunk_left = r->chunk_left;
		r->prev_sectors_left = r->sectors_left;
		progress=1;
	} else {
		r->wdog_stuck++;
	}

	if (r->wdog_stuck > 50) {
		printf("ide_watchdog: timeout waiting for completion (lba=%ld) ST=%02x ERR=%02x\n", r->lba_cur,ast,err);
		ata_rescueit(ac);
		return;
	}

	s=splbio();
	if (r->chunk_left > 0 && r->sectors_left > 0) {
		BUMP(ac,wd_serviced);
		ide_arm_watchdog(ac,HZ/10);
		splx(s);
		return;	
	}
	if (r->chunk_left == 0 && r->sectors_left > 0) {
		BUMP(ac,wd_rekicked);
		ata_program_next_chunk(ac, r, HZ/8);
		splx(s);
		return;
	}
	if (r->chunk_left == 0 && r->sectors_left == 0) {
		BUMP(ac,eoc_polled);
		ata_finish_current(ac, EOK, 12);
		splx(s);
        	return;
	}
	splx(s);
}

void
ide_q_put(ata_ctrl_t *ac, ata_req_t *r)
{
	ata_ioque_t *q=ac->ioque;
	int	s;

	ATADEBUG(5,"ide_q_put()\n");

	s = splbio();
        r->next = (ata_req_t *)0;
        if (q->q_tail)
                q->q_tail->next = r;
        else
                q->q_head = r;
        q->q_tail = r;
        ac->nreq++;
	splx(s);	
}

ata_req_t *
ide_q_get(ata_ctrl_t *ac)
{
	ata_ioque_t *q=ac->ioque;
	ata_req_t *r;
	int 	s;

	s = splbio();
	r = q ? q->q_head : NULL;
	if (r) {
		q->q_head = r->next;
		if (!q->q_head) q->q_tail = (ata_req_t *)0;
		r->next = (ata_req_t *)0;
		ac->nreq--;
	}
	splx(s);
	ATADEBUG(5,"ide_q_get() returns %lx\n",r);
	return r;
}

void
ide_kick(ata_ctrl_t *ac)
{
	ata_ioque_t *q = ac->ioque;
	int 	s, do_start=0;

	ATADEBUG(5,"ide_kick(flags=%08x)\n",ac->flags);

	s = splbio();
	if (AC_HAS_FLAG(ac, ACF_INTR_MODE)) {
		if (!AC_HAS_FLAG(ac,ACF_BUSY) && !q->cur) do_start=1;
	} else {
		if (!q->cur) do_start=1;
	}
	splx(s);

        if (do_start) ide_start(ac);

	/* Drive synchronously in POLLMODE */
	if (!AC_HAS_FLAG(ac, ACF_INTR_MODE) && 
	     AC_HAS_FLAG(ac, ACF_BUSY))
		ide_poll_engine(ac);
}

void
ide_start(ata_ctrl_t *ac)
{
	ata_ioque_t *q = ac->ioque;
        ata_req_t  *r;
	u8_t	drive, ast;
        int	s;

	ast=inb(ATA_ALTSTATUS_O(ac));
	ATADEBUG(5,"ide_start(#req=%d ST=%02x)\n",ac->nreq,ast);

        s = splbio();
	if (AC_HAS_FLAG(ac,ACF_BUSY) || 
	    AC_HAS_FLAG(ac,ACF_CLOSING) || q->cur) { splx(s); return; }

        /* pop from queue */
	r=ide_q_get(ac);
        if (!r) { 
		splx(s);
		return;
	}

        q->cur   = r;
	q->state = AS_PRIMING;
	r->await_drq_ticks = HZ * 2;
	AC_SET_FLAG(ac,ACF_BUSY); 

	r->lba_cur 	= r->lba;
	r->sectors_left	= r->nsec;
	r->chunk_left	= 0;
	r->xfer_off	= 0;
	
	ATADEBUG(5,"Req=%ld lba=%lu nsec=%lu addr=%lx - lba_cur=%lu secleft=%lu chunkleft=%lu xoff=%lu chunk_bytes=%lu\n",
			r->reqid,
			r->lba,
			r->nsec,
			r->addr,
			r->lba_cur,
			r->sectors_left,
			r->chunk_left,
			r->xfer_off,
			r->chunk_bytes);

	ata_program_next_chunk(ac,r,HZ/8);
	splx(s);
	return;
}

void
ide_need_kick(ata_ctrl_t *ac)
{
	int	s;
	
	ATADEBUG(5,"ide_need_kick()\n");

	s=splbio();
	if (AC_HAS_FLAG(ac, ACF_IN_ISR) || AC_HAS_FLAG(ac,ACF_POLL_RUNNING)) {
		AC_SET_FLAG(ac,ACF_PENDING_KICK);
		splx(s);
		return;
	}
	splx(s);
	ide_kick(ac);
}
