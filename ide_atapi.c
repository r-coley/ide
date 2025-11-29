#include "ide.h"

/*
 * Decode and print ATAPI REQUEST SENSE buffer.
 * This is primarily for debugging/logging; it does not change behavior.
 */
void
atapi_decode_sense(u8_t *s, int len)
{
	int sk, asc, ascq;
	u32_t info;

	if (s == 0 || len < 14)
		return;

	/* Fixed format sense header is 18 bytes; we only rely on first 14. */
	if (s[0] < 0x70 || s[0] > 0x73) {
		if (s[0] != 0xf0)
			printf("ATAPI: sense: unknown format 0x%x\n", s[0]);
		return;
	}

	sk   = s[2] & 0x0F;          /* Sense Key */
	asc  = s[12];
	ascq = s[13];

	info = ((u32_t)s[3] << 24) |
	       ((u32_t)s[4] << 16) |
	       ((u32_t)s[5] <<  8) |
	       ((u32_t)s[6] <<  0);

	/* Basic sense key text */
	printf("ATAPI: SK=%x, ASC=%02x, ASCQ=%02x, INFO=%08lx",
	       sk, asc, ascq, (unsigned long)info);

	switch (sk) {
	case 0x0: /* NO SENSE */
		printf(" (No Sense)");
		break;
	case 0x1:
		printf(" (Recovered Error)");
		break;
	case 0x2:
		printf(" (Not Ready)");
		break;
	case 0x3:
		printf(" (Medium Error)");
		break;
	case 0x4:
		printf(" (Hardware Error)");
		break;
	case 0x5:
		printf(" (Illegal Request)");
		break;
	case 0x6:
		printf(" (Unit Attention)");
		break;
	case 0x7:
		printf(" (Data Protect)");
		break;
	case 0x8:
		printf(" (Blank Check)");
		break;
	case 0xB:
		printf(" (Aborted Command)");
		break;
	default:
		break;
	}

	/* A few common ASC/ASCQ combinations that are useful to see. */
	if (sk == 0x2 && asc == 0x04) {
		printf(" - Logical unit not ready, in progress of becoming ready");
	} else
	if (sk == 0x2 && asc == 0x3A) {
		printf(" - Medium not present");
	} else
	if (sk == 0x6) {
		printf(" - Unit attention (media changed?)");
	}

	printf("\n");
}

void
atapi_dosend_packet(ata_ctrl_t *ac, int drive, u16_t byte_count)
{
	ATADEBUG(1,"atapi_dosend_packet(%s, drive=%d, bc=%d)\n",
		Cstr(ac),drive,byte_count);

	/* Select device + 400ns settle */
	ata_sel(ac,drive,0);

	/* FEAT=0, SECCNT=0, Byte Count in CYCLOW/HIGH */
	outb(ATA_FEAT_O(ac),    0x00);
	outb(ATA_SECTCNT_O(ac), 0x00);
	outb(ATA_CYLLOW_O(ac),  (u8_t)(byte_count & 0xFF));
	outb(ATA_CYLHIGH_O(ac), (u8_t)(byte_count >> 8));

	outb(ATA_CMD_O(ac), ATA_CMD_PACKET);
	ata_delay400(ac); ata_delay400(ac);
}

int
atapi_read_capacity(ata_ctrl_t *ac, u8_t drive, u32_t *out_blocks, u32_t *out_blksz)
{
	ata_unit_t *u = ac->drive[drive];
	u16_t 	xfer_len = 8;   /* READ CAPACITY(10) returns 8 bytes */
	u8_t 	buf[16], ast, err;
	int 	i, er, cdblen;
	u32_t	last_lba, blksz, blocks;

	/* 10-byte CDB inside 12-byte packet: opcode only, rest zero */
	cdblen=build_cdb_pkt(CDB_READ_CAPACITY, (u8_t *)u->cdb, 0, 0);

	if (ata_sel(ac,drive,0) != 0) return EIO;

	/* Program expected byte count */
	atapi_dosend_packet(ac, drive, (u16_t)xfer_len);

	if (ata_wait(ac,ATA_SR_DRQ,ATA_SR_BSY,200000L,0,0) != 0) return EIO;

	atapi_send_cdb(ac,u->cdb,cdblen,__LINE__);

	if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_ERR, 500000L, 0, 0) != 0) 	
		return EIO;

	{
		u16_t tmp[4];
		insw(ATA_DATA_O(ac),tmp,4);
		for(i=0;i<4;i++) {
			buf[(i<<1)+0] = CDB16_L(tmp[i]);
			buf[(i<<1)+1] = CDB16_H(tmp[i]);
		}
	}
 	(void)ata_wait(ac, 0, ATA_SR_BSY | ATA_SR_DRQ, 500000L, &ast, 0);

	/* Parse: [last LBA][block length]; both big-endian */
	last_lba = ((u32_t)buf[0] << 24) | ((u32_t)buf[1] << 16) |
		   ((u32_t)buf[2] << 8)  | (u32_t)buf[3];
	blksz    = ((u32_t)buf[4] << 24) | ((u32_t)buf[5] << 16) |
		   ((u32_t)buf[6] << 8)  | (u32_t)buf[7];
	blocks   = last_lba + 1U;

	if (out_blocks) *out_blocks = blocks;
	if (out_blksz)  *out_blksz  = blksz;

	/* Persist for later use (optional fields in ata_unit[]) */
	u->atapi_blocks = blocks;
	u->atapi_blksz  = blksz;
	return 0;
}

/* --- ATAPI: issue 12-byte SCSI INQUIRY and get the device class --- */
int
atapi_inquiry(ata_ctrl_t *ac, u8_t drive)
{
	ata_unit_t *u=ac->drive[drive];
	int 	i, er, cdblen;
	u16_t 	avail, words, xfer_len = 36; 
	u8_t 	buf[64], pdt, rmb, ast, err;

	cdblen=build_cdb_pkt(CDB_INQUIRY, (u8_t *)u->cdb, (u8_t)xfer_len, 0);

	if (ata_sel(ac,drive,0) != 0) return EIO;

	/* Program expected transfer size into LBA1/LBA2 (ATAPI byte-count) */
	atapi_dosend_packet(ac, drive, (u16_t)xfer_len);

	/* Wait for DRQ to send the packet */
	if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_BSY, 200000L, 0, 0) != 0) 
		return EIO;

	atapi_send_cdb(ac,u->cdb,cdblen,__LINE__);

	/* Now the device will transfer data; poll for DRQ then read */
	if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_ERR, 500000L, 0, 0) != 0) 
		return EIO;

	/* The device sets the actual byte count to read in LBA1/LBA2 */
	avail = ((u16_t)inb(ATA_CYLHIGH_O(ac)) << 8) |
		 (u16_t)inb(ATA_CYLLOW_O(ac));
	if (avail == 0 || avail > xfer_len) avail = xfer_len;

	words = (int)(avail + 1) >> 1;
	for (i = 0; i < (int)words; i++) {
		u16_t w = inw(ATA_DATA_O(ac));
		if ((i<<1) < sizeof(buf)) {
			buf[(i<<1)+0] = CDB16_L(w);
			buf[(i<<1)+1] = CDB16_H(w);
		}
	}

	/* Final status: wait for DRQ to clear, BSY=0, then read STATUS once */
	(void)ata_wait(ac, 0, ATA_SR_BSY | ATA_SR_DRQ, 500000L, &ast, 0); 

	/* Parse PDT and strings */
	pdt = buf[0] & 0x1F;
	rmb = (buf[1] >> 7) & 1;

	u->pdt = pdt;
	u->rmb = rmb;
	if (pdt == 0x05) U_SET_FLAG(u,UF_CDROM);
	if (pdt == 0x00 && rmb) U_SET_FLAG(u,UF_MOZIP);
	if (rmb) U_SET_FLAG(u,UF_REMOVABLE);

	strcpy(u->vendor,getstr((char *)&buf[8],8,FALSE,TRUE,FALSE));
	strcpy(u->product,getstr((char *)&buf[16],16,FALSE,TRUE,FALSE));

	return 0;
}

int
atapi_read10(ata_ctrl_t *ac,u8_t drive,u32_t lba,u16_t nblks,void *buf)
{ 
	ata_unit_t *u=ac->drive[drive];
	int	cdblen;
	u32_t	xfer;

	/* total transfer size in bytes */
	xfer = (u32_t)nblks * 
		(ac->drive[drive]->atapi_blksz ? ac->drive[drive]->atapi_blksz 
					       : 2048);

	cdblen=build_cdb_pkt(CDB_READ_10, (u8_t *)u->cdb, lba, nblks);

	ATADEBUG(1,"READ10: drive=%d LBA=%lu nblks=%u xfer=%lu blksz=%u\n",
		 drive,lba,nblks,xfer,
		 ac->drive[drive]->atapi_blksz);

	/* send the command */
	return atapi_packet(ac,drive,(u8_t *)u->cdb,cdblen,buf,xfer,1,(u8_t *)u->sense,sizeof(u->sense));
}

int
atapi_mode_sense10(ata_ctrl_t *ac,u8_t drive,u8_t page,u8_t subpage,void *buf,u16_t len)
{
	ata_unit_t *u=ac->drive[drive];
	int	cdblen;

	cdblen=build_cdb_pkt(CDB_MODE_SENSE_10, (u8_t *)u->cdb, (page<<8)|subpage, len);
	return atapi_packet(ac,drive,(u8_t *)u->cdb,12,buf,len,1,
					(u8_t *)u->sense,sizeof(u->sense));
}

int
atapi_mode_sense6(ata_ctrl_t *ac,u8_t drive,u8_t page,u8_t subpage,void *buf,u8_t len)
{
	ata_unit_t *u=ac->drive[drive];
	int	cdblen;

	cdblen=build_cdb_pkt(CDB_MODE_SENSE_6, (u8_t *)u->cdb, (page<<8)|subpage, len);
	return atapi_packet(ac,drive,(u8_t *)u->cdb,cdblen,buf,len,1,(u8_t *)u->sense,sizeof(u->sense));
}

/* Issue an ATAPI command and transfer data (polled PIO).
 * dir: 1=data-in, 0=data-out, <0=no data
 * Returns 0 on success; EIO/ETIMEDOUT on failure. If sense!=NULL and an error
 * occurs, a REQUEST SENSE(6) of up to sense_len bytes is attempted.
 */
int
atapi_packet(ata_ctrl_t *ac, u8_t drive, u8_t *cdb, int cdb_len, void *buf, u32_t xfer_len, int dir, u8_t *sense, int sense_len)
{
	ata_unit_t *u = ac->drive[drive];
	u8_t  st, err, ir, errreg;
	u16_t bc, wcount, w;
	u32_t to_xfer;
	int   rc, spins;
	int   retries = 0;

	ATADEBUG(1,"atapi_packet(%s)\n",Cstr(ac));
retry_cmd:
	U_CLR_FLAG(u,UF_ABORT);

	/* Step 1: send PACKET + CDB */
	rc = atapi_send_packet(ac, drive, cdb, cdb_len);
	if (rc != 0) {
		goto sense_or_fail;
	}

	/*
	 * some ATAPI commands (PLAY AUDIO, PAUSE/RESUME etc) have no data
	 * phase. Many drives and 86Box in particular will only interrupt
	 * once the PACKET is accepted and never again when playback
	 * actually begins. If we drove these through the normal DRQ/data
	 * loop, ata_packet() would sit forever in ata_wait()
	 *
	 * For such "no data" commands we simply:
	 * - wait for BSY+DRQ to clear
	 * - check for ERR,
	 * - and consider the command complete
	 */
	if (xfer_len == 0 && dir <= 0 &&
	    (cdb[0] == CDB_PLAY_AUDIO_MSF ||
	     cdb[0] == CDB_PLAY_AUDIO_10 ||
	     cdb[0] == CDB_PAUSE_RESUME)) {
		/* Poll until the drive is no longer busy and has no DRQ */
		rc = ata_wait(ac,0,ATA_SR_BSY|ATA_SR_DRQ,1000000,&st,0);
		if (rc != 0 || (st & ATA_SR_ERR)) {
			/* Treat as error */
			if (rc == 0) rc=EIO;
			goto sense_or_fail;
		}
		/* Command accepted; playback is now in progress or paused */
		return 0;
	}

/* Step 2: data/status loop (polled) */
	for (; !U_HAS_FLAG(u,UF_ABORT);) {
		/* Wait for BSY to clear */
		ata_wait(ac,0,ATA_SR_BSY,1000000L,&st,0);

		/* If DRQ dropped, command is finishing; read final STATUS */
		if ((st & ATA_SR_DRQ) == 0) {
			if (st & (ATA_SR_ERR | ATA_SR_DWF)) {
				errreg = inb(ATA_FEAT_O(ac));
				(void)errreg;
				rc = EIO;
				goto sense_or_fail;
			}
			/* No DRQ and no error means the device is done sending
			 * data. Even if we request more (xfer_len>0). Many 
			 * ATAPI devices (and 86Box) return less data than the
			 * allocation length - Treat this as end of data
			 */
			xfer_len=0;
			break;
#if 0
			if (xfer_len == 0) break; /* success */
			continue;
#endif
		}

		/* DRQ asserted: read IReason + ByteCount */
		ir  = inb(ATA_SECTCNT_O(ac)) & 0x03;
		bc  = inb(ATA_CYLLOW_O(ac));
		bc |= ((u16_t)inb(ATA_CYLHIGH_O(ac))) << 8;
		if (bc == 0) {
			bc = u->atapi_blksz ? u->atapi_blksz : 2048;
			if (bc == 0) bc=2048;
		}

		switch(ir) {
		case 0x00:
		case 0x02:
			to_xfer = (xfer_len < (u32_t)bc) ? xfer_len:(u32_t)bc;
			wcount = (u16_t)((to_xfer+1) >> 1);

			if (ir == 0x02)
				insw(ATA_DATA_O(ac),buf,wcount);
			else
				outsw(ATA_DATA_O(ac),buf,wcount);
		
			if ((u32_t)bc > to_xfer) {
				u16_t extra = (u16_t)(((u32_t)bc-to_xfer+1)>>1);
				while (extra--) {
					if (ir == 0x02)
						(void)inw(ATA_DATA_O(ac));
					else
						(void)outw(ATA_DATA_O(ac),0);
				}
			}
			buf = (void *)((u8_t *)buf + to_xfer);
			xfer_len = (xfer_len >= to_xfer)?(xfer_len-to_xfer):0;

			inb(ATA_ALTSTATUS_O(ac));
			break;

		case 0x01:
		default:
			printf("Uknown Phase\n");
			break;
		}
	}
	if (ata_wait(ac,0,ATA_SR_BSY,100000L,&st,&err) != 0) {
		rc=EIO;
		goto sense_or_fail;
	}
	if (st & (ATA_SR_ERR | ATA_SR_DWF)) {
		rc=EIO;
		goto sense_or_fail;
	}
	return 0;

sense_or_fail:
	/* Try REQUEST SENSE(6) to report SK/ASC/ASCQ if buffer was provided */
	if (sense && sense_len >= 18) {
		u8_t  rs_cdb[12];
		int  i;

		for (i = 0; i < 12; i++) rs_cdb[i] = 0;
		rs_cdb[0] = CDB_REQUEST_SENSE; /* REQUEST SENSE (6) */
		rs_cdb[4] = 18;            /* alloc length */

		/* Clear pending status then issue sense; ignore its return */
		(void)inb(ATA_ALTSTATUS_O(ac));
		(void)atapi_send_packet(ac,drive, rs_cdb, 12);

		if (ata_wait(ac, ATA_SR_DRQ, ATA_SR_BSY, 100000, &st, 0) == 0) {
			u16_t want = (u16_t)((sense_len < 18) ? sense_len : 18);
			u16_t wds  = (u16_t)((int)(want+1) >> 1);
			for(w=0; w < wds; w++)
				*(((u16_t *)sense) + w) = inw(ATA_DATA_O(ac));
			(void)inb(ATA_ALTSTATUS_O(ac));
			/* Decode and log sense data for debugging */
			atapi_decode_sense(sense, sense_len);

			/* Basic retry logic for transient conditions. */
			if (retries < ATAPI_MAX_RETRIES) {
				u8_t sk  = sense[2] & 0x0f;
				u8_t asc = sense[12];
				u8_t ascq= sense[13];

				/* Unit Attention or Becoming Ready: retry after TUR. */
				if (sk == 0x06 || (sk == 0x02 && asc == 0x04)) {
					retries++;
					if (sk == 0x02 && asc == 0x04) {
						/* NOT READY / BECOMING READY: give drive a moment. */
						drv_usecwait(50000);
					}
					(void)atapi_test_unit_ready(ac, drive);
					U_CLR_FLAG(u, UF_ABORT);
					goto retry_cmd;
				}
			}
		}
	}
	return (rc != 0) ? rc : EIO;
}

int
atapi_test_unit_ready(ata_ctrl_t *ac, u8_t drive)
{
	ata_unit_t *u = ac->drive[drive];
	int	cdblen, rc;

	cdblen=build_cdb_pkt(CDB_TEST_UNIT_READY, (u8_t *)u->cdb, 0, 0);
	rc = atapi_packet(ac,drive,(u8_t *)u->cdb,sizeof(u->cdb),NULL,0,1,(u8_t *)u->sense,sizeof(u->sense));
	if (rc == 0) {
		U_SET_FLAG(u,UF_HASMEDIA);
		return 0;
	}

	if ((u->sense[2] & 0x0f) == 0x02 && u->sense[12] == 0x3a) {
		U_CLR_FLAG(u,UF_HASMEDIA);
		return 0;
	}
	return 0;
}

char *
atapi_class_name(ata_unit_t *u)
{
	switch (u->pdt) {
	case 0x00: return u->rmb ? "Removable Disk (ZIP/MO)":"Direct-Access Disk";
	case 0x05: return "CD/DVD"; 
	case 0x01: return "Tape";
	case 0x04: return "WORM";
	case 0x07: return "Optical Memory";
	case 0x0E: return "Simplified Direct-Access";
	default:   return "Unknown ATAPI";
	}
}

int
atapi_write10(ata_ctrl_t *ac,u8_t drive,u32_t lba,u16_t nblks,const void *buf)
{
	ata_unit_t *u = ac->drive[drive];
	u32_t	xfer;
	int	cdblen;

	cdblen=build_cdb_pkt(CDB_WRITE_10, (u8_t *)u->cdb, lba, nblks);
	xfer = (u32_t)nblks * (ac->drive[drive]->atapi_blksz 
				? ac->drive[drive]->atapi_blksz : 2048);

	return atapi_packet(ac,drive,(u8_t *)u->cdb,cdblen,(void *)buf,xfer,0,(u8_t *)u->sense,sizeof(u->sense));
}


int
atapi_read_toc(ata_ctrl_t *ac, u8_t drive, int msf, u8_t format, u8_t track_session, void *buf, u16_t len)
{
	ata_unit_t *u = ac->drive[drive];
	u8_t cdb[12];

	if (len > 4096) len=4096;

	bzero((caddr_t)cdb, sizeof(cdb));
	cdb[0] = CDB_READ_TOC;
	cdb[1] = (msf) ? 0x02 : 0x00; /* MSF bit */
	cdb[2] = format;              /* reserved / format-dependent */
	cdb[6] = track_session;  /* starting track or session */
	cdb[7] = (u8_t)(len >> 8);
	cdb[8] = (u8_t)(len & 0xFF);
	cdb[9] = format & 0x0F;  /* format in low 4 bits */

	return atapi_packet(ac, drive, cdb, sizeof(cdb), buf, (u32_t)len, 1,
			 (u8_t *)u->sense, sizeof(u->sense));
}

int
atapi_play_audio_msf(ata_ctrl_t *ac, u8_t drive,
		       u8_t start_m, u8_t start_s, u8_t start_f,
		       u8_t end_m,   u8_t end_s,   u8_t end_f)
{
	ata_unit_t *u = ac->drive[drive];
	u8_t cdb[12];

	bzero((caddr_t)cdb, sizeof(cdb));
	cdb[0] = CDB_PLAY_AUDIO_MSF;
	cdb[3] = start_m;
	cdb[4] = start_s;
	cdb[5] = start_f;
	cdb[6] = end_m;
	cdb[7] = end_s;
	cdb[8] = end_f;

	return atapi_packet(ac, drive, cdb, sizeof(cdb), NULL, 0, -1,
			 (u8_t *)u->sense, sizeof(u->sense));
}

void
atapi_send_cdb(ata_ctrl_t *ac, u8_t *cdb, int cdb_len, int line)
{

	if (cdb_len < 0) cdb_len=0;
	if (!ATAPI_VALID_CDB(cdb_len)) {
		printf("Invalid CDBLEN\n");
		return;
	}

	outsw(ATA_DATA_O(ac),cdb,(cdb_len+1)>>1);
	drv_usecwait(40);
}

int
atapi_start_irq(ata_ctrl_t *ac, ata_req_t *r)
{
	ata_unit_t *u = ac->drive[r->drive];
	u32_t	blksz = (u && u->atapi_blksz) ? u->atapi_blksz : 2048;
	u16_t	bc    = (u16_t)blksz;

	ATADEBUG(1,"atapi_start_irq()\n");

	/* Select drive and program the PACKET command with transfer length. */
	ata_sel(ac, r->drive, 0);
	outb(ATA_FEAT_O(ac),    0x00);
	outb(ATA_SECTCNT_O(ac), 0x00);
	outb(ATA_SECTNUM_O(ac), 0x00);
	outb(ATA_CYLLOW_O(ac),  (u8_t)(bc & 0xFF));
	outb(ATA_CYLHIGH_O(ac), (u8_t)(bc >> 8));

	outb(ATA_CMD_O(ac), ATA_CMD_PACKET);
	ata_delay400(ac);

	/* Let interrupts handle the DRQ/CDB/data phases. */
	ata_wait(ac,0,ATA_SR_BSY,10000,0,0);
	return 0;
}


/*
 * Build CDB for basic READ/WRITE and simple commands.
 */
int
build_cdb_pkt(u8_t opcode, u8_t *cdb, u32_t lba, u16_t nblks)
{
	switch (opcode) {
	case CDB_WRITE_10:
	case CDB_READ_10:
		bzero((caddr_t)cdb, 12);
		cdb[0] = opcode;
		cdb[2] = CDB32_B3(lba);
		cdb[3] = CDB32_B2(lba);
		cdb[4] = CDB32_B1(lba);
		cdb[5] = CDB32_B0(lba);
		cdb[7] = CDB16_H(nblks);
		cdb[8] = CDB16_L(nblks);
		return 12;

	case CDB_MODE_SENSE_10:
	    {
		/*u8_t  page    = (u8_t)(lba >> 8);
		u8_t  subpage = (u8_t)(lba & 0xff);*/

		u8_t  page    = CDB16_H((u16_t)lba);
		u8_t  subpage = CDB16_L((u16_t)lba);
		u16_t len     = nblks;

		bzero((caddr_t)cdb, 12);
		cdb[0] = opcode;
		cdb[2] = (u8_t)((page & 0x3f) |
		                ((subpage != 0) ? 0x40 : 0x00));
		cdb[3] = subpage;
		cdb[7] = CDB16_H(len);
		cdb[8] = CDB16_L(len);
		return 12;
	    }

	case CDB_INQUIRY:
	    {
		u16_t xfer_len = (u16_t)lba;

		bzero((caddr_t)cdb, 12);
		cdb[0] = opcode;
		cdb[1] = 0x00;  /* EVPD=0 */
		cdb[2] = 0x00;  /* page=0 */
		cdb[3] = CDB16_H(xfer_len);
		cdb[4] = CDB16_L(xfer_len); /* allocation length */
		cdb[5] = 0;
		return 12;
	    }

	case CDB_READ_CAPACITY:
	case CDB_TEST_UNIT_READY:
		bzero((caddr_t)cdb, 12);
		cdb[0] = opcode;
		return 12;
	}
	return -1;
}

int
atapi_request(ata_ctrl_t *ac, ata_req_t *r, int arm_ticks)
{
	ata_unit_t  *u = ac->drive[r->drive];
	ata_ioque_t *q = ac->ioque;
	u8_t   sense[18];
	u32_t  blksz, max_bytes, avail, xfer;
	u16_t  nblks;
	int    dir, rc;

	ATADEBUG(2,"atapi_request(Reqid=%ld,arm_ticks=%d)\n",
		 r ? r->reqid : 0, arm_ticks);

	r->flags &= ~ATA_RF_CDB_SENT;

	blksz = (u && u->atapi_blksz) ? u->atapi_blksz : 2048;

	/* Defensive: if both nsec and sectors_left are zero but we still have
	 * a non-zero byte count in the original buf, derive a reasonable
	 * sector count from b_bcount and the device block size. This covers
	 * cases like CD-ROM (2048-byte blocks) when the caller used 512-byte
	 * dd blocks and the strategy layer computed r->nsec == 0.
	 */
	if (r->nsec == 0 && r->sectors_left == 0 &&
	    r->bp != NULL && r->bp->b_bcount > 0) {
		u32_t bytes = (u32_t)r->bp->b_bcount;
		u32_t n     = (bytes + blksz - 1) / blksz; /* ceil(bytes/blksz) */
		if (n == 0)
			n = 1;
		r->nsec         = n;
		r->sectors_left = n;
		r->lba_cur      = r->lba;
	}

	/* Ensure sectors_left is initialised at least once from nsec. */
	if (r->sectors_left == 0 && r->nsec > 0) {
		r->sectors_left = r->nsec;
		if (r->lba_cur < r->lba)
			r->lba_cur = r->lba;
	}

	/* Decide transfer size for this command (align to device block) */
	max_bytes = (q && q->xfer_bufsz) ? q->xfer_bufsz : blksz;
	if (max_bytes < blksz) max_bytes = blksz;
	max_bytes = (max_bytes / blksz) * blksz;

	avail = r->sectors_left * blksz;
	xfer  = (avail < max_bytes) ? avail : max_bytes;
	if (xfer == 0) xfer = blksz;

	nblks = (u16_t)(xfer / blksz);
	dir   = r->is_write ? 0 : 1;

	r->cdb_len = build_cdb_pkt(r->is_write ? CDB_WRITE_10 : CDB_READ_10,
			       r->cdb, r->lba_cur, nblks);

	/* ---------- Interrupt-driven path ---------- */
	if (AC_HAS_FLAG(ac, ACF_INTR_MODE)) {
		int s;

		if (xfer == 0 || nblks == 0)
			return 0;

		if (!q) {
			printf("atapi_request: no queue in IRQ mode\n");
			return EFAULT;
		}

		/* In IRQ mode we use the caller's buffer directly. */
		r->xptr        = (caddr_t)r->addr;
		r->chunk_bytes = xfer;
		r->xfer_off    = 0;

		r->atapi_phase = ATAPI_PHASE_WAIT_PKT_DRQ;
		r->atapi_dir   = r->is_write ? ATAPI_DIR_WRITE : ATAPI_DIR_READ;

		s = splbio();
		q->state = AS_XFER;
		AC_SET_FLAG(ac,ACF_BUSY);
		q->cur   = r;
		splx(s);

		rc = atapi_start_irq(ac, r);
		if (rc != 0) {
			/* Could not issue PACKET, clear busy and fail. */
			s = splbio();
			q->cur   = 0;
			q->state = AS_IDLE;
			AC_CLR_FLAG(ac,ACF_BUSY);
			splx(s);
			return rc;
		}

		r->await_drq_ticks = HZ * 2;
		if (arm_ticks) ide_arm_watchdog(ac, arm_ticks);

		return 0;   /* transfer continues in atapi_service_irq() */
	}

	/* ---------- Polled path (synchronous) ---------- */
	rc = atapi_packet(ac, r->drive, r->cdb, r->cdb_len,
			  (void *)r->addr, xfer, dir,
			  sense, sizeof(sense));
	if (rc == 0) {
		r->lba_cur      += nblks;
		r->addr         += xfer;
		r->sectors_left -= nblks;
		r->xfer_off     += xfer;
		r->chunk_bytes   = xfer;
		r->flags        &= ~ATA_RF_NEEDCOPY;
	} else {
		atapi_decode_sense(sense,sizeof(sense));
	}
	return rc;
}

/* Send an ATAPI PACKET and the 12-byte CDB.
 * Returns 0 on success (CDB accepted), or EIO/ETIMEDOUT on failure.
 */
int
atapi_send_packet(ata_ctrl_t *ac, u8_t drive, u8_t *cdb, int cdb_len)
{
	ata_unit_t *u = ac->drive[drive];
	u8_t 	ast, err, ir, cod, io;
	u16_t 	words_cdb[6], bc;
	int 	i, attempt;

	ATADEBUG(1,"atapi_send_packet(%s, CDB[%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x])\n",
		Cstr(ac),cdb[0],cdb[1],cdb[2],cdb[3],cdb[4],cdb[5],cdb[6],cdb[7],cdb[8],cdb[9],cdb[10],cdb[11]);

	if (cdb_len>12) cdb_len=12;
	for(attempt=0; attempt<2; attempt++) {
		if (attempt == 1) {
			ata_softreset_ctrl(ac);
			if (ata_wait(ac,0,ATA_SR_BSY,200000,&ast,0) != 0) {
				printf("ide_send_packet: timedout\n");
				return ETIMEDOUT;
			}
		}

		bc = u->atapi_blksz ? u->atapi_blksz : 2048;
		if (bc==0) bc=2048;

		atapi_dosend_packet(ac,drive,bc);

		if (ata_wait(ac,ATA_SR_DRQ,ATA_SR_BSY,200000,&ast,0) != 0)
			continue;

		ir = inb(ATA_SECTCNT_O(ac)) & 0x03;
		cod = ir & ATAPI_IR_COD;   /* Command/Data = 0x01 */
		io  = ir & ATAPI_IR_IO;    /* 1 = device->host */
		if (!cod || io) { 
			ATADEBUG(1,"%s: PACKET phase mismatch: ST=%02x IR=%02x (retry)\n",
				Cstr(ac), inb(ATA_ALTSTATUS_O(ac)),ir);
			continue;
		}
		ata_delay400(ac);

		/* Write exactly 12 bytes of CDB (pad to 6 words) */
		atapi_send_cdb(ac, cdb, cdb_len,__LINE__);
		return 0;
	}
	return ETIMEDOUT;
}

/* Error handling: record status/error and finish the request. */
void
atapi_handle_error(ata_ctrl_t *ac,ata_req_t *r,u8_t st)
{
	r->ast = st;
	r->err = inb(ATA_ERROR_O(ac));
	r->atapi_phase = ATAPI_PHASE_ERROR;
	ata_finish_current(ac, EIO, __LINE__);
}

/* Command phase: CoD=1, IO=0 send the CDB if not already sent. */
void
atapi_handle_command_phase(ata_ctrl_t *ac,ata_req_t *r)
{
	r->atapi_phase = ATAPI_PHASE_SEND_PACKET;

	if ((r->flags & ATA_RF_CDB_SENT) == 0) {
		ATADEBUG(2,"ATAPI send CDB len=%d\n", (r->cdb_len + 1) >> 1);
		outsw(ATA_DATA_O(ac), (void *)r->cdb, (r->cdb_len + 1) >> 1);
		r->flags |= ATA_RF_CDB_SENT;
	}

	r->atapi_phase = ATAPI_PHASE_WAIT_DATA;
}

/* Data phase: CoD=0 DATA OUT or DATA IN, depending on IO bit. */
void
atapi_handle_data_phase(ata_ctrl_t *ac,ata_req_t *r,u8_t ir,u16_t bc,u32_t blksz)
{
	u32_t  remain_req, remain_buf, want;
	u16_t  wcount;
	u8_t   st2;

	ATADEBUG(3,"%s: %s xptr=%lx chunk_bytes=%ld bc=%u\n",
		Cstr(ac),
		(ir & ATAPI_IR_IO) ? "DATAIN" : "DATAOUT",
		(u_long)r->xptr, (long)r->chunk_bytes, (unsigned)bc);

	/* No buffer / chunk discard data and fail. */
	if (r->xptr == 0 || r->chunk_bytes == 0) {
		wcount = (u16_t)((int)(bc + 1) >> 1);
		while (wcount--) {
			if ((ir & ATAPI_IR_IO) == 0)
				outw(ATA_DATA_O(ac), 0);
			else
				(void)inw(ATA_DATA_O(ac));
		}
		ata_finish_current(ac, EFAULT, __LINE__);
		r->atapi_phase = ATAPI_PHASE_ERROR;
		return;
	}

 	/* How much the request still needs vs how much of the staging buffer
 	 * chunk is left.
 	 */
	remain_req = r->sectors_left * blksz;
	remain_buf = (r->xfer_off < r->chunk_bytes)
               ? (r->chunk_bytes - r->xfer_off)
               : 0;

	want = bc;
	if (want > remain_req) want = remain_req;
	if (want > remain_buf) want = remain_buf;

	/* Device wants to transfer, but we have nowhere to put/read from. */
	if (want == 0) {
		wcount = (u16_t)((int)(bc + 1) >> 1);
		while (wcount--) {
			if ((ir & ATAPI_IR_IO) == 0)
				outw(ATA_DATA_O(ac), 0);
			else
				(void)inw(ATA_DATA_O(ac));
		}
		ata_finish_current(ac, EFAULT, __LINE__);
		r->atapi_phase = ATAPI_PHASE_ERROR;
		return;
	}

	/* Transfer "want" bytes of data; pad/flush up to bc if needed. */
	wcount = (u16_t)((int)(want + 1) >> 1);

	if ((ir & ATAPI_IR_IO) == 0) {
		/* Data Out: host -> device */
		outsw(ATA_DATA_O(ac),
			(void *)((u8_t *)r->xptr + r->xfer_off), wcount);
		r->atapi_phase = ATAPI_PHASE_PIO_OUT;
	} else {
		/* Data In: device -> host */
		insw(ATA_DATA_O(ac),
			(void *)((u8_t *)r->xptr + r->xfer_off), wcount);
		r->atapi_phase = ATAPI_PHASE_PIO_IN;
	}

	/* If bc > want, discard the remaining words. */
	if ((u32_t)bc > want) {
		u16_t extra = (u16_t)((((u32_t)bc - want) + 1) >> 1);
		while (extra--) {
			if ((ir & ATAPI_IR_IO) == 0)
				outw(ATA_DATA_O(ac), 0);
			else
				(void)inw(ATA_DATA_O(ac));
		}
	}

	r->xfer_off += want;

	/* Update LBA / sectors_left based on full blocks consumed. */
	while (want >= blksz && r->sectors_left) {
		r->sectors_left--;
		r->lba_cur++;
		want -= blksz;
	}

		/* Some drives clear DRQ/BSY immediately after the last data phase;
	 * if so and we've consumed our chunk / sectors, finish now.
	 */
	st2 = inb(ATA_STATUS_O(ac));
	if ((st2 & (ATA_SR_DRQ | ATA_SR_BSY)) == 0) {
		atapi_maybe_finish(ac, r, st2);
		if (r->atapi_phase == ATAPI_PHASE_IDLE ||
		    r->atapi_phase == ATAPI_PHASE_ERROR)
			return;
	}

/* Otherwise expect more data or completion. */
	r->atapi_phase = ATAPI_PHASE_WAIT_DATA;
}

/* Final completion check when DRQ=0. */
void
atapi_maybe_finish(ata_ctrl_t *ac,ata_req_t *r,u8_t st)
{
	ata_ioque_t *q = ac->ioque;

	/* Only consider completion when BSY and DRQ are both clear. */
	if ((st & (ATA_SR_DRQ | ATA_SR_BSY)) == 0) {
		if (r->sectors_left == 0 || r->xfer_off >= r->chunk_bytes) {
			/*
			 * For READs in interrupt mode we staged data into xfer_buf;
			 * copy it back to the caller's buffer now.
			 */
			if (!r->is_write && (r->flags & ATA_RF_NEEDCOPY) && q && q->xfer_buf) {
				bcopy(q->xfer_buf, (caddr_t)r->addr, (size_t)r->chunk_bytes);
				r->addr  += r->chunk_bytes;
				r->flags &= ~ATA_RF_NEEDCOPY;
			}

			ata_finish_current(ac, 0, __LINE__);
			r->atapi_phase = ATAPI_PHASE_IDLE;
		}
	}
}


/* Phase-aware, hardware-driven ATAPI IRQ service routine. */
void
atapi_service_irq(ata_ctrl_t *ac,ata_req_t *r,u8_t st)
{
	ata_unit_t *u;
	u32_t  blksz;
	u16_t  bc;
	u8_t   ir, cod, io;

	if (r == 0) return;

	u = ac->drive[r->drive];
	blksz = (u && u->atapi_blksz) ? u->atapi_blksz : 2048;

	/* Still busy, nothing meaningful to do yet. */
	if (st & ATA_SR_BSY) {
		BUMP(ac, irq_bsy_skipped);
		return;
	}

	/* Hard error or device fault. */
	if (st & (ATA_SR_ERR | ATA_SR_DWF)) {
		r->atapi_phase = ATAPI_PHASE_ERROR;
		atapi_handle_error(ac, r, st);
		return;
	}

	/* No DRQ: this is completion-ish. */
	if ((st & ATA_SR_DRQ) == 0) {
		if (r->atapi_phase == ATAPI_PHASE_WAIT_COMPLETE ||
		    r->atapi_phase == ATAPI_PHASE_PIO_IN ||
		    r->atapi_phase == ATAPI_PHASE_PIO_OUT ||
		    r->atapi_phase == ATAPI_PHASE_WAIT_DATA) {
			atapi_maybe_finish(ac, r, st);
		} else {
			printf("atapi: DRQ=0 in phase %d, status=0x%02x\n",
						(int)r->atapi_phase, st);
			r->atapi_phase = ATAPI_PHASE_ERROR;
			atapi_maybe_finish(ac, r, st);
		}
		return;
	}

	/* ---- DRQ is set: we must service a phase ---- */
	ir  = inb(ATA_SECTCNT_O(ac)) & 0x03;
	cod = ir & ATAPI_IR_COD;   /* Command/Data */
	io  = ir & ATAPI_IR_IO;    /* 1 = device->host */

	bc  = (u16_t)inb(ATA_CYLLOW_O(ac));
	bc |= (u16_t)inb(ATA_CYLHIGH_O(ac)) << 8;
	if (bc == 0) bc = (u16_t)blksz;

	/* Command phase: CoD=1, IO=0. */
	if (cod && !io) {
		if (r->atapi_phase != ATAPI_PHASE_WAIT_PKT_DRQ &&
		    r->atapi_phase != ATAPI_PHASE_SEND_PACKET) {
			printf("atapi: unexpected CDB phase, phase=%d st=0x%02x ir=0x%02x\n",
				(int)r->atapi_phase, st, ir);
		}
		atapi_handle_command_phase(ac, r);
		return;
	}

	/* Data phase: CoD=0 (in or out). */
	if (!cod) {
		if (io) {
			/* Device -> host: PIO IN */
			if (r->atapi_phase != ATAPI_PHASE_WAIT_DATA &&
			    r->atapi_phase != ATAPI_PHASE_PIO_IN) {
				printf("atapi: unexpected DATA IN phase, phase=%d st=0x%02x ir=0x%02x\n",
					(int)r->atapi_phase, st, ir);
			}
			r->atapi_phase = ATAPI_PHASE_PIO_IN;
		} else {
			/* Host -> device: PIO OUT */
			if (r->atapi_phase != ATAPI_PHASE_WAIT_DATA &&
			    r->atapi_phase != ATAPI_PHASE_PIO_OUT) {
				printf("atapi: unexpected DATA OUT phase, phase=%d st=0x%02x ir=0x%02x\n",
					(int)r->atapi_phase, st, ir);
			}
			r->atapi_phase = ATAPI_PHASE_PIO_OUT;
		}

		atapi_handle_data_phase(ac, r, ir, bc, blksz);
		return;
	}

	/* cod=1, io=1 weird combo. Treat as an error / unexpected phase. */
	printf("atapi: unexpected IR combination cod=%d io=%d phase=%d st=0x%02x\n",
           		(int)cod, (int)io, (int)r->atapi_phase, st);
	r->atapi_phase = ATAPI_PHASE_ERROR;
}

void
atapi_poll_engine(ata_ctrl_t *ac)
{
	printf("atapi_poll_engine()\n");
}
