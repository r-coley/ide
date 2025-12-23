/*
 * ide_funcs.h
 */

#ifndef _IDE_FUNCS_H
#define _IDE_FUNCS_H

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
#include <sys/xdebug.h>
#include <sys/kdebugger.h>

#include "ide.h"
#include "ide_hw.h"

extern 	ata_ctrl_t ata_ctrl[];
extern	int 	atadebug;
extern	int 	ata_force_polling;
extern	int 	atapi_force_polling;
extern 	ata_unit_t ata_unit[];
extern	u32_t req_seq;

/*** ide_core ***/
void 	ataprint(dev_t, char *);
int 	ataopen(dev_t *, int, int, cred_t *);
int 	ataclose(dev_t, int, int, cred_t *);
void 	atabreakup(struct buf *);
int 	atastrategy(struct buf *);
int 	ataread(dev_t, struct uio *, cred_t *);
int 	atawrite(dev_t, struct uio *, cred_t *);
int 	ataioctl(dev_t, int, caddr_t, int, cred_t *, int *);
int 	atasize(dev_t dev);
int 	atainit(void);
ata_ctrl_t *atafindctrl(int);
int 	ataintr(int);

/*** ide_queue ***/
void 	ide_arm_watchdog(ata_ctrl_t *, int);
void 	ide_cancel_watchdog(ata_ctrl_t *);
void 	ide_watchdog(caddr_t);
void 	ide_start(ata_ctrl_t *);
ata_req_t *ide_q_get(ata_ctrl_t *);
void 	ide_q_put(ata_ctrl_t *, ata_req_t *);
void 	ide_kick(ata_ctrl_t *);
void 	ide_need_kick(ata_ctrl_t *);

/*** ide_ata ***/
int 	ata_sel(ata_ctrl_t *,int, u32_t);
void 	ata_delay400(ata_ctrl_t *);
int 	ata_wait(ata_ctrl_t *, u8_t, u8_t, long, u8_t *, u8_t *);
int 	ata_identify(ata_ctrl_t *, int);
int 	ata_flush_cache(ata_ctrl_t *,u8_t);
void 	ata_quiesce_ctrl(ata_ctrl_t *);
void	ata_dump_stats(void);
void 	ata_softreset_ctrl(ata_ctrl_t *);
int 	ata_enable_pio_multiple(ata_ctrl_t *, u8_t, u8_t);
void 	ata_negotiate_pio_multiple(ata_ctrl_t *, u8_t);
int 	ata_err(ata_ctrl_t *, u8_t *, u8_t *);
void 	ata_rescue(int);
void 	ata_rescueit(ata_ctrl_t *);
int 	pio_one_sector(ata_ctrl_t *, ata_req_t *);
void 	ata_service_irq(ata_ctrl_t *, ata_req_t *, u8_t);
void 	ata_program_taskfile(ata_ctrl_t *, ata_req_t *);
int 	ata_program_next_chunk(ata_ctrl_t *,ata_req_t *,int);
int 	ata_prog_pio(ata_ctrl_t *,ata_req_t *,int);
void 	ata_finish_current(ata_ctrl_t *, int,int);
int 	ata_data_phase_service(ata_ctrl_t *,ata_req_t *);
void 	ata_prime_write(ata_ctrl_t *, ata_req_t *);
int	ata_pushreq(ata_ctrl_t *,ata_req_t *);
int	multicmd(ata_ctrl_t *,int,u32_t,u32_t);

/*** ide_atapi ***/
void 	atapi_program_packet(ata_ctrl_t *, ata_req_t *, u16_t);
void 	atapi_dosend_packet(ata_ctrl_t *, int, u16_t,int);
int 	atapi_read_capacity(ata_ctrl_t *ac, u8_t, u32_t *, u32_t *);
int 	atapi_inquiry(ata_ctrl_t *, u8_t);
int 	atapi_read10(ata_ctrl_t *, u8_t, u32_t, u16_t,void *);
int 	atapi_mode_sense10(ata_ctrl_t *, u8_t, u8_t, u8_t, void *, u16_t);
int 	atapi_mode_sense6(ata_ctrl_t *, u8_t, u8_t, u8_t, void *, u8_t);
int 	atapi_packet(ata_ctrl_t *, u8_t, u8_t *, int, void *, u32_t, int, u8_t *, int,int);
int	atapi_test_unit_ready(ata_ctrl_t *, u8_t);
char 	*atapi_class_name(ata_unit_t *);
int 	atapi_write10(ata_ctrl_t *, u8_t, u32_t, u16_t,const void *);
void 	atapi_send_cdb(ata_ctrl_t *, u8_t *, int, int);
int	atapi_start_irq(ata_ctrl_t *, ata_req_t *);
void	atapi_service_irq(ata_ctrl_t *, ata_req_t *, u8_t);
int 	build_cdb_pkt(u8_t, u8_t *, u32_t, u32_t);
int 	atapi_prog_packet(ata_ctrl_t *, ata_req_t *, int); /*WHY*/
int 	atapi_send_packet(ata_ctrl_t *, u8_t, u8_t *, int);
int 	atapi_read_toc(ata_ctrl_t *, u8_t, int, u8_t, u8_t, void *, u16_t);
int 	atapi_play_audio_msf(ata_ctrl_t *, u8_t,
			      u8_t, u8_t, u8_t,
			      u8_t, u8_t, u8_t);
int 	atapi_pause_resume(ata_ctrl_t *, u8_t, int);
int 	atapi_start_stop(ata_ctrl_t *, u8_t, int, int);

void 	atapi_handle_error(ata_ctrl_t *, ata_req_t *, u8_t);
void 	atapi_handle_command_phase(ata_ctrl_t *, ata_req_t  *);
void 	atapi_handle_data_phase(ata_ctrl_t *,ata_req_t *,u8_t,u16_t,u32_t);
void 	atapi_maybe_finish(ata_ctrl_t *,ata_req_t *,u8_t,int);
void 	atapi_decode_sense(u8_t *, int);

/*** ide_misc ***/
char 	*getstr(char *, int, int, int, int);
void	ATADEBUG(int,char *,...);
int 	ata_getblock(dev_t, daddr_t, caddr_t, u32_t);
int 	ata_putblock(dev_t, daddr_t, caddr_t, u32_t);
int 	berror(struct buf *, int, int);
int 	bok(struct buf *, int);
char 	*Cstr(ata_ctrl_t *);
char 	*Dstr(dev_t);
char 	*Istr(int);
void 	reset_queue(ata_ctrl_t *,int);
void 	ata_attach(int);
int 	ata_read_vtoc(dev_t, int);
void 	ata_copy_model(u16_t *, char *);
int 	ata_read_signature(ata_ctrl_t *, u8_t, u16_t *);
int 	ata_probe_unit(ata_ctrl_t *,u8_t);
void 	ata_region_from_dev(dev_t, u32_t *, u32_t *);
void 	CopyTbl(ata_part_t *,struct ipart *);
int 	ata_pdinfo(dev_t);
void 	ide_poll_engine(ata_ctrl_t *);

#endif /* _IDE_FUNCS_H */
