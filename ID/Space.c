#include "sys/types.h"
#include "sys/conf.h"
#include "sys/ide.h"
#include "config.h"

#define ATAMAJOR0       ATA_CMAJOR_0    /* Board major device number */

int	ata_major = ATAMAJOR0;		/* major device number */

/* Exported globals expected by SVR4 */
int 	atadevflag = D_NEW | D_DMA;
int	atadebug   = 0;
int 	ata_intr_mode = 1;
int 	atapi_intr_mode = 1;
int	ata_debug_console = 0;

ata_ctrl_t ata_ctrl[ATA_MAX_CTRL] = {
	{ 0x1F0, 14, ACF_NONE }, /* c0 (Primary)   */
	{ 0x170, 15, ACF_PRESENT }, /* c1 (Secondary) */
	{ 0x1E8, 11, ACF_PRESENT }, /* c2 (Tertiary) */
	{ 0x168, 10, ACF_NONE    }  /* c3 (Quaternary) */
};

