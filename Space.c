#include "sys/types.h"
#include "sys/conf.h"
#include "config.h"


#define ATAMAJOR0       ATA_CMAJOR_0    /* Board major device number */

int	ata_major = ATAMAJOR0;		/* major device number */
