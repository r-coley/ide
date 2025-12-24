#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioccom.h>
#include "ide.h"

static
void
make_link(char *target,char *linkpath)
{
	struct stat st;

	if (lstat(linkpath,&st) == 0) {
		if (unlink(linkpath) != 0) {
			fprintf(stderr,"unlink(%s) failed: %s\n",
				linkpath,strerror(errno));
			return;
		}
	} else {
		if (errno != ENOENT) {
			fprintf(stderr,"lstat(%s) failed: %s\n",
				linkpath,strerror(errno));
			return;
		}
	}
	if (symlink(target,linkpath) != 0) {
		fprintf(stderr,"symlink(%s->%s) failed: %s\n",	
				linkpath,target,strerror(errno));	
	}
}

int
main(void)
{
	int C, D, u;
	int cd=0, zip=0;
	char linkpath[64];

	/*** Clean old entries ***/
	for(u=0;u<(2*4);u++) {
		sprintf(linkpath,"/dev/cdrom%d",u);
		unlink(linkpath);
		sprintf(linkpath,"/dev/zip%d",u);
		unlink(linkpath);
	}

	for(C=0;C<4;C++) {
	    for(D=0;D<2;D++) {
		char devpath[64];
		int fd;
		struct v_gettype gt;

		sprintf(devpath,"/dev/rata/c%dd%dp0", C,D);

		printf("Trying to open [%s]\n",devpath);
		fd = open(devpath,O_RDONLY|O_NONBLOCK);
		if (fd < 0) continue;

		memset(&gt,0,sizeof(gt));
		if (ioctl(fd,V_GETTYPE,&gt) != 0) continue;
		if (!(gt.flags & (UF_PRESENT|UF_ATAPI))) continue;

	 	if (gt.flags & UF_CDROM) {
			sprintf(linkpath,"/dev/cdrom%d",cd);
			make_link(devpath,linkpath);

			printf("cdrom%d -> %s (%s)\n", cd,devpath,gt.model);
			cd++;
		}

	 	if (gt.flags & UF_MOZIP) {
			sprintf(linkpath,"/dev/zip%d",zip);
			make_link(devpath,linkpath);

			printf("zip%d -> %s (%s)\n", zip,devpath,gt.model);
			zip++;
		}
		close(fd);
	    }
	}
	exit(0);
}
