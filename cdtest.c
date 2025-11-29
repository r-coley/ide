/*
 * Simple userland test program for ATAPI CD-ROM TOC and audio ioctls.
 *
 * Usage:
 *   cc -o cdtest cdtest.c
 *   ./cdtest [/dev/cdrom0]
 */

#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef unsigned int uint32_t;
typedef unsigned int * uintptr_t;

/* Must match ide.h */
#define CDIOC_BASE      ('C'<<8)
#define CDIOC_READTOC   (CDIOC_BASE | 0x01)
#define CDIOC_PLAYMSF   (CDIOC_BASE | 0x02)

typedef struct cd_toc_io {
    uint32_t toc_buf;   /* user pointer */
    unsigned short toc_len;
    unsigned char  msf;
    unsigned char  format;
    unsigned char  track;
    unsigned char  reserved;
} cd_toc_io_t;

typedef struct cd_msf_io {
    unsigned char start_m;
    unsigned char start_s;
    unsigned char start_f;
    unsigned char end_m;
    unsigned char end_s;
    unsigned char end_f;
} cd_msf_io_t;

struct track_info {
    int track;
    int adr;
    int control;
    int m, s, f;
};

static void
print_usage(const char *prog)
{
    fprintf(stderr, "Usage: %s [/dev/cdrom0]\n", prog);
}

static int
parse_toc_msf(const unsigned char *toc, unsigned short len,
              struct track_info *tracks, int max_tracks,
              int *first_track, int *last_track)
{
    unsigned short toc_data_len;
    int n_entries, i;
    const unsigned char *p;

    if (len < 4)
        return -1;

    toc_data_len = (toc[0] << 8) | toc[1];
    *first_track = toc[2];
    *last_track  = toc[3];

    if (toc_data_len < 2)
        return -1;

    n_entries = (int)(toc_data_len - 2) / 8;
    if (n_entries > max_tracks)
        n_entries = max_tracks;

    p = toc + 4;

    for (i = 0; i < n_entries; i++) {
        unsigned char adr_ctl = p[1];
        unsigned char track   = p[2];
        unsigned char m       = p[5];
        unsigned char s       = p[6];
        unsigned char f       = p[7];

        tracks[i].track   = track;
        tracks[i].adr     = (adr_ctl >> 4) & 0x0F;
        tracks[i].control = adr_ctl & 0x0F;
        tracks[i].m       = m;
        tracks[i].s       = s;
        tracks[i].f       = f;

        p += 8;
    }

    return n_entries;
}

static void
print_tracks(const struct track_info *tracks, int n_tracks,
             int first_track, int last_track)
{
    int i;

    printf("First track: %d, Last track: %d\n", first_track, last_track);
    printf("Track  ADR  CTRL  Type   Start (MM:SS:FF)\n");
    printf("-----  ---  ----  ------  --------------\n");

    for (i = 0; i < n_tracks; i++) {
        const char *type;

        if (tracks[i].control & 0x04)
            type = "data";
        else
            type = "audio";

        printf("%5d  %3d  0x%02x  %-5s  %02d:%02d:%02d\n",
               tracks[i].track,
               tracks[i].adr,
               tracks[i].control,
               type,
               tracks[i].m, tracks[i].s, tracks[i].f);
    }
}

static int
find_track_range(const struct track_info *tracks, int n_tracks,
                 int track,
                 unsigned char *sm, unsigned char *ss, unsigned char *sf,
                 unsigned char *em, unsigned char *es, unsigned char *ef)
{
    int i;

    for (i = 0; i < n_tracks; i++) {
        if (tracks[i].track == track) {
            int j;

            *sm = (unsigned char)tracks[i].m;
            *ss = (unsigned char)tracks[i].s;
            *sf = (unsigned char)tracks[i].f;

            if (i + 1 < n_tracks) {
                *em = (unsigned char)tracks[i+1].m;
                *es = (unsigned char)tracks[i+1].s;
                *ef = (unsigned char)tracks[i+1].f;
            } else {
                int m = tracks[i].m + 5;
                *em = (unsigned char)m;
                *es = (unsigned char)tracks[i].s;
                *ef = (unsigned char)tracks[i].f;
            }
            return 0;
        }
    }
    return -1;
}

int
main(int argc, char **argv)
{
    const char *dev = "/dev/cdrom0";
    int fd;
    unsigned char toc[4096];
    cd_toc_io_t tio;
    struct track_info tracks[128];
    int first_track, last_track;
    int n_tracks;
    int rc;

    if (argc > 2) {
        print_usage(argv[0]);
        return 1;
    }
    if (argc == 2)
        dev = argv[1];

    fd = open(dev, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        perror(dev);
        return 1;
    }

    memset(&tio, 0, sizeof(tio));
    tio.toc_buf = (uint32_t)(uintptr_t)toc;
    tio.toc_len = sizeof(toc);
    tio.msf     = 1;
    tio.format  = 0x00;
    tio.track   = 0x00;

    rc = ioctl(fd, CDIOC_READTOC, &tio);
    if (rc < 0) {
        perror("CDIOC_READTOC");
        close(fd);
        return 1;
    }

    printf("TOC: %u bytes returned\n", tio.toc_len);

    n_tracks = parse_toc_msf(toc, tio.toc_len, tracks,
                             (int)(sizeof(tracks)/sizeof(tracks[0])),
                             &first_track, &last_track);
    if (n_tracks <= 0) {
        fprintf(stderr, "Failed to parse TOC\n");
        close(fd);
        return 1;
    }

    print_tracks(tracks, n_tracks, first_track, last_track);

    printf("\nEnter track number to play (0 to skip): ");
    fflush(stdout);

    {
        int t = 0;
        if (scanf("%d", &t) == 1 && t > 0) {
            unsigned char sm, ss, sf, em, es, ef;
            cd_msf_io_t msf;

            if (find_track_range(tracks, n_tracks, t,
                                 &sm, &ss, &sf, &em, &es, &ef) != 0) {
                fprintf(stderr, "Track %d not found in TOC\n", t);
            } else {
                msf.start_m = sm;
                msf.start_s = ss;
                msf.start_f = sf;
                msf.end_m   = em;
                msf.end_s   = es;
                msf.end_f   = ef;

                printf("Playing track %d: %02d:%02d:%02d to %02d:%02d:%02d\n",
                       t, sm, ss, sf, em, es, ef);

                if (ioctl(fd, CDIOC_PLAYMSF, &msf) < 0) {
                    perror("CDIOC_PLAYMSF");
                } else {
                    printf("PLAYMSF issued successfully.\n");
                }
            }
        }
    }

    close(fd);
    return 0;
}
