/*
 *  fs/partitions/bubl.c
 *
 *  Alessandro Rubini for BTicino, 2010
 */

#include "check.h"
#include "bubl.h"

struct bubl_part {
	__le32 sstart; /* start sector */
	__le32 ssize;  /* sector size */
	char name[8];
};

#define BUBL_MAGIC0	0x6c627562 /* "bubl", little-endian */
#define BUBL_MAGIC1	0x74726170 /* "part", little-endian */
#define BUBL_VERSION	0

struct bubl_ptable {
	__le32 magic0;
	__le32 magic1;
	__le32 version;
	__le32 unused;
	struct bubl_part part[15];
};

static const int bubl_addr[] = { /* FIXME: I want a better way */
	0, 4*1024, 8*1024, 16*1024, 32*1024, 64*1024, 128*1024, 256*1024, -1
};

int bubl_partition(struct parsed_partitions *state, struct block_device *bdev)
{
	int i, pos;
	Sector sect;
	unsigned char *data;
	struct bubl_ptable *ptable;

	for (pos = 0; bubl_addr[pos] != -1; pos++) {
		/* Check at this position */
		data = read_dev_sector(bdev, bubl_addr[pos] / 512, &sect);
		if (!data)
			return -1;

		ptable = (typeof(ptable))data;

		if (ptable->magic0 != BUBL_MAGIC0
		    || ptable->magic1 != BUBL_MAGIC1) {
			printk(KERN_DEBUG "ptable@%x: bad magic\n",
			       bubl_addr[pos]);
			goto next;
		}
		if (ptable->version != BUBL_VERSION) {
			printk(KERN_DEBUG "ptable@%x: bad version 0x%x\n",
			       bubl_addr[pos],
				ptable->version);
			goto next;
		}
		for (i = 0;
		     i < sizeof(ptable->part)/sizeof(ptable->part[0]);
		     i++) {
			if (ptable->part[i].ssize)
				put_partition(state, i+1,
					      ptable->part[i].sstart,
					      ptable->part[i].ssize);
		}
		put_dev_sector(sect);
		return 1;
	next:
		put_dev_sector(sect);
	}
	return 0;
}
