#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>

int main(int argc, char *argv[])
{
	struct stat st;
	char buf[1024];
	int len, fd;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <iface>\n", argv[0]);
		return 1;
	}

	if (strchr(argv[1], '/') != NULL) {
		fprintf(stderr, "Invalid interface name\n");
		return 1;
	}
	if (snprintf(buf, sizeof(buf), "/sys/class/backlight/%s/brightness",
		argv[1]) >= sizeof(buf)) {
		fprintf(stderr, "Interface name is too long\n");
		return 1;
	}
	fd = open(buf, O_RDWR);
	if (fd < 0 || fstat(fd, &st) || major(st.st_dev)) {
		fprintf(stderr, "Cannot access backlight interface '%s'\n", argv[1]);
		return 1;
	}

	while (fgets(buf, sizeof(buf), stdin)) {
		len = strlen(buf);
		if (write(fd, buf, len) != len) {
			fprintf(stderr, "Failed to update backlight interface '%s'\n", argv[1]);
			return 2;
		}
	}

	return 0;
}
