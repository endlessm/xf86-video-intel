#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>

int main(int argc, char *argv[])
{
	struct stat st;
	char buf[1024], *b = buf;
	int len, fd;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <iface>\n", argv[0]);
		return 1;
	}

	snprintf(buf, sizeof(buf), "/sys/class/backlight/%s/brightness", argv[1]);
	fd = open(buf, O_RDWR);
	if (fd < 0 || fstat(fd, &st) || major(st.st_dev)) {
		fprintf(stderr, "Cannot access backlight interface '%s'\n", argv[1]);
		return 1;
	}

	while ((len = read(0, b, sizeof(buf) - (b - buf) - 1)) > 0) {
		len += b - buf;
		buf[len] = '\0';

		b = buf;
		do {
			char *end = strchr(b, '\n');
			if (end == NULL)
				break;

			++end;
			if (write(fd, b, end - b) != end - b) {
				fprintf(stderr, "Failed to update backlight interface '%s'\n", argv[1]);
				return 2;
			}

			b = end;
		} while (1);

		memmove(buf, b, len = buf + len - b);
		b = buf + len;
	}

	return 0;
}
