/* Stub syscall newlib minimal. Tidak ada stdio/berkas lewat newlib di firmware ini
 * (konsol lewat SerialLink/CDC, berkas lewat FatFs), jadi semuanya gagal dengan rapi. */
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

int _close(int fd) { (void)fd; errno = EBADF; return -1; }
int _fstat(int fd, struct stat *st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
int _lseek(int fd, int off, int dir) { (void)fd; (void)off; (void)dir; return 0; }
int _read(int fd, char *p, int n) { (void)fd; (void)p; (void)n; errno = EIO; return -1; }
int _write(int fd, const char *p, int n) { (void)fd; (void)p; return n; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
void _exit(int code) { (void)code; for (;;) { } }
