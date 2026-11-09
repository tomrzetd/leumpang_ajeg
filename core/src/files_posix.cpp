// Backend berkas POSIX: ESP-IDF (VFS FAT di /sd) dan simulator host. Tidak dipakai STM32 (FatFs).
#include "leumpang/files.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

namespace la {

namespace {
class PosixFs final : public FileOps {
public:
    explicit PosixFs(const char* root) { snprintf(root_, sizeof root_, "%s", root); }
    bool available() override { struct stat st; return ::stat(root_, &st) == 0 && S_ISDIR(st.st_mode); }

    int list(const char* path, uint16_t start, FileEntry* out, int max, bool& more) override {
        char p[160]; full(path, p);
        DIR* d = opendir(p);
        if (!d) return available() ? -FS_NOT_FOUND : -FS_NO_SD;
        int idx = 0, n = 0; more = false;
        while (dirent* e = readdir(d)) {
            if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..")) continue;
            if (idx++ < start) continue;
            if (n >= max) { more = true; break; }
            FileEntry& f = out[n++];
            const size_t nl = strlen(e->d_name);
            if (nl >= sizeof f.name) { --n; continue; }          // nama terlalu panjang untuk protokol
            memcpy(f.name, e->d_name, nl + 1);
            char fp[224];
            if ((size_t)snprintf(fp, sizeof fp, "%s/%.*s", p, (int)nl, e->d_name) >= sizeof fp) { --n; continue; }
            struct stat st{};
            ::stat(fp, &st);
            f.dir = S_ISDIR(st.st_mode); f.size = f.dir ? 0 : (uint32_t)st.st_size;
        }
        closedir(d);
        return n;
    }
    int read(const char* path, uint32_t off, uint8_t* buf, size_t len) override {
        char p[160]; full(path, p);
        FILE* f = fopen(p, "rb");
        if (!f) return available() ? -FS_NOT_FOUND : -FS_NO_SD;
        int r = -FS_IO;
        if (fseek(f, (long)off, SEEK_SET) == 0) r = (int)fread(buf, 1, len, f);
        fclose(f);
        return r;
    }
    FileStatus write(const char* path, uint32_t off, const uint8_t* d, size_t n, bool trunc) override {
        if (!available()) return FS_NO_SD;
        char p[160]; full(path, p);
        FILE* f = fopen(p, trunc ? "wb" : "r+b");
        if (!f) return FS_NOT_FOUND;
        FileStatus st = FS_OK;
        if (fseek(f, (long)off, SEEK_SET) != 0 || (n && fwrite(d, 1, n, f) != n)) st = FS_IO;
        if (fclose(f) != 0) st = FS_IO;
        return st;
    }
    FileStatus remove(const char* path) override {
        char p[160]; full(path, p);
        if (path[0] == 0) return FS_DENIED;
        if (::unlink(p) == 0 || ::rmdir(p) == 0) return FS_OK;
        return errno == ENOENT ? FS_NOT_FOUND : FS_IO;
    }
    FileStatus mkdir(const char* path) override {
        char p[160]; full(path, p);
        if (::mkdir(p, 0775) == 0) return FS_OK;
        return errno == EEXIST ? FS_EXISTS : FS_IO;
    }
    FileStatus rename(const char* a, const char* b) override {
        char pa[160], pb[160]; full(a, pa); full(b, pb);
        return ::rename(pa, pb) == 0 ? FS_OK : (errno == ENOENT ? FS_NOT_FOUND : FS_IO);
    }
    FileStatus stat(const char* path, uint32_t& size, bool& dir) override {
        char p[160]; full(path, p);
        struct stat st{};
        if (::stat(p, &st) != 0) return available() ? FS_NOT_FOUND : FS_NO_SD;
        dir = S_ISDIR(st.st_mode); size = dir ? 0 : (uint32_t)st.st_size;
        return FS_OK;
    }
private:
    void full(const char* rel, char* out) { snprintf(out, 160, "%s%s%s", root_, rel[0] ? "/" : "", rel); }
    char root_[48];
};
}  // namespace

FileOps* posixFileOps(const char* root) { return new PosixFs(root); }

}  // namespace la
