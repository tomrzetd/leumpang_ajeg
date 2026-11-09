// Backend berkas POSIX: ESP-IDF (VFS FAT di /sd) dan simulator host. Tidak dipakai STM32 (FatFs).
// Windows (studio MSVC): direktori lewat std::filesystem, sisanya CRT.
#include "leumpang/files.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#include <io.h>
#include <filesystem>
#include <string>
#ifndef S_ISDIR
#define S_ISDIR(m) (((m) & _S_IFMT) == _S_IFDIR)
#endif
#define LA_MKDIR(p) ::_mkdir(p)
#define LA_RMDIR(p) ::_rmdir(p)
#define LA_UNLINK(p) ::_unlink(p)
#else
#include <dirent.h>
#include <unistd.h>
#define LA_MKDIR(p) ::mkdir(p, 0775)
#define LA_RMDIR(p) ::rmdir(p)
#define LA_UNLINK(p) ::unlink(p)
#endif

namespace la {

namespace {
class PosixFs final : public FileOps {
public:
    explicit PosixFs(const char* root) { snprintf(root_, sizeof root_, "%s", root); }
    bool available() override { struct stat st; return ::stat(root_, &st) == 0 && S_ISDIR(st.st_mode); }

    int list(const char* path, uint16_t start, FileEntry* out, int max, bool& more) override {
        char p[160]; full(path, p);
        int idx = 0, n = 0; more = false;
#ifdef _WIN32
        std::error_code ec;
        std::filesystem::directory_iterator it(p, ec), end;
        if (ec) return available() ? -FS_NOT_FOUND : -FS_NO_SD;
        for (; it != end; it.increment(ec)) {
            if (ec) break;
            const std::string nm = it->path().filename().string();
            const char* dn = nm.c_str();
#else
        DIR* d = opendir(p);
        if (!d) return available() ? -FS_NOT_FOUND : -FS_NO_SD;
        while (dirent* e = readdir(d)) {
            const char* dn = e->d_name;
#endif
            if (!strcmp(dn, ".") || !strcmp(dn, "..")) continue;
            if (idx++ < start) continue;
            if (n >= max) { more = true; break; }
            FileEntry& f = out[n++];
            const size_t nl = strlen(dn);
            if (nl >= sizeof f.name) { --n; continue; }          // nama terlalu panjang untuk protokol
            memcpy(f.name, dn, nl + 1);
            char fp[224];
            if ((size_t)snprintf(fp, sizeof fp, "%s/%.*s", p, (int)nl, dn) >= sizeof fp) { --n; continue; }
            struct stat st{};
            ::stat(fp, &st);
            f.dir = S_ISDIR(st.st_mode); f.size = f.dir ? 0 : (uint32_t)st.st_size;
        }
#ifndef _WIN32
        closedir(d);
#endif
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
        if (LA_UNLINK(p) == 0 || LA_RMDIR(p) == 0) return FS_OK;
        return errno == ENOENT ? FS_NOT_FOUND : FS_IO;
    }
    FileStatus mkdir(const char* path) override {
        char p[160]; full(path, p);
        if (LA_MKDIR(p) == 0) return FS_OK;
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
