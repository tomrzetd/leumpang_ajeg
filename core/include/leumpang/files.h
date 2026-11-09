// =============================================================================
//  leumpang-core - layanan berkas SD lewat LAP (Fase 4A).
//  Dipakai desktop (manajer berkas, unggah script/rekaman, kalibrasi profil) dan kelak HP.
//  Jalur selalu relatif akar SD, dipisah '/', tanpa "..", maks 95 karakter.
// =============================================================================
#pragma once
#include <cstddef>
#include <cstdint>

namespace la {

enum FileStatus : uint8_t { FS_OK = 0, FS_NOT_FOUND = 1, FS_IO = 2, FS_BAD_PATH = 3, FS_NO_SD = 4, FS_DENIED = 5, FS_EXISTS = 6 };

struct FileEntry { char name[48]; uint32_t size; bool dir; };

/** Backend berkas (POSIX/VFS ESP-IDF, FatFs STM32, folder host di simulator). Dipanggil dari thread komunikasi. */
class FileOps {
public:
    virtual ~FileOps() = default;
    virtual bool available() = 0;
    /** Isi direktori mulai indeks 'start'. Mengembalikan jumlah entri (<= max), more = masih ada; <0 = -FileStatus. */
    virtual int list(const char* path, uint16_t start, FileEntry* out, int max, bool& more) = 0;
    /** Baca hingga len byte dari offset. Mengembalikan jumlah byte (0 = EOF) atau -FileStatus. */
    virtual int read(const char* path, uint32_t offset, uint8_t* buf, size_t len) = 0;
    /** Tulis di offset; truncate = buat/kosongkan berkas dulu. */
    virtual FileStatus write(const char* path, uint32_t offset, const uint8_t* d, size_t n, bool truncate) = 0;
    virtual FileStatus remove(const char* path) = 0;
    virtual FileStatus mkdir(const char* path) = 0;
    virtual FileStatus rename(const char* from, const char* to) = 0;
    virtual FileStatus stat(const char* path, uint32_t& size, bool& dir) = 0;
};

/** Validasi jalur dari host; tulis bentuk bersih ke out ("" = akar). */
bool sanitizePath(const char* in, size_t n, char* out, size_t outLen);

/** Backend POSIX: root = "/sd" (ESP-IDF) atau folder host. Tidak dikompilasi untuk STM32. */
FileOps* posixFileOps(const char* root);

}  // namespace la
