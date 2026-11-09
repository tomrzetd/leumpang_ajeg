// Bagian portabel layanan berkas (juga STM32).
#include "leumpang/files.h"

namespace la {

bool sanitizePath(const char* in, size_t n, char* out, size_t outLen) {
    if (!out || outLen < 2) return false;
    size_t k = 0, seg = 0;
    for (size_t i = 0; i < n && in[i]; ++i) {
        char c = in[i];
        if (c == '\\') return false;
        if (c == '/') {
            if (seg == 0) continue;                         // buang '/' ganda / di depan
            seg = 0;
        } else {
            const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                            c == '_' || c == '-' || c == '.' || c == ' ';
            if (!ok) return false;
            if (c == '.' && seg == 0 && i + 1 < n && in[i + 1] == '.') return false;   // ".."
            ++seg;
        }
        if (k + 1 >= outLen || k >= 95) return false;
        out[k++] = c;
    }
    while (k > 0 && out[k - 1] == '/') --k;
    out[k] = 0;
    return true;
}

}  // namespace la
