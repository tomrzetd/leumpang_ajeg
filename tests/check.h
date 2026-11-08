// Kerangka tes minimal (tanpa dependensi)
#pragma once
#include <cstdio>
#include <cstring>

static int g_checkFails = 0, g_checkTotal = 0;
#define CHECK(c, msg) do { ++g_checkTotal; if (!(c)) { ++g_checkFails; printf("FAIL  %s  (%s:%d)\n", msg, __FILE__, __LINE__); } \
                           else printf("ok    %s\n", msg); } while (0)
#define CHECK_EQ(a, b, msg) do { const long long va_ = (long long)(a), vb_ = (long long)(b); ++g_checkTotal; \
    if (va_ != vb_) { ++g_checkFails; printf("FAIL  %s  (dapat %lld, harap %lld) (%s:%d)\n", msg, va_, vb_, __FILE__, __LINE__); } \
    else printf("ok    %s\n", msg); } while (0)
inline int checkSummary(const char* name) {
    printf("== %s: %d/%d lulus\n", name, g_checkTotal - g_checkFails, g_checkTotal);
    return g_checkFails ? 1 : 0;
}
