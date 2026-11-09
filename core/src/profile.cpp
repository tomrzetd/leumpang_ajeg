#include "leumpang/profile.h"
#include "leumpang/controller.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace la {

uint8_t RobotProfile::presentCount() const {
    uint8_t n = 0;
    for (auto& b : present) for (bool p : b) n += p ? 1 : 0;
    return n;
}

static void homeFromPositions(const ServoPositions& v, int16_t o[SERVO_BUSES][SERVO_SLOTS]) {
    for (int b = 0; b < 2; ++b) {
        o[b][0] = b == 0 ? v.waist : v.head;
        o[b][1] = v.U0[b]; o[b][2] = v.U1[b]; o[b][3] = v.U2[b]; o[b][4] = v.E[b];
        o[b][5] = v.K2[b]; o[b][6] = v.K1[b]; o[b][7] = v.K0[b]; o[b][8] = v.H[b]; o[b][9] = v.A0[b]; o[b][10] = v.A1[b];
    }
}

const RobotProfile& defaultProfile() {
    static RobotProfile p;
    static bool init = false;
    if (!init) {
        memset(&p, 0, sizeof(p));
        snprintf(p.name, sizeof(p.name), "KHR-3HV leumpang");
        p.kind = RobotKind::HUMANOID_KHR;
        homeFromPositions(HOME_POSE, p.home);
        for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) {
            p.joint[b][s] = JOINTS[b][s];
            snprintf(p.jname[b][s], sizeof(p.jname[b][s]), "%s", JOINT_NAMES[b][s]);
            snprintf(p.model[b][s], sizeof(p.model[b][s]), "KRS-2552");
            p.present[b][s] = true;
        }
        init = true;
    }
    return p;
}

// ---------------------------------------------------------------------------
// JSON mini (tanpa alokasi): cukup untuk skema profil.
// ---------------------------------------------------------------------------
namespace {
struct Js {
    const char* s; size_t n, i = 0; char* err; size_t errLen; bool bad = false;
    void fail(const char* m) {
        if (bad) return;
        bad = true;
        if (err && errLen) snprintf(err, errLen, "JSON posisi %u: %s", (unsigned)i, m);
    }
    void ws() { while (i < n && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) ++i; }
    bool eat(char c) { ws(); if (i < n && s[i] == c) { ++i; return true; } return false; }
    char peek() { ws(); return i < n ? s[i] : '\0'; }
    bool str(char* out, size_t cap) {
        ws();
        if (i >= n || s[i] != '"') { fail("diharapkan string"); return false; }
        ++i; size_t k = 0;
        while (i < n && s[i] != '"') {
            char c = s[i++];
            if (c == '\\' && i < n) { c = s[i++]; if (c == 'n') c = ' '; else if (c == 'u') { i += 4; c = '?'; } }
            if (out && k + 1 < cap) out[k++] = c;
        }
        if (out && cap) out[k] = 0;
        if (i >= n) { fail("string tidak ditutup"); return false; }
        ++i; return true;
    }
    bool num(double& v) {
        ws();
        char* e = nullptr;
        char tmp[32]; size_t k = 0;
        while (i + k < n && k < sizeof(tmp) - 1 && strchr("+-0123456789.eE", s[i + k])) { tmp[k] = s[i + k]; ++k; }
        tmp[k] = 0;
        v = strtod(tmp, &e);
        if (k == 0 || e != tmp + k) { fail("diharapkan angka"); return false; }
        i += k; return true;
    }
    bool lit(const char* w) { ws(); const size_t L = strlen(w); if (i + L <= n && !strncmp(s + i, w, L)) { i += L; return true; } return false; }
    void skip() {                                   // lewati nilai apa pun
        const char c = peek();
        if (c == '"') { str(nullptr, 0); return; }
        if (c == '{' || c == '[') {
            const char close = c == '{' ? '}' : ']';
            ++i;
            if (eat(close)) return;
            do { if (c == '{') { str(nullptr, 0); if (!eat(':')) { fail("diharapkan ':'"); return; } } skip(); } while (!bad && eat(','));
            if (!eat(close)) fail("kurung tidak ditutup");
            return;
        }
        if (lit("true") || lit("false") || lit("null")) return;
        double d; num(d);
    }
};

int slotByName(const char* nm) {                   // nama sendi KHR -> bus*16+slot
    for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) if (!strcmp(JOINT_NAMES[b][s], nm)) return b * 16 + s;
    return -1;
}
}  // namespace

bool parseProfile(const char* json, size_t n, RobotProfile& out, char* err, size_t errLen) {
    Js j{json, n, 0, err, errLen};
    char key[24], kind[24] = "humanoid-khr", name[24] = "robot";
    // lintasan 1: cari "kind" & "name" (menentukan nilai awal)
    if (!j.eat('{')) { j.fail("diharapkan objek"); return false; }
    if (!j.eat('}')) do {
        if (!j.str(key, sizeof key) || !j.eat(':')) { j.fail("diharapkan kunci"); return false; }
        if (!strcmp(key, "kind")) j.str(kind, sizeof kind);
        else if (!strcmp(key, "name")) j.str(name, sizeof name);
        else j.skip();
    } while (!j.bad && j.eat(','));
    if (j.bad) return false;

    const bool generic = !strcmp(kind, "generic");
    if (!generic && strcmp(kind, "humanoid-khr")) {
        if (err) snprintf(err, errLen, "kind '%s' tidak dikenal (humanoid-khr / generic)", kind);
        return false;
    }
    out = defaultProfile();
    out.fromSd = true;
    out.kind = generic ? RobotKind::GENERIC : RobotKind::HUMANOID_KHR;
    snprintf(out.name, sizeof(out.name), "%s", name);
    if (generic) {
        for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) {
            out.present[b][s] = false; out.home[b][s] = 0;
            out.joint[b][s] = JointDef{(uint8_t)s, 7500, 1, ICS_POS_MIN, ICS_POS_MAX};
            snprintf(out.jname[b][s], sizeof(out.jname[b][s]), "B%d S%d", b, s);
        }
    }

    // lintasan 2: servos
    j.i = 0; j.eat('{');
    do {
        if (!j.str(key, sizeof key) || !j.eat(':')) return false;
        if (strcmp(key, "servos")) { j.skip(); continue; }
        if (!j.eat('[')) { j.fail("servos harus array"); return false; }
        if (j.eat(']')) continue;
        do {
            if (!j.eat('{')) { j.fail("entri servo harus objek"); return false; }
            double bus = -1, slot = -1, id = -1, center = -1, sign = 0, lo = -1, hi = -1, home = 1e9;
            int present = -1; char nm[16] = "", model[12] = "";
            if (!j.eat('}')) do {
                char k[16]; double v = 0;
                if (!j.str(k, sizeof k) || !j.eat(':')) return false;
                if (!strcmp(k, "name")) j.str(nm, sizeof nm);
                else if (!strcmp(k, "model")) j.str(model, sizeof model);
                else if (!strcmp(k, "present")) { present = j.lit("true") ? 1 : (j.lit("false") ? 0 : -2); if (present == -2) j.fail("present harus true/false"); }
                else if (j.peek() == '"' || j.peek() == '{' || j.peek() == '[' || j.peek() == 'n') j.skip();
                else {
                    if (!j.num(v)) return false;
                    if (!strcmp(k, "bus")) bus = v; else if (!strcmp(k, "slot")) slot = v; else if (!strcmp(k, "id")) id = v;
                    else if (!strcmp(k, "center")) center = v; else if (!strcmp(k, "sign")) sign = v;
                    else if (!strcmp(k, "min")) lo = v; else if (!strcmp(k, "max")) hi = v; else if (!strcmp(k, "home")) home = v;
                }
            } while (!j.bad && j.eat(','));
            if (j.bad || !j.eat('}')) { j.fail("entri servo tidak ditutup"); return false; }
            if ((bus < 0 || slot < 0) && nm[0]) { const int bs = slotByName(nm); if (bs >= 0) { bus = bs / 16; slot = bs % 16; } }
            if (bus < 0 || bus > 1 || slot < 0 || slot >= SERVO_SLOTS) {
                if (err) snprintf(err, errLen, "servo '%s': bus/slot tidak valid (bus 0..1, slot 0..%d)", nm, SERVO_SLOTS - 1);
                return false;
            }
            const int b = (int)bus, s = (int)slot;
            JointDef& d = out.joint[b][s];
            if (id >= 0) d.id = (uint8_t)id;
            if (center >= 0) d.center = (int16_t)center;
            if (sign != 0) d.sign = sign > 0 ? 1 : -1;
            if (lo >= 0) d.lo = (int16_t)lo;
            if (hi >= 0) d.hi = (int16_t)hi;
            if (home < 1e8) out.home[b][s] = (int16_t)home;
            if (nm[0]) snprintf(out.jname[b][s], sizeof(out.jname[b][s]), "%s", nm);
            if (model[0]) snprintf(out.model[b][s], sizeof(out.model[b][s]), "%s", model);
            out.present[b][s] = present == 0 ? false : true;
        } while (!j.bad && j.eat(','));
        if (j.bad || !j.eat(']')) { j.fail("array servos tidak ditutup"); return false; }
    } while (!j.bad && j.eat(','));
    if (j.bad) return false;
    return validateProfile(out, err, errLen);
}

bool validateProfile(const RobotProfile& p, char* err, size_t errLen) {
    for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) {
        if (!p.present[b][s]) continue;
        const JointDef& d = p.joint[b][s];
        const char* nm = p.jname[b][s];
        if (d.id > 31) {
            if (err) snprintf(err, errLen, "%s: id %u > 31", nm, d.id);
            return false;
        }
        if (d.lo < ICS_POS_MIN || d.hi > ICS_POS_MAX || d.lo >= d.hi) {
            if (err) snprintf(err, errLen, "%s: batas %d..%d harus dalam %d..%d", nm, d.lo, d.hi, ICS_POS_MIN, ICS_POS_MAX);
            return false;
        }
        if (d.center < d.lo || d.center > d.hi) {
            if (err) snprintf(err, errLen, "%s: center %d di luar batas", nm, d.center);
            return false;
        }
        const int32_t h = (int32_t)d.center + (int32_t)d.sign * p.home[b][s];
        if (h < d.lo || h > d.hi) {
            if (err) snprintf(err, errLen, "%s: pose HOME (%ld) di luar batas", nm, (long)h);
            return false;
        }
        for (int s2 = s + 1; s2 < SERVO_SLOTS; ++s2)
            if (p.present[b][s2] && p.joint[b][s2].id == d.id) {
                if (err) snprintf(err, errLen, "bus %d: id %u dipakai dua kali (%s, %s)", b, d.id, nm, p.jname[b][s2]);
                return false;
            }
    }
    if (p.humanoid())
        for (int b = 0; b < 2; ++b) for (int s = 1; s < SERVO_SLOTS; ++s)
            if (!p.present[b][s]) {
                if (err) snprintf(err, errLen, "humanoid-khr butuh semua sendi (%s tidak ada)", JOINT_NAMES[b][s]);
                return false;
            }
    return true;
}

size_t writeProfile(const RobotProfile& p, char* buf, size_t len) {
    size_t k = 0;
    auto put = [&](const char* fmt, auto... a) {
        if (k >= len) return;
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
#endif
        const int w = snprintf(buf + k, len - k, fmt, a...);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
        k = (w < 0 || (size_t)w >= len - k) ? len : k + (size_t)w;
    };
    put("{\n  \"name\": \"%s\",\n  \"kind\": \"%s\",\n  \"servos\": [\n", p.name, p.humanoid() ? "humanoid-khr" : "generic");
    bool first = true;
    for (int b = 0; b < 2; ++b) for (int s = 0; s < SERVO_SLOTS; ++s) {
        const JointDef& d = p.joint[b][s];
        put("%s    {\"bus\": %d, \"slot\": %d, \"id\": %u, \"name\": \"%s\", \"model\": \"%s\", \"center\": %d, \"sign\": %d, "
            "\"min\": %d, \"max\": %d, \"home\": %d, \"present\": %s}",
            first ? "" : ",\n", b, s, d.id, p.jname[b][s], p.model[b][s], d.center, d.sign, d.lo, d.hi, p.home[b][s],
            p.present[b][s] ? "true" : "false");
        first = false;
    }
    put("\n  ]\n}\n");
    return k >= len ? 0 : k;
}

}  // namespace la
