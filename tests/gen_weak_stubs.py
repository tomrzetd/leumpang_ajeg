#!/usr/bin/env python3
"""Hasilkan definisi weak (return 0) untuk semua fungsi di tests/idf_stubs/idf_stub.h.
Dipakai untuk link-check firmware di host (menangkap simbol hilang antar-modul)."""
import re, sys
src = open(sys.argv[1]).read()
src = re.sub(r'//.*', '', src)
src = '\n'.join(l for l in src.split('\n') if not l.strip().startswith('#') and l.strip() not in ('extern "C" {', '}'))
prev = None
while prev != src:
    prev = src
    src = re.sub(r'\{[^{}]*\}', '', src)
out = ['#include "idf_stub.h"', 'extern "C" {']
for piece in src.split(';'):
    p = ' '.join(piece.split())
    if '(' not in p or p.startswith(('typedef', 'extern', 'enum', 'union', 'struct')) or '(*' in p:
        continue
    m = re.match(r'(.*?)(\w+)\s*\((.*)\)$', p)
    if not m:
        continue
    ret, name, args = m.group(1).strip(), m.group(2), m.group(3)
    parts = [a.strip() for a in args.split(',')] if args.strip() not in ('', 'void') else []
    named = ', '.join(f'{a} a{i}' for i, a in enumerate(parts))
    body = '' if ret == 'void' else 'return 0;'
    voids = ''.join(f'(void)a{i};' for i in range(len(parts)))
    out.append(f'__attribute__((weak)) {ret} {name}({named}) {{ {voids} {body} }}')
out.append('extern __attribute__((weak)) esp_event_base_t const WIFI_EVENT = "W";')
out.append('extern __attribute__((weak)) esp_event_base_t const IP_EVENT = "I";')
out.append('__attribute__((weak)) void* __dso_handle = nullptr;   // host -nostartfiles')
out.append('}')
open(sys.argv[2], 'w').write('\n'.join(out) + '\n')
