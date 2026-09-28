#!/usr/bin/env python

'''
script to create ap_romfs_embedded.h from a set of static files

Andrew Tridgell
May 2017
'''

import os, sys, zlib

def write_encode(out, s):
    out.write(s.encode())

def strip_lua(src):
    '''remove comments and indentation from Lua source, keeping every newline
    so line numbers in runtime errors still match the original file'''
    out = []
    i, n = 0, len(src)

    def long_bracket(j):
        # level of a long bracket opening at j ("[[", "[=[", ...), or -1
        if j >= n or src[j] != '[':
            return -1
        k = j + 1
        while k < n and src[k] == '=':
            k += 1
        return k - j - 1 if k < n and src[k] == '[' else -1

    if src.startswith('#'):
        # first-line shebang, skipped by the Lua loader: keep it verbatim
        e = src.find('\n')
        e = n if e < 0 else e
        out.append(src[:e])
        i = e
    while i < n:
        c = src[i]
        if c == '-' and src.startswith('--', i):
            lvl = long_bracket(i + 2)
            if lvl >= 0:
                close = ']' + '=' * lvl + ']'
                e = src.find(close, i + 4 + lvl)
                if e < 0:
                    raise ValueError('unterminated long comment')
                body = src[i:e + len(close)]
                # a comment is whitespace to Lua: keep its newlines, or one space
                out.append('\n' * body.count('\n') if '\n' in body else ' ')
                i = e + len(close)
            else:
                e = src.find('\n', i)
                i = n if e < 0 else e
        elif c == '[' and long_bracket(i) >= 0:
            lvl = long_bracket(i)
            close = ']' + '=' * lvl + ']'
            e = src.find(close, i + 2 + lvl)
            if e < 0:
                raise ValueError('unterminated long string')
            out.append(src[i:e + len(close)])
            i = e + len(close)
        elif c in '"\'':
            j = i + 1
            while j < n and src[j] != c:
                if src[j] == '\\':
                    j += 1
                    if src.startswith('\r\n', j):
                        j += 1
                    elif j < n and src[j] == 'z':
                        # \z skips the whitespace after it, newlines included
                        while j + 1 < n and src[j + 1] in ' \t\r\n\f\v':
                            j += 1
                elif src[j] == '\n':
                    raise ValueError('unterminated string')
                j += 1
            if j >= n:
                raise ValueError('unterminated string')
            out.append(src[i:j + 1])
            i = j + 1
        elif c == '\n':
            # drop trailing whitespace before the newline and indentation after it
            while out and out[-1] in (' ', '\t'):
                out.pop()
            out.append('\n')
            i += 1
            while i < n and src[i] in ' \t':
                i += 1
        else:
            out.append(c)
            i += 1
    return ''.join(out)

def embed_file(out, f, idx, embedded_name, uncompressed):
    '''embed one file'''
    try:
        contents = open(f,'rb').read()
    except Exception:
        raise Exception("Failed to embed %s" % f)

    if embedded_name.endswith(".lua"):
        # AP_ROMFS decompresses a file whole into one malloc on open, and a
        # 137KB LTE_modem.lua failed that allocation (ENOENT) on a Pixhawk6C.
        # Comments and indentation don't change the compiled script.
        try:
            stripped = strip_lua(contents.decode('utf-8', 'surrogateescape')).encode('utf-8', 'surrogateescape')
            print("Stripped %s: %u -> %u bytes" % (embedded_name, len(contents), len(stripped)))
            contents = stripped
        except ValueError as e:
            print("Not stripping %s: %s" % (embedded_name, e))

    if embedded_name.endswith("bootloader.bin"):
        # round size to a multiple of 32 bytes for bootloader, this ensures
        # it can be flashed on a STM32H7 chip
        blen = len(contents)
        pad = (32 - (blen % 32)) % 32
        if pad != 0:
            contents += bytes([0xff]*pad)
            print("Padded %u bytes for %s to %u" % (pad, embedded_name, len(contents)))

    crc = crc32(contents)
    write_encode(out, '__EXTFLASHFUNC__ static const uint8_t ap_romfs_%u[] = {' % idx)

    if uncompressed:
        # terminate if there's not already an existing null. we don't add it to
        # the contents to avoid storing the wrong length
        null_terminate = 0 not in contents
        b = contents
    else:
        # compress it (max level, max window size, raw stream, max mem usage)
        z = zlib.compressobj(level=9, method=zlib.DEFLATED, wbits=-15, memLevel=9)
        b = z.compress(contents)
        b += z.flush()
        # decompressed data will be null terminated at runtime, nothing to do here
        null_terminate = False

    write_encode(out, ",".join(str(c) for c in b))
    if null_terminate:
        write_encode(out, ",0")
    write_encode(out, '};\n\n');
    return crc, len(contents)

def crc32(bytes, crc=0):
    '''crc32 equivalent to crc32_small() from AP_Math/crc.cpp'''
    for byte in bytes:
        crc ^= byte
        for i in range(8):
            mask = (-(crc & 1)) & 0xFFFFFFFF
            crc >>= 1
            crc ^= (0xEDB88320 & mask)
    return crc

def create_embedded_h(filename, files, uncompressed=False):
    '''create a ap_romfs_embedded.h file'''

    out = open(filename, "wb")
    write_encode(out, '''// generated embedded files for AP_ROMFS\n\n''')

    # remove duplicates and sort
    files = sorted(list(set(files)))
    crc = {}
    decompressed_size = {}
    for i in range(len(files)):
        (name, filename) = files[i]
        try:
            crc[filename], decompressed_size[filename] = embed_file(out, filename, i, name, uncompressed)
        except Exception as e:
            print(e)
            return False

    write_encode(out, '''const AP_ROMFS::embedded_file AP_ROMFS::files[] = {\n''')

    for i in range(len(files)):
        (name, filename) = files[i]
        if uncompressed:
            ustr = ' (uncompressed)'
        else:
            ustr = ''
        print("Embedding file %s:%s%s" % (name, filename, ustr))
        write_encode(out, '{ "%s", sizeof(ap_romfs_%u), %d, 0x%08x, ap_romfs_%u },\n' % (
            name, i, decompressed_size[filename], crc[filename], i))
    write_encode(out, '};\n')
    out.close()
    return True

if __name__ == '__main__':
    import sys
    flist = []
    for i in range(1, len(sys.argv)):
        f = sys.argv[i]
        flist.append((f, f))
    create_embedded_h("/tmp/ap_romfs_embedded.h", flist)
