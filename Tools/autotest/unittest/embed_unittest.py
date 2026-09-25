#!/usr/bin/env python3

'''
Unit tests for Tools/ardupilotwaf/embed.py: the Lua comment stripper and the
ROMFS header it feeds.

The strongest check compiles Lua before and after stripping with luac5.3,
debug info kept, and requires identical bytecode: same code, same line
numbers. Those tests are skipped when luac5.3 is not installed.

    python3 Tools/autotest/unittest/embed_unittest.py
'''

import contextlib
import io
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
import warnings
import zlib

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.insert(0, os.path.join(ROOT, 'Tools', 'ardupilotwaf'))
import embed  # noqa: E402
from embed import strip_lua  # noqa: E402

LUAC = shutil.which('luac5.3')
LTE_SCRIPT = os.path.join(ROOT, 'libraries', 'AP_HAL_ChibiOS', 'hwdef', 'Pixhawk6C-bdshot',
                          'scripts', 'LTE_modem.lua')


def luac(src):
    '''bytecode of src with debug info, or None if it does not compile'''
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, 'out.luac')
        r = subprocess.run([LUAC, '-o', out, '-'], input=src.encode('utf-8', 'surrogateescape'),
                           capture_output=True)
        if r.returncode != 0:
            return None
        with open(out, 'rb') as f:
            return f.read()


# name: (source, expected stripped output)
CASES = {
    'line comment': ('local a = 1\n-- a comment\nlocal b = 2\n',
                     'local a = 1\n\nlocal b = 2\n'),
    'trailing comment': ('local a = 1   -- trailing\nreturn a\n',
                         'local a = 1\nreturn a\n'),
    'indentation': ('if true then\n    local x = 1\n\tlocal y = 2\nend\n',
                    'if true then\nlocal x = 1\nlocal y = 2\nend\n'),
    'block comment': ('--[[\nheader\n  lines\n]]\nlocal a = 1\n',
                      '\n\n\n\nlocal a = 1\n'),
    'block comment with level': ('--[==[\n ]] still comment\n]==]\nlocal a = 1\n',
                                 '\n\n\nlocal a = 1\n'),
    # a comment is whitespace: "not--[[c]]x" must not become the name "notx"
    'inline block comment': ('local x = false\nreturn not--[[c]]x\n',
                             'local x = false\nreturn not x\n'),
    'dashes inside strings': ('local a = "x -- not a comment"\nlocal b = \'--[[ nor this\'\n'
                              'local c = "q\\"--q"\n', None),
    'long strings': ('local s = [[\n  -- kept\n    indented\n]]\nlocal t = [=[ ]] ]=]\n', None),
    'backslash newline in string': ('local s = "a\\\n  b"\nreturn s\n', None),
    'backslash crlf in string': ('local s = "a\\\r\n  b"\r\nreturn s\r\n', None),
    'z escape': ('local s = "a\\z\n      b"\nreturn s\n', None),
    'shebang': ('#!/usr/bin/lua\nlocal a = 1 -- c\n', '#!/usr/bin/lua\nlocal a = 1\n'),
    'minus operators': ('local b, c = 3, 4\nlocal a = b - -c\nlocal d = 5 -1\nreturn a, d\n', None),
    'comment at end of file': ('local a = 1 -- no newline at end', 'local a = 1 '),
    'crlf line endings': ('local a = 1 -- c\r\n  local b = 2\r\nreturn a + b\r\n',
                          'local a = 1\nlocal b = 2\r\nreturn a + b\r\n'),
}


class TestStripLua(unittest.TestCase):

    def test_expected_output(self):
        for name, (src, expected) in CASES.items():
            with self.subTest(name):
                # None: nothing to strip, output must equal the input
                self.assertEqual(strip_lua(src), src if expected is None else expected)

    def test_line_count_kept(self):
        for name, (src, _) in CASES.items():
            with self.subTest(name):
                self.assertEqual(strip_lua(src).count('\n'), src.count('\n'))

    def test_unterminated_raises(self):
        for src in ('local s = "abc\nreturn s\n', "local s = 'abc", 'local s = [[abc\n',
                    '--[[ never closed\nlocal a = 1\n', '--[==[ wrong level ]]\n'):
            with self.subTest(src):
                with self.assertRaises(ValueError):
                    strip_lua(src)


@unittest.skipUnless(LUAC, 'luac5.3 not installed')
class TestBytecodeIdentical(unittest.TestCase):

    def test_cases(self):
        for name, (src, _) in CASES.items():
            with self.subTest(name):
                before = luac(src)
                self.assertIsNotNone(before, 'test case is not valid Lua')
                self.assertEqual(luac(strip_lua(src)), before)

    def test_every_lua_file_in_repo(self):
        count = 0
        for dirpath, dirnames, filenames in os.walk(ROOT):
            if dirpath == ROOT:
                # git submodules and build output; Lua "modules" dirs elsewhere are kept
                dirnames[:] = [d for d in dirnames if d not in ('.git', 'modules', 'build')]
            for fn in filenames:
                if not fn.endswith('.lua'):
                    continue
                path = os.path.join(dirpath, fn)
                with open(path, encoding='utf-8', errors='surrogateescape', newline='') as f:
                    src = f.read()
                before = luac(src)
                if before is None:
                    continue    # not valid Lua 5.3 to begin with
                count += 1
                with self.subTest(os.path.relpath(path, ROOT)):
                    self.assertEqual(luac(strip_lua(src)), before)
        self.assertGreater(count, 50, 'expected to find the repo Lua scripts')
        print('\n  %d Lua files compiled identically after stripping' % count, file=sys.stderr)


def embed_and_read_back(files, uncompressed=False):
    '''run create_embedded_h on {name: bytes}; return {name: (data, size, crc)}
    with each file decompressed the way AP_ROMFS does'''
    with tempfile.TemporaryDirectory() as d:
        flist = []
        for i, (name, data) in enumerate(files.items()):
            path = os.path.join(d, 'f%d' % i)
            with open(path, 'wb') as f:
                f.write(data)
            flist.append((name, path))
        header = os.path.join(d, 'ap_romfs_embedded.h')
        # embed_file() reads with a bare open(): hide its ResourceWarning
        with contextlib.redirect_stdout(io.StringIO()), warnings.catch_warnings():
            warnings.simplefilter('ignore', ResourceWarning)
            if not embed.create_embedded_h(header, flist, uncompressed):
                raise RuntimeError('create_embedded_h failed')
        with open(header) as f:
            h = f.read()
    arrays = {int(m.group(1)): bytes(int(x) for x in m.group(2).split(','))
              for m in re.finditer(r'ap_romfs_(\d+)\[\] = \{([^}]*)\};', h)}
    out = {}
    for m in re.finditer(r'\{ "([^"]+)", sizeof\(ap_romfs_(\d+)\), (\d+), 0x([0-9a-f]{8}), ', h):
        name, idx, size, crc = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4), 16)
        raw = arrays[idx]
        data = raw[:size] if uncompressed else zlib.decompress(raw, -15)
        out[name] = (data, size, crc)
    return out


class TestEmbedFile(unittest.TestCase):

    GOOD = b'-- header\nlocal a = 1  -- one\n    return a\n'
    BAD = b'local s = "never closed\n-- comment\n'
    FILES = {
        'scripts/good.lua': GOOD,
        'scripts/bad.lua': BAD,
        'defaults.parm': b'X 1 -- not Lua, left alone\n    Y 2\n',
        'data.bin': bytes(range(256)) * 4,
    }

    def check(self, uncompressed):
        got = embed_and_read_back(self.FILES, uncompressed)
        self.assertEqual(set(got), set(self.FILES))
        expected = dict(self.FILES)
        expected['scripts/good.lua'] = strip_lua(self.GOOD.decode()).encode()
        for name, (data, size, crc) in got.items():
            with self.subTest(name):
                self.assertEqual(data, expected[name])
                self.assertEqual(size, len(expected[name]))
                self.assertEqual(crc, embed.crc32(expected[name]))

    def test_compressed(self):
        self.check(uncompressed=False)

    def test_uncompressed(self):
        self.check(uncompressed=True)

    def test_stripped_lua_is_smaller(self):
        got = embed_and_read_back({'scripts/good.lua': self.GOOD})
        self.assertLess(got['scripts/good.lua'][1], len(self.GOOD))

    def test_bad_lua_falls_back_to_original(self):
        got = embed_and_read_back({'scripts/bad.lua': self.BAD})
        self.assertEqual(got['scripts/bad.lua'][0], self.BAD)


@unittest.skipUnless(os.path.exists(LTE_SCRIPT), 'LTE_modem.lua not in this tree')
class TestLteModemScript(unittest.TestCase):
    '''the file that failed to open from ROMFS at 137 KB (ENOENT)'''

    def setUp(self):
        with open(LTE_SCRIPT, 'rb') as f:
            self.src = f.read()
        self.got = embed_and_read_back({'scripts/LTE_modem.lua': self.src})['scripts/LTE_modem.lua']

    def test_embedded_size(self):
        # rc2 opened fine at 112 KB; stay well under it
        self.assertLess(self.got[1], 100000)

    def test_line_count_kept(self):
        self.assertEqual(self.got[0].count(b'\n'), self.src.count(b'\n'))

    @unittest.skipUnless(LUAC, 'luac5.3 not installed')
    def test_bytecode_identical(self):
        self.assertEqual(luac(self.got[0].decode('utf-8', 'surrogateescape')),
                         luac(self.src.decode('utf-8', 'surrogateescape')))


if __name__ == '__main__':
    unittest.main()
