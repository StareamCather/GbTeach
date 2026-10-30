"""Native double clicks confined to the observed, owned editor viewport.

QGraphicsView's mouse-grab state is sensitive to synthetic double-click
sequences. XTest supplies the full press/release sequence used by X11 itself.
No screen coordinate is accepted: points are translated from a live QWidget.
"""
import ctypes
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import time

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tests.ui_probe import gazebo_window_id


def double_click(bridge, identity, x, y):
    rows = bridge.snapshot()
    by = {r['id']: r for r in rows}
    target = by[identity]
    if not target.get('visible') or not (0 <= x < target['rect'][2] and 0 <= y < target['rect'][3]):
        raise ValueError('Native input requires a visible observed point inside the owned widget')
    current, offset_x, offset_y = target, x, y
    while current.get('name') != 'mainWindow':
        if 'geometry' in current:
            offset_x += current['geometry'][0]
            offset_y += current['geometry'][1]
        current = by[current['parent']]
    record = json.loads((bridge.folder / 'session.json').read_text())
    metadata = json.loads((Path(record['runtime_log']).parent / 'session.json').read_text())
    pid = next(p['pid'] for p in metadata['processes'] if '图形' in p['name'])
    window, _ = gazebo_window_id(SimpleNamespace(token=record['token'], gui=SimpleNamespace(pid=pid)))
    xlib, test = ctypes.CDLL('libX11.so.6'), ctypes.CDLL('libXtst.so.6')
    xlib.XOpenDisplay.argtypes = [ctypes.c_char_p]
    xlib.XOpenDisplay.restype = ctypes.c_void_p
    display = xlib.XOpenDisplay(None)
    if not display:
        raise RuntimeError('No active X11 display for owned editor test')
    xlib.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    xlib.XDefaultRootWindow.restype = ctypes.c_ulong
    xlib.XTranslateCoordinates.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong,
                                          ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int),
                                          ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_ulong)]
    xlib.XSetInputFocus.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
    xlib.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    xlib.XFlush.argtypes = [ctypes.c_void_p]
    xlib.XCloseDisplay.argtypes = [ctypes.c_void_p]
    test.XTestFakeMotionEvent.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_ulong]
    test.XTestFakeButtonEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
    try:
        gx, gy, child = ctypes.c_int(), ctypes.c_int(), ctypes.c_ulong()
        if not xlib.XTranslateCoordinates(display, window, xlib.XDefaultRootWindow(display),
                                          round(offset_x), round(offset_y), ctypes.byref(gx),
                                          ctypes.byref(gy), ctypes.byref(child)):
            raise RuntimeError('Owned editor window disappeared before input')
        xlib.XRaiseWindow(display, window)
        xlib.XSetInputFocus(display, window, 2, 0)
        test.XTestFakeMotionEvent(display, -1, gx.value, gy.value, 0)
        xlib.XFlush(display)
        time.sleep(.04)
        for _ in range(2):
            test.XTestFakeButtonEvent(display, 1, 1, 0)
            test.XTestFakeButtonEvent(display, 1, 0, 0)
            xlib.XFlush(display)
            time.sleep(.06)
    finally:
        xlib.XCloseDisplay(display)
