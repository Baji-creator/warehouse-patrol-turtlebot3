import ctypes
import os
import re
import subprocess
import time

os.environ['DISPLAY'] = ':0'
os.environ['XAUTHORITY'] = '/run/user/1000/gdm/Xauthority'
rundir = open('/tmp/visual_final_dir').read().strip()
xlib = ctypes.CDLL('libX11.so.6')
xlib.XOpenDisplay.restype = ctypes.c_void_p
xlib.XOpenDisplay.argtypes = [ctypes.c_char_p]
xlib.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
xlib.XFlush.argtypes = [ctypes.c_void_p]
display = xlib.XOpenDisplay(b':0')
class ClientMessage(ctypes.Structure):
    _fields_ = [('type', ctypes.c_int), ('serial', ctypes.c_ulong),
                ('send_event', ctypes.c_int), ('display', ctypes.c_void_p),
                ('window', ctypes.c_ulong), ('message_type', ctypes.c_ulong),
                ('format', ctypes.c_int), ('data', ctypes.c_long * 5)]

class Event(ctypes.Union):
    _fields_ = [('client', ClientMessage), ('padding', ctypes.c_long * 24)]

xlib.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
xlib.XDefaultRootWindow.restype = ctypes.c_ulong
xlib.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
xlib.XInternAtom.restype = ctypes.c_ulong
xlib.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                           ctypes.c_long, ctypes.POINTER(Event)]
root = xlib.XDefaultRootWindow(display)

def send(window, message, values):
    event = Event()
    event.client.type = 33
    event.client.display = display
    event.client.window = window
    event.client.message_type = xlib.XInternAtom(display, message, 0)
    event.client.format = 32
    for index, value in enumerate(values):
        event.client.data[index] = value
    xlib.XSendEvent(display, root, 0, (1 << 20) | (1 << 19), ctypes.byref(event))

tree = subprocess.check_output(['xwininfo', '-root', '-tree'])
for name, pattern in [('gazebo', r'(0x[0-9a-f]+) "Gazebo"'),
                      ('rviz', r'(0x[0-9a-f]+) "warehouse_patrol.rviz[^"]* - RViz"')]:
    match = re.search(pattern, tree)
    if not match:
        print('MISSING_WINDOW ' + name)
        continue
    window = int(match.group(1), 16)
    send(window, b'_NET_WM_STATE', [1,
         xlib.XInternAtom(display, b'_NET_WM_STATE_MAXIMIZED_HORZ', 0),
         xlib.XInternAtom(display, b'_NET_WM_STATE_MAXIMIZED_VERT', 0), 1])
    send(window, b'_NET_ACTIVE_WINDOW', [1, 0])
    xlib.XRaiseWindow(display, window)
    xlib.XFlush(display)
    time.sleep(3)
    subprocess.call(['gnome-screenshot', '-f', os.path.join(rundir, name + '_desktop.png')])
    subprocess.call(['import', '-window', match.group(1), os.path.join(rundir, name + '_window.png')])
    print('CAPTURED ' + name)
