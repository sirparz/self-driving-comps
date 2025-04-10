import struct

JS_EVENT_BUTTON = 0x01  # Button pressed/released
JS_EVENT_AXIS   = 0x02  # Axis moved
JS_EVENT_INIT   = 0x80  # Initial state of device

# Open joystick device
with open('/dev/input/js0', 'rb') as js:
    print("Listening to /dev/input/js0 ... (Ctrl+C to quit)")

    while True:
        evbuf = js.read(8)
        if evbuf:
            time, value, type_raw, number = struct.unpack('IhBB', evbuf)
            event_type = type_raw & ~JS_EVENT_INIT

            if event_type == JS_EVENT_AXIS:
                print(f"[AXIS {number:02}] Value: {value:6}")
            elif event_type == JS_EVENT_BUTTON:
                state = "Pressed" if value else "Released"
                print(f"[BUTTON {number:02}] {state}")