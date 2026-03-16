import inputs

while True:
    events = inputs.get_gamepad()
    for e in events:
        print(e.ev_type, e.code, e.state)