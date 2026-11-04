DEADZONE = 0.35


def controller_commands(axes):
    values = list(axes[:4])
    values.extend([0.0] * (4 - len(values)))
    left_x, left_y, right_x, right_y = values
    commands = set()

    if left_x < -DEADZONE:
        commands.add(b"L")
    elif left_x > DEADZONE:
        commands.add(b"R")
    if left_y < -DEADZONE:
        commands.add(b"U")
    elif left_y > DEADZONE:
        commands.add(b"D")

    if right_x < -DEADZONE:
        commands.add(b"a")
    elif right_x > DEADZONE:
        commands.add(b"d")
    if right_y < -DEADZONE:
        commands.add(b"w")
    elif right_y > DEADZONE:
        commands.add(b"s")

    return commands
