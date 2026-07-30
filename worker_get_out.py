def get_out(door_stat,door_pos):
    if door_stat == "open":
        agent_move(door_pos)
        return 1
    elif door_stat == "close":
        info = "door not open"
        return info