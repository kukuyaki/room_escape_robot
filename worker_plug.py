def calculate_move_pos():
    print("hi")
    pass
def card(plug_pos,socket_pos):
    if plug_pos == (-1,-1):
        info = "plug not exit"
        return info
    if socket_pos == (-1,-1):
        info = "socket not exit"
        return info

    target_pos = calculate_move_pos(plug_pos)
    agent_move(target_pos)
    agent_grab(card_pos)

    target_pos = caculate_move_pos(socket_pos)
    agent_move(target_pos)
    agent_plugin(socket_pos)
    return 1
