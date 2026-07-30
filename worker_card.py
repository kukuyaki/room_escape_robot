def card(card_pos,sensor_pos):
    if card_pos == (-1,-1):
        info = "card not exit"
        return info
    if sensor_pos == (-1,-1):
        info = "sensor not exit"
        return info
    target_pos = caculate_move_pos(card_pos)
    agent_move(target_pos)
    agent_grab(card_pos)

    target_pos = caculate_move_pos(sensor_pos)
    agent_move(target_pos)
    agent_tap(sensor_pos)
    return 1
