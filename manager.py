def manager(image, xylist):
    while 1:
        if xylist["plug"] and xylist["socket"]:
            worker_plug()
        if xylist["card"] and xylist["sensor"]:
            worker_card()
        if xylist["door"] == "open":
            worker_get_out()
        if success():
            break
    print("finish!!!!!!!!!!!!!!!")
    pass