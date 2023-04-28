import spacenavigator
import time
import pywinusb.hid as hid 

muie = spacenavigator.list_devices()
nav1 = spacenavigator.open(callback=spacenavigator.print_mouse_R,button_callback=spacenavigator.toggle_led, DeviceNumber=0)
nav2 = spacenavigator.open(callback=spacenavigator.print_mouse_L,button_callback=spacenavigator.toggle_led, DeviceNumber=1)


if nav1 and nav2:
    while 1:
        time.sleep(0.1)
