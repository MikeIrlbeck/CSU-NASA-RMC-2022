#USB Xbox360 controller on LINUX
# CSU NASAmining: Dylan Clem
# step one, install xbox360 driver and then python 3 driver
#pip3 install -U xbox360controller
#sudo apt install python3-pip
#https://github.com/linusg/xbox360controller/blob/master/docs/API.md#xbox360controller-parameters
#The code will run until Ctrl+C is presses.
#Each time on of the left or right axis is moved, the event will be processed.
#Additionally, the events of the A button are being processed.

import signal
from xbox360controller import Xbox360Controller


def on_button_pressed(button):
    print('Button {0} was pressed'.format(button.name))


def on_button_released(button):
    print('Button {0} was released'.format(button.name))


def on_axis_moved(axis):
    print('Axis {0} moved to {1} {2}'.format(axis.name, axis.x, axis.y))

try:
    with Xbox360Controller(0, axis_threshold=0.2) as controller:
        # Button A events
        controller.button_a.when_pressed = on_button_pressed
        controller.button_a.when_released = on_button_released

        # Left and right axis move event
        controller.axis_l.when_moved = on_axis_moved
        controller.axis_r.when_moved = on_axis_moved

        signal.pause()
except KeyboardInterrupt:
    pass
