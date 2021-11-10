## USB Controller Python Integration
#NASA Mining Team: Dylan Clem

#import evdev on Latte Panda
#https://core-electronics.com.au/tutorials/using-usb-and-bluetooth-controllers-with-python.html

from evdev import InputDevice, categorize, ecodes

#creates object 'gamepad' to store the data
#you can call it whatever you like
gamepad = InputDevice('/dev/input/event3')

#button code variables (change to suit your device)
aBtn = 34
bBtn = 36
xBtn = 35
yBtn = 23

up = 46
down = 32
left = 18
right = 33

start = 24
select = 49

lTrig = 37
rTrig = 50

#prints out device info at start
print(gamepad)

#loop and filter by event code and print the mapped label
for event in gamepad.read_loop():
    if event.type == ecodes.EV_KEY:
        if event.value == 1:
            if event.code == yBtn:
                print("Y")
            elif event.code == bBtn:
                print("B")
            elif event.code == aBtn:
                print("A")
            elif event.code == xBtn:
                print("X")

            elif event.code == up:
                print("up")
            elif event.code == down:
                print("down")
            elif event.code == left:
                print("left")
            elif event.code == right:
                print("right")

            elif event.code == start:
                print("start")
            elif event.code == select:
                print("select")

            elif event.code == lTrig:
                print("left bumper")
            elif event.code == rTrig:
                print("right bumper")
