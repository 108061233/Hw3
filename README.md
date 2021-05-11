First, "cd ~/ee2405/Hw3/final_code", change the ip address and wifi hotspot, 
and "sudo mbed compile --source . --source ~/ee2405/mbed-os/ -m B_L4S5I_IOT01A -t GCC_ARM --profile tflite.json -f" to compile.

Second, "sudo screen /dev/ttyACM0" to open screen and open another terminal to final_code and then "sudo python3 wifi_mqtt/mqtt_client.py".

Third, after the set finish, input "/UI/run 1" move to gesture UI mode, in gesture UI mode the LED3 will blink and you can use gesture ring: angle + 20
(move B_L4S5I_IOT01A with a circle), slope: angle = 40(move B_L4S5I_IOT01A with a tilt L), line: angle = 60(move B_L4S5I_IOT01A from left to right), angle
will show on LCD and you can use USER BUTTON to select bound angle, then gesture UI mode will terminate and angle will show on python.

Fourth, input "/tilt/run 1" move to tilt detection mode, in this mode, LED2 will blink, you can randomly move B_L4S5I_IOT01A and angle will show on LCD,
if  B_L4S5I_IOT01A is static LED1 will be bright, after angle is over bound angle you choose more than 10 times, the mode will terminate and angle will show
on python.

Note1: gesture is not sensitive, the ring will appear many times, so you may not get slope and line easily.
Note2: if you do nothing in screen too long, it may be no react, please restart from second step.
