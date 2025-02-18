#!/bin/env python3
# Pelmetcam - software for my raspberry pi powered helmet cam

from GPSController import *
from createDataOverlay import *
import RPi.GPIO as GPIO
from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder
from libcamera import Transform
import time
import datetime
import threading
import os
import argparse

# constants
VIDEOFPS = 25.0
VIDEOWIDTH = 1920
VIDEOHEIGHT = 1080
SLOWFLASHTIMES = [2,2]
FASTFLASHTIMES = [0.5,0.5]
BUTTONSHORTPRESSTICKS = 5
BUTTONLONGPRESSTICKS = 200
BUTTONTICKTIME = 0.01
BUTTONGPIOPIN = 22
LEDGPIOPIN = 17

frame_counter = 0

def frame_callback(request):
    global frame_counter
    frame_counter += 1

# class for managing the Button
class ButtonControl(threading.Thread):
    class ButtonPressStates():
        NOTPRESSED = 0
        SHORTPRESS = 1
        LONGPRESS = 2
    def __init__(self, gpioPin, pressedState, shortPressTicks, longPressTicks, tickTime):
        #setup threading
        threading.Thread.__init__(self)
        #persist data
        self.gpioPin = gpioPin
        self.pressedState = pressedState
        self.shortPressTicks = shortPressTicks
        self.longPressTicks = longPressTicks
        self.tickTime = tickTime
        #init gpio
        GPIO.setup(self.gpioPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
    def get(self):
        return GPIO.input(self.gpioPin)

    def pressed(self):
        #Returns a boolean representing whether the button is pressed
        buttonPressed = False
        # if gpio input is equal to the pressed state
        if GPIO.input(self.gpioPin) == self.pressedState: buttonPressed = True
        return buttonPressed

    def run(self):
        #start the button control
        self.running = True
        self.lastPressedState = self.ButtonPressStates.NOTPRESSED

        # if the button is pressed when the class starts, wait till it is released
        while self.pressed(): time.sleep(self.tickTime)

        # while the control is running
        while self.running:
            # wait for the button to be pressed
            while self.pressed() == False and self.running:
                time.sleep(self.tickTime)

            ticks = 0
            # wait for the button to be released
            while self.pressed() == True and self.running:
                ticks += 1
                time.sleep(self.tickTime)

            #was it press a short or long time    
            if ticks > self.shortPressTicks and ticks < self.longPressTicks:
                self.lastPressedState = self.ButtonPressStates.SHORTPRESS
            if ticks > self.longPressTicks:
                self.lastPressedState = self.ButtonPressStates.LONGPRESS

            #wait in between button presses
            time.sleep(0.5)

    def checkLastPressedState(self):
        #gets the last pressed state but doesnt reset it
        return self.lastPressedState
    
    def getLastPressedState(self):
        #gets the last pressed state and resets it
        theLastPressedState = self.lastPressedState
        self.lastPressedState = self.ButtonPressStates.NOTPRESSED
        return theLastPressedState

    def stopController(self):
        self.running = False

# class for managing LED
class LEDControl():
    class LEDStates():
        #states of LED
        OFF = 0
        ON = 1
        FLASH = 2
        
    def __init__(self, gpioPin):
        #init gpio
        self.gpioPin = gpioPin
	# setup gpio pin as output
        GPIO.setup(self.gpioPin, GPIO.OUT)
        #set led to off
        self.off()

    def set(self, ledValue):
        #Sets the value of the LED [True / False]
        self.ledValue = ledValue
        GPIO.output(self.gpioPin, self.ledValue)

    def get(self):
        #Gets the value of the led
        return self.ledValue

    def on(self):
        #Turns the LED on
        self.state = self.LEDStates.ON
        self.set(True)

    def off(self):
        #Turns the LED off
        self.state = self.LEDStates.OFF
        self.set(False)

    def flash(self, timeOn, timeOff):
        #if the led is already flashing, set it to off and wait for it to stop
        if self.state == self.LEDStates.FLASH:
            self.off()
            self.flashthread.join()
        # flash the LED on a thread
        self.state = self.LEDStates.FLASH
        self.flashthread = threading.Thread(target=self.__flashLED, args=(timeOn, timeOff))
        self.flashthread.start()

    def __flashLED(self, timeOn, timeOff):
        #loops untils the LED is changed from FLASH (i.e. on or off)
        while self.state == self.LEDStates.FLASH:
            if self.get() == True:
                self.set(False)
                time.sleep(timeOff)
            else:
                self.set(True)
                time.sleep(timeOn)
    	
    def toggle(self):
        #Toggles the LED, if its on, turns it off and vice versa
        if self.get == True: self.off()
        else: self.on()

#function for updating the LED based on the GPS Fix state
def updateGPSFixLED(lastFixMode, newFixMode, led):
    #has the fix mode change or the led is not flashing
    if lastFixMode != newFixMode or led.state != led.LEDStates.FLASH:
        #fix mode has changed, change the led and return it
        lastFixMode = newFixMode
        if newFixMode == 1:
            # 1 = no fix - fast flash
            led.flash(FASTFLASHTIMES[0], FASTFLASHTIMES[1])
        else:
            # 2/3 = fix - slow flash
            led.flash(SLOWFLASHTIMES[0], SLOWFLASHTIMES[1])
    return lastFixMode

if __name__ == "__main__":

    #Command line options
    parser = argparse.ArgumentParser(description="Pelmetcam")
    parser.add_argument("path", help="The location of the data directory")
    parser.add_argument("-d", "--dataoverlay", action="store_true", help="Output data overlay images at runtime")
    #parser.add_argument("-i", "--image-type", action="store_true", help="Output image type: <j - JPEG, p - PNG>")
    args = parser.parse_args()
    
    try:

        print ("Starting pi powered helmet cam")
        print ("Data path - " + args.path)
        print ("Data overlay - " + str(args.dataoverlay))
        
        #set gpio mode
        GPIO.setmode(GPIO.BCM)

        #create LED
        led = LEDControl(LEDGPIOPIN)
                           
        #create button
        button = ButtonControl(BUTTONGPIOPIN, 0, BUTTONSHORTPRESSTICKS, BUTTONLONGPRESSTICKS, BUTTONTICKTIME)
        button.start()
        print ("Button - started controller")

        #start gps controller
        gpscontrol = GpsController()
        gpscontrol.start()
        print ("GPS - started controller")

        #start temp sensor controller, 3 second refresh
        #tempcontrol = TempSensorController(TEMPSENSORID, 3)
        #tempcontrol.start()
        #print ("Temp - started controller")

        #get the current fix and set the LED status
        gpsFixMode = updateGPSFixLED(0, gpscontrol.fix.mode, led)
        
        print ("Pelmetcam Ready")

        #while the button hasnt received a long press (shutdown), keep on looping
        while(button.checkLastPressedState() != button.ButtonPressStates.LONGPRESS):
            
            #update gps fix mode and led, each repetition
            gpsFixMode = updateGPSFixLED(gpsFixMode, gpscontrol.fix.mode, led)
            
            #has the button been pressed for recording?
            if (button.checkLastPressedState() == button.ButtonPressStates.SHORTPRESS):
                
                #get the last pressed state of the button and reset it
                button.getLastPressedState()

                #create data folder
                # can I get a time from GPS?
                currenttime = gpscontrol.fixdatetime
                # if not, use the system time
                if(currenttime == None):
                    currenttime = datetime.datetime.now()
                foldername = args.path + "/" + "{0:02d}".format(currenttime.year) + "{0:02d}".format(currenttime.month) + "{0:02d}".format(currenttime.day) + "{0:02d}".format(currenttime.hour) + "{0:02d}".format(currenttime.minute) + "{0:02d}".format(currenttime.second)
                if not os.path.exists(foldername): os.makedirs(foldername)
                print ("Data - folder created - " + foldername)
                
                #create data file
                datafile = open(foldername+"/data.csv", "w")
                
                #create data overlay drawer class
                if args.dataoverlay: datadrawer = DataDrawer(foldername)
                
                #start recording
                #create picamera - dont use LED (my fork of picamera supports this)
                with Picamera2() as camera:
                    #turn LED on
                    led.on()

                    #setup camera
                    camera.post_callback = frame_callback
                    camera.video_configuration.controls.FrameRate = VIDEOFPS
                    video_config = camera.create_video_configuration(main={"size": (VIDEOWIDTH, VIDEOHEIGHT)}, lores={"size": (320, 240)}, display="lores")
                    camera.configure (video_config)
                    #camera.set_controls ({"AeEnable": True, "AeExposureMode": controls.AeConstraintModeEnum.Short, "AfMode": controls.AeExposureModeEnum.Auto})
                    camera.set_controls ({"AeEnable": True})

                    camera.start_preview(Preview.DRM,
                        x=0, y=0,
                        width=320, height=240,
                        transform=Transform(hflip=1, vflip=0))

                    #camera.video_configuration.controls.FrameRate = VIDEOFPS
                    #camera.video_configuration.controls.FrameRate = 25.0
                    #camera.video_stabilization = True
                    
                    #start recording
                    print ("Recorder: Started")
                    #encoder = H264Encoder(bitrate=10000000)
                    #output = foldername + "/video.h264"
                    #camera.start_recording(encoder, output)
                    output = foldername + "/video.mp4"
                    camera.start_and_record_video(output)

                    # wait for the button to be pressed
                    while(button.checkLastPressedState() == button.ButtonPressStates.NOTPRESSED):
                        #get frame number
                        #framenumber = camera.frame
                        #framenumber = 9
                        #wait for a bit, the GPS data is a little behind + give the processor a rest
                        time.sleep(0.1)
                        #record data
                        #dataString = str(framenumber) + "," 
                        dataString = str(frame_counter) + "," 
                        dataString += str(gpscontrol.fix.mode) + "," 
                        dataString += str(gpscontrol.fixdatetime) + "," 
                        dataString += str(gpscontrol.fix.time) + "," 
                        dataString += str(gpscontrol.fix.latitude) + "," 
                        dataString += str(gpscontrol.fix.longitude) + ","
                        dataString += str(gpscontrol.fix.altitude) + ","
                        dataString += str(gpscontrol.fix.speed) + ","
                        dataString += str(gpscontrol.fix.track) + ","
                        dataString += str(gpscontrol.fix.climb) + ","
                        dataString += str(0.0) + ","
                        dataString += str(0.0) + "\n"
                        datafile.write(dataString)
                        #debug, print (data to screen
                        #print(dataString)
                        if args.dataoverlay:
                            dataitems = dataString.split(",")
                            #create frame
                            # newDataFrame(self, frameNo, mode, date, lat, lon, altitude speed, track, climb, tempC)
                            datadrawer.newDataFrame(int(dataitems[0]),
                                                    int(dataitems[1]),
                                                    dataitems[2],
                                                    float(dataitems[4]),                    
                                                    float(dataitems[5]),
                                                    float(dataitems[6]),
                                                    float(dataitems[7]),
                                                    float(dataitems[8]),
                                                    float(dataitems[9]),
                                                    float(dataitems[10]))

                    #stop the camera
                    camera.stop_recording()
                    camera.close()

                    #turn led off
                    led.off()

                    #recording has finished
                    print ("Recorder: Stopped")
                
                print(f"Total frames recorded: {frame_counter}")

                #close data file
                datafile.close()

                #if the button was short pressed, reset the button press
                # meaning if it was long pressed, the program will finish
                if (button.checkLastPressedState() == button.ButtonPressStates.SHORTPRESS):
                    #get the last pressed state and reset it
                    button.getLastPressedState()
            
            #wait for a bit
            time.sleep(0.1)
            #debug, dots to see code is running
            #print ("."
        
    except KeyboardInterrupt:
        print ("User Cancelled (Ctrl C)")
        
    except:
        print ("Unexpected error - ", sys.exc_info()[0], sys.exc_info()[1])
        raise
    
    finally:
        print ("Stopping Pelmetcam")
        #shutdown temp controller
        #tempcontrol.stopController()
        #tempcontrol.join()
        #print ("Temp - Stopped controller")
        #shutdown gps controller
        gpscontrol.stopController()
        gpscontrol.join()
        print ("GPS - Stopped controller")
        #stop button
        button.stopController()
        button.join()
        print ("Button - Stopped controller")
        #turn off led
        led.off()
        #cleanup gpio
        GPIO.cleanup()
        print ("Stopped")
