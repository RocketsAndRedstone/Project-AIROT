import krpc
from time import sleep
from threading import Thread
from CircleQueue import CircleQueue
from math import pi
from PID import PID

def main():
    conn = krpc.connect(name="Orbital launch control")
    vessel = conn.space_center.active_vessel

    global hasAborted , inFlight , targetPitch
    targetPitch = CircleQueue(2)
    hasAborted = CircleQueue(1)
    inFlight = CircleQueue(1)

    rollThread = Thread(target = rollProgram , args=(vessel,))
    pitchAngleThread = Thread(target = calcPitchAngle , args=(vessel,))
    abortThread = Thread(target = monitorAbort , args=(vessel,))

    hasAborted.enqueue(False)
    inFlight.enqueue(False)

    vessel.control.sas = True
    vessel.control.throttle = 1
    vessel.control.activate_next_stage()
    inFlight.enqueue(True)
    sleep(0.25)
    vessel.control.activate_next_stage()
    pitchAngleThread.start()
    sleep(0.25)
    abortThread.start()
    sleep(5)
    monitorStaging(vessel)
    rollThread.start()
    rollThread.join()
    abortThread.join()
    pitchAngleThread.join()

def monitorAbort(vessel):
    while (vessel.flight().surface_altitude < 50000):
        if ((targetPitch.peek() - 5)  > vessel.flight().pitch or (vessel.flight().pitch > (targetPitch.peek() + 5))):
            print("Aborted")
            hasAborted.enqueue(True)
            inFlight.enqueue(False)
            vessel.control.abort = True
            break

def rollProgram(vessel):
    loopTime = 0.25
    rollPid = PID(0.25 , 0.25 , 0.25 , loopTime , -90.0)
    #Updates vessel's roll from default 0 degrees
    vessel.control.roll = -1

    #figure out better condition for roll PID end
    while (( -89 < vessel.flight().roll < -91) and (not hasAborted.peek()) and (inFlight.peek())):
        output = rollPid.updateOutput(vessel.flight().roll)
        output = rollPid.applyLimits(-1 , 1)
        output = rollPid.applyDeadzone(0.5)
        vessel.control.roll = output
        sleep(loopTime)

def gravTurn(vessel):
    #TODO add logic for smooth and consistant grav turn
    pass

def headingLock(vessel):
    #TODO add logic to follow a set azumith using yaw for proper orbital insertion
    pass

def monitorStaging(vessel):
    #TODO add logic for monitoring when to activate staging
    pass

def calcPitchAngle(vessel):
    kerbinRadius = 600000
    startLatitude = vessel.flight().latitude
    startLongitude = vessel.flight().longitude

    #fligtht trajectory equation: f(x) = -0.001x^2 , f'(x) = -0.002x

    while (inFlight.peek() and not hasAborted.peek()):
        relativeLatitude = abs(abs(startLatitude) - abs(vessel.flight().latitude))
        relativeLongitude = abs(abs(startLongitude) - abs(vessel.flight().longitude))
        
        #degrees to radians
        relativeLatitude = relativeLatitude * (pi / 180)
        relativeLongitude = relativeLongitude * (pi / 180)
        
        #radians to meters
        relativeLatitude = relativeLatitude * kerbinRadius
        relativeLongitude = relativeLongitude * kerbinRadius

        downrangeDistance = ((relativeLatitude ** 2) + (relativeLongitude ** 2)) ** 0.5
        
        slope = -0.002 * downrangeDistance
        slope = round(slope , 3)
        slope += 90
        if (slope <= 0):
            slope = 0
        targetPitch.enqueue(slope)
        
        sleep(0.25)

if (__name__ == "__main__"):
    main()
    quit()