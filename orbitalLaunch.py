import krpc
from time import sleep
from threading import Thread
from CircleQueue import CircleQueue
from math import pi
from PID import PID

def main():
    try:
        conn = krpc.connect(name="Orbital launch control")
        vessel = conn.space_center.active_vessel

        global CLOCKFREQUENCY , hasAborted , inFlight , targetPitch
        CLOCKFREQUENCY = 0.25
        targetPitch = CircleQueue(2)
        hasAborted = CircleQueue(1)
        inFlight = CircleQueue(1)

        rollThread = Thread(target = rollProgram , args=(vessel,))
        pitchAngleThread = Thread(target = calcPitchAngle , args=(vessel,))
        gravTurnThread = Thread(target = gravTurn , args=(vessel,))
        abortThread = Thread(target = monitorAbort , args=(vessel,))

        hasAborted.enqueue(False)
        inFlight.enqueue(False)

        vessel.control.sas = True
        vessel.control.throttle = 1
        vessel.control.activate_next_stage()
        inFlight.enqueue(True)
        sleep(CLOCKFREQUENCY)
        vessel.control.activate_next_stage()
        pitchAngleThread.start()
        sleep(CLOCKFREQUENCY)
        abortThread.start()
        sleep(5)
        monitorStaging(vessel)
        rollThread.start()
        rollThread.join()
        gravTurnThread.start()
        abortThread.join()
        pitchAngleThread.join()
        gravTurnThread.join()
        abortContigencys(vessel)
        
    except (KeyboardInterrupt):
        #TODO fix this exeption not being recognized instantly in other threads
        print("Ending launch sequence")
        inFlight.enqueue(False)

        if (rollThread.is_alive() and vessel.met > 7):
            rollThread.join()

        if (pitchAngleThread.is_alive() and vessel.met > 0.25):
            pitchAngleThread.join()
        
        if (gravTurnThread.is_alive() and vessel.met > 7):
            gravTurnThread.join()

        if (abortThread.is_alive() and vessel.met > 1):
            abortThread.join()

def monitorAbort(vessel):
    while (vessel.flight().surface_altitude < 50000 and inFlight.peek()):
        if ((targetPitch.peek() - 5)  > vessel.flight().pitch or (vessel.flight().pitch > (targetPitch.peek() + 5))):
            print("Aborted")
            hasAborted.enqueue(True)
            inFlight.enqueue(False)
            vessel.control.abort = True
            break

def rollProgram(vessel):
    rollPid = PID(0.15 , 0.1 , 0.05 , CLOCKFREQUENCY , -30.0)
    #Updates vessel's roll from default 0 degrees
    vessel.control.roll = -1

    while (not(-29 < vessel.flight().roll < -31) and ((not hasAborted.peek()) and (inFlight.peek()))):
        output = rollPid.updateOutput(vessel.flight().roll)
        output = rollPid.applyLimits(-1 , 1)
        output = rollPid.applyDeadzone(0.5 , vessel.flight().roll)
        vessel.control.roll = output
        sleep(CLOCKFREQUENCY)

    vessel.control.roll = 0

def gravTurn(vessel):
    turnPID = PID(0.25 , 0.15 , 0.1 , CLOCKFREQUENCY , targetPitch.peek())
    while((vessel.orbit.apoapsis_altitude < 100000) and ((not hasAborted.peek()) and (inFlight.peek()))):
        output = turnPID.updateOutput(vessel.flight().pitch)
        output = turnPID.applyLimits(-1 , 1)
        output = turnPID.applyDeadzone(0.5 , vessel.flight().pitch)
        vessel.control.pitch = output
        print(output)
        sleep(CLOCKFREQUENCY)

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

    while (inFlight.peek() and (not hasAborted.peek())):
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
        slope += 85
        if (slope <= 0):
            slope = 0
        targetPitch.enqueue(slope)
        
        sleep(CLOCKFREQUENCY)

def abortContigencys(vessel):
    if((hasAborted.peek()) and vessel.orbit.periapsis_altitude < 70000):
        while (vessel.flight().vertical_speed > 0):
            continue

        vessel.control.activate_next_stage()
        vessel.control.sas = False

        while (vessel.flight().surface_altitude > 3000):
            continue

        vessel.control.activate_next_stage()

        while (vessel.flight().surface_altitude > 2000):
            continue

        vessel.control.activate_next_stage()

if (__name__ == "__main__"):
    main()
    quit()