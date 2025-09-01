import krpc
from time import sleep
from threading import Thread , Event
from signal import signal , SIGINT
from CircleQueue import CircleQueue
from math import pi , cos
from PID import PID

def main():
    conn = krpc.connect(name="Orbital launch control")
    vessel = conn.space_center.active_vessel

    global CLOCKFREQUENCY , hasAborted , inFlight , targetPitch , interuptEvent , signal
    CLOCKFREQUENCY = 0.25
    targetPitch = CircleQueue(2)
    hasAborted = CircleQueue(1)
    inFlight = CircleQueue(1)
    
    interuptEvent = Event()
    signal(SIGINT , handler)

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
    staging(vessel , 6)

    rollThread.start()
    rollThread.join()

    gravTurnThread.start()

    abortThread.join()
    
    pitchAngleThread.join()
    gravTurnThread.join()
    
    abortContigencys(vessel)
    abortThread.join()

def monitorAbort(vessel):
    while (vessel.flight().surface_altitude < 50000 and inFlight.peek()):
        if ((targetPitch.peek() - 5)  > vessel.flight().pitch or (vessel.flight().pitch > (targetPitch.peek() + 5))):
            if(interuptEvent.is_set()):
                break
            print("Aborted")
            hasAborted.enqueue(True)
            inFlight.enqueue(False)
            vessel.control.abort = True
            break

def rollProgram(vessel):
    rollPid = PID(0.15 , 0.1 , 0.05 , CLOCKFREQUENCY , -30.0)
    #Updates vessel's roll from default 0 degrees
    vessel.control.roll = -1

    while (not(-29 > vessel.flight().roll > -31) and ((not hasAborted.peek()) and (inFlight.peek()))):
        if(interuptEvent.is_set()):
            break
        output = rollPid.updateOutput(vessel.flight().roll)
        output = rollPid.applyLimits(-1 , 1)
        output = rollPid.applyDeadzone(0.5 , vessel.flight().roll)
        vessel.control.roll = output
        sleep(CLOCKFREQUENCY)

    vessel.control.roll = 0

def gravTurn(vessel):
    turnPID = PID(0.25 , 0.15 , 0.1 , CLOCKFREQUENCY , targetPitch.peek())

    while((vessel.orbit.apoapsis_altitude < 100000) and ((not hasAborted.peek()) and (inFlight.peek()))):
        if(interuptEvent.is_set()):
            break
        turnPID.updateTarget(targetPitch.peek())
        output = turnPID.updateOutput(vessel.flight().pitch)
        output = turnPID.applyLimits(-1 , 1)
        output = turnPID.applyDeadzone(0.5 , vessel.flight().pitch)
        vessel.control.pitch = output
        #print(output)
        sleep(CLOCKFREQUENCY)
    
    vessel.control.pitch = 0

def headingLock(vessel):
    headingPID = PID(0.25 , 0.15 , 0.1 , CLOCKFREQUENCY , 90)

    while ((vessel.orbit.apoapsis_altituse < 100000) and (not hasAborted.peek())  and inFlight.peek()):
        if (interuptEvent.is_set()):
            break        
        output = headingPID.updateOutput(vessel.flight().yaw)
        output = headingPID.applyLimits(-1 , 1)
        output = headingPID.applyDeadzone(0.5 , vessel.flight().yaw)
        vessel.control.yaw = output

        sleep(CLOCKFREQUENCY)

    vessel.control.yaw = 0

def staging(vessel , stage):
    while (vessel.resources_in_decouple_stage(stage , False).amount("LiquidFuel") > 0):
        if (interuptEvent.is_set()):
            break
        continue
    
    vessel.control.throttle = 0
    sleep(0.25)
    vessel.control.activate_next_stage()
    vessel.control.throttle = 1

def calcPitchAngle(vessel):
    kerbinRadius = 600000
    startLatitude = vessel.flight().latitude
    startLongitude = vessel.flight().longitude

    #fligtht trajectory equation: f(x) = -0.001x^2 , f'(x) = -0.002x

    while (inFlight.peek() and (not hasAborted.peek())):
        if(interuptEvent.is_set()):
            break
        relativeLatitude = abs(abs(startLatitude) - abs(vessel.flight().latitude))
        relativeLongitude = abs(abs(startLongitude) - abs(vessel.flight().longitude))
        
        #degrees to radians
        relativeLatitude = relativeLatitude * (pi / 180)
        relativeLongitude = relativeLongitude * (pi / 180)
        
        #radians to meters
        relativeLatitude = relativeLatitude * kerbinRadius
        relativeLongitude = relativeLongitude * kerbinRadius
        #TODO normalize distance to surface altitude and compinsate for Kerbin's rotation ^, use sidereal day/rotation speed for calculation?

        downrangeDistance = ((relativeLatitude ** 2) + (relativeLongitude ** 2)) ** 0.5
        #compinsate for Kerbin's rotation, Sidereal rotatinal velocity = 174.94 m/s
        downrangeDistance += 174.94 * vessel.met * cos(vessel.orbit().inclination)
        
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

def handler(signum , frame):
    print("interupt received")
    interuptEvent.set()
    inFlight.enqueue(False)

if (__name__ == "__main__"):
    main()
    quit()