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

    stage = 4

    rollThread = Thread(target = rollProgram , args=(vessel,))
    pitchAngleThread = Thread(target = pitchAngle , args=(vessel,))
    gravTurnThread = Thread(target = gravTurn , args=(vessel,))
    headingLockThread = Thread(target= headingLock , args=(vessel,))
    abortThread = Thread(target = monitorAbort , args=(vessel,))
    stageThread = Thread(target = staging , args=(vessel, stage))

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
    
    sleep(CLOCKFREQUENCY * 20)

    rollThread.start()
    sleep(CLOCKFREQUENCY * 40)

    #abortThread.start()
    
    gravTurnThread.start()
    sleep(CLOCKFREQUENCY * 4)
    stageThread.start()
    sleep(CLOCKFREQUENCY * 4)
    headingLockThread.start()

    #abortThread.join()

    stageThread.join()
    pitchAngleThread.join()
    gravTurnThread.join()
    rollThread.join()
    headingLockThread.join()
    inFlight = False

    vessel.control.throttle = 0
    
    abortContigencys(vessel)
    

def monitorAbort(vessel):
    while (vessel.orbit.apoapsis_altitude < 70000 and inFlight.peek()):
        if(interuptEvent.is_set()):
                break
        if ((targetPitch.peek() - 5)  > vessel.flight().pitch or (vessel.flight().pitch > (targetPitch.peek() + 5))):            
            print("Aborted")
            hasAborted.enqueue(True)
            inFlight.enqueue(False)
            vessel.control.pitch = 1
            vessel.control.abort = True
            break

def rollProgram(vessel):
    rollPid = PID(0.105 , 0.1 , 0.05 , CLOCKFREQUENCY , 0)
    vessel.control.yaw = 1
    sleep(CLOCKFREQUENCY * 4)
    vessel.control.yaw = 0

    while (vessel.orbit.periapsis_altitude < 100000 and ((not hasAborted.peek()) and (inFlight.peek()))):
        if(interuptEvent.is_set()):
            break
        output = rollPid.updateOutput(vessel.flight().roll)
        output = rollPid.applyLimits(-1 , 1)
        output = rollPid.applyDeadzone(0.5 , vessel.flight().roll)
        vessel.control.roll = output
        sleep(CLOCKFREQUENCY)

    vessel.control.roll = 0

def gravTurn(vessel):
    turnPID = PID(0.2 , 0.125 , 0.125 , CLOCKFREQUENCY , targetPitch.peek())
    stageTwoRelight = True

    while((vessel.orbit.periapsis_altitude < 100000) and ((not hasAborted.peek()) and (inFlight.peek()))):
        if(interuptEvent.is_set()):
            break
        pitchTarget = targetPitch.peek()
        if (vessel.orbit.apoapsis_altitude > 100000):
            pitchTarget = 0
            if (vessel.orbit.time_to_apoapsis > 65 and stageTwoRelight):
                vessel.control.throttle = 0
            else:
                vessel.control.throttle = 1
                stageTwoRelight = False
        turnPID.updateTarget(pitchTarget)
        output = turnPID.updateOutput(vessel.flight().pitch)
        output = turnPID.applyLimits(-1 , 1)
        output = turnPID.applyDeadzone(0.5 , vessel.flight().pitch)
        vessel.control.pitch = output
        sleep(CLOCKFREQUENCY)
    
    vessel.control.pitch = 0

def headingLock(vessel):
    headingPID = PID(0.15 , 0.125 , 0.15 , CLOCKFREQUENCY , 90)

    while ((vessel.orbit.periapsis_altitude < 100000) and (not hasAborted.peek())  and inFlight.peek()):
        if (interuptEvent.is_set()):
            break        
        output = headingPID.updateOutput(vessel.flight().heading)
        output = headingPID.applyLimits(-1 , 1)
        output = headingPID.applyDeadzone(1 , vessel.flight().heading)
        vessel.control.yaw = output

        sleep(CLOCKFREQUENCY)

    vessel.control.yaw = 0

def staging(vessel , stage):
    while (vessel.resources_in_decouple_stage(stage , False).amount("LiquidFuel") > 0.5):
        if (interuptEvent.is_set()):
            break
        sleep(CLOCKFREQUENCY)
        continue
    
    vessel.control.throttle = 0
    sleep(CLOCKFREQUENCY)
    vessel.control.activate_next_stage()
    sleep(CLOCKFREQUENCY * 2)
    vessel.control.throttle = 1
    
    while (vessel.orbit.periapsis_altitude < 0):
                continue

    vessel.control.activate_next_stage()

def pitchAngle(vessel):
    referenceAltitude = 5000
    pitch = 70

    while ((vessel.orbit.apoapsis_altitude < 100000) and (inFlight.peek() and (not hasAborted.peek()))):
        if (interuptEvent.is_set()):
            break
        if (vessel.flight().surface_altitude < referenceAltitude):
            targetPitch.enqueue(pitch)
        else:
            pitch -= 15
            referenceAltitude += 10000

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