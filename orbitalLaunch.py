import krpc
from time import sleep
from threading import Thread
from CircleQueue import CircleQueue
from PID import PID

def main():
    conn = krpc.connect(name="Orbital launch control")
    vessel = conn.space_center.active_vessel

    global hasAborted , inFlight
    hasAborted = CircleQueue(1)
    inFlight = CircleQueue(1)

    rollThread = Thread(target=rollProgram , args=(vessel,))
    abortThread = Thread(target=monitorAbort , args=(vessel,))

    hasAborted.enqueue(False)
    inFlight.enqueue(False)

    vessel.control.sas = True
    vessel.control.throttle = 1
    vessel.control.activate_next_stage()
    inFlight.enqueue(True)
    sleep(0.25)
    vessel.control.activate_next_stage()
    sleep(5)
    monitorStaging(vessel)
    rollThread.start()
    rollThread.join()

def monitorAbort(vessel):
    #TODO add abort criteria for full flight regime untill abort tower sep
    pass

def rollProgram(vessel):
    loopTime = 0.25
    rollPid = PID(0.25 , 0.25 , 0.25 , loopTime , -90.0)

    #figure out better condition for roll PID end
    while ((vessel.flight().surface_altitude < 1000) and (not hasAborted.peek()) and (inFlight.peek())):
        output = rollPid.updateOutput(vessel.flight().roll)
        output = rollPid.applyLimits(-1 , 1)
        output = rollPid.applyDeadzone(0.5)
        print(output)
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

if (__name__ == "__main__"):
    main()
    quit()