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

    hasAborted.enqueue(False)
    inFlight.enqueue(False)


def monitorAbort(vessel):
    #TODO add abort criteria for full flight regime untill abort tower sep
    pass

def rollProgram(vessel):
    loopTime = 0.25
    rollPid = PID(0.5 , 0.5 , 0.5 , loopTime , 90)

    #figure out better condition for roll PID end
    while ((vessel.flight().surface_alttude < 500) and (not hasAborted.peek()) and (inFlight.peek())):
        output = rollPid.updateOutput(vessel.flight().roll)
        output = rollPid.applyLimits(-1 , 1)
        output = rollPid.applyDeadzone(0.05)
        vessel.control.roll = output
        sleep(loopTime)

def gravTurn(vessel):
    #TODO add logic for smooth and consistant grav turn
    pass

def headingLock(vessel):
    #TODO add logic to follow a set azumith using yaw for proper orbital insertion
    pass

def staging(vessel):
    #TODO add logic for monitoring when to activate staging
    pass