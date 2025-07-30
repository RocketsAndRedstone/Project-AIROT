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


def monitorAbort(vessel):
    #TODO add abort criteria for full flight regime untill abort tower sep
    pass

def rollProgram(vessel):
    #TODO add roll program logic
    pass

def gravTurn(vessel):
    #TODO add logic for smooth and consistant grav turn
    pass

def headingLock(vessel):
    #TODO add logic to follow a set azumith using yaw for proper orbital insertion
    pass

def staging(vessel):
    #TODO add logic for monitoring when to activate staging
    pass