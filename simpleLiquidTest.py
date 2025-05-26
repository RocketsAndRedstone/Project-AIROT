import krpc
from time import sleep
from threading import Thread
from queue import Queue
from CircleQueue import CircleQueue

def main():
    conn = krpc.connect(name="Launch Liquid 1")
    vessel = conn.space_center.active_vessel
    global hasAborted , inFlight, rollRate , pitchRate

    targetAapoapsis = 75000
    
    hasAborted = Queue()
    rollRate = CircleQueue(5)
    pitchRate = CircleQueue(5)
    vessel.control.sas = True

    abortThread = Thread(target=checkAbort, args=(vessel,))
    rollThread = Thread(target=rollProgram, args=(vessel,))
    gravTurnThread = Thread(target=gravTurn , args=(vessel,))
    headingThread = Thread(target=maintainHeading, args=(vessel,))
    ratesThread =Thread(target=rates, args=(vessel,))   

    #engine ignition and pad seperation
    vessel.control.activate_next_stage()
    vessel.control.throttle = 1
    sleep(1)
    vessel.control.activate_next_stage()

    inFlight = True
    ratesThread.start()
    
    sleep(5)    
    rollThread.start()
    abortThread.start()
    headingThread.start()
    gravTurnThread.start()
    
    #monitor fuel levels and schedule stageing


    #stage and jetison abort tower

    rollThread.join()
    headingThread.join()
    gravTurnThread.join()
    inFlight = False
    
    abortThread.join()

def rollProgram(vessel):
    prevError = 0
    intergral = 0
    targetRoll = -90
    dt = 0.25
    proportinalGain = 0.05
    intergralGain = 0.025
    derivGain = 0.025

    while (inFlight):
       # print("Roll Rate" , rollRate.dequeue())
        if(not hasAborted.empty()):
            print("aborting roll program")
            break
        error =   targetRoll - vessel.flight().roll
        proportinal = error
        intergral = (intergral + error) * dt
        derivative = (error - prevError) / dt

        vessel.control.roll = (proportinalGain * proportinal) + (intergralGain * intergral) + (derivGain * derivative)

        prevError = error
        sleep(dt)
    vessel.control.roll = 0

    print("roll program complete at" , vessel.flight().roll)

def checkAbort(vessel):
    while((vessel.flight().surface_altitude > 20) and (vessel.flight().surface_altitude < 5000)):
        if(vessel.flight().pitch < 35):
            vessel.control.abort = True
            print("Aborted")
            hasAborted.put("Aborted")
            inFlight = False
            break
    vessel.control.toggle_action_group(9)

def maintainHeading(vessel):
    prevError = 0
    intergral = 0
    targetHeading = 5
    dt = 0.25
    proportinalGain = 0.05
    intergralGain = 0.05
    derivGain = 0.025
    while (vessel.flight().g_force > 0.1):
        if(not hasAborted.empty()):
            print("aborting heading lock")
            break
        error = targetHeading - vessel.flight().heading
        proportinal = error
        intergral = (intergral + error) * dt
        derivative = (error - prevError) / dt
        
        vessel.control.yaw = (proportinalGain * proportinal) + (intergralGain * intergral) + (derivGain * derivative)
        
        prevError = error
        sleep(dt)
    vessel.control.yaw = 0

def gravTurn(vessel):
    prevError = 0
    intergral = 0
    targetRate = -2
    dt = 0.25
    proportinalGain = 0.05
    intergralGain = 0.05
    derivGain = 0.1
    while(vessel.resources.has_resource("LiquidFuel")):
        if(not hasAborted.empty()):
            print("aborting grav turn")
            break
        error = targetRate - pitchRate.dequeue()
        proportinal = error
        intergral = (intergral + error) * dt
        derivative = (error - prevError) / dt
        
        vessel.control.pitch = (proportinalGain * proportinal) + (intergralGain * intergral) + (derivGain * derivative)
        
        prevError = error
        sleep(dt)
    
def rates(vessel):
    sampleRate = 0.125
    lastPitch = vessel.flight().pitch
    lastYaw = vessel.flight().heading
    lastRoll = vessel.flight().roll
    lastVelocity = vessel.flight().speed
    
    while(inFlight):        
        sleep(sampleRate)
        currPitch = vessel.flight().pitch
        currYaw = vessel.flight().heading
        currRoll = vessel.flight().roll
        currVelocity = vessel.flight.speed
        
        pitchRate.enqueue((currPitch - lastPitch) / sampleRate)
        yawRate = (currYaw - lastYaw) / sampleRate
        rollRate.enqueue((currRoll - lastRoll) / sampleRate)
        verticalAcel = (currVelocity - lastVelocity) / sampleRate
        
        lastPitch = currPitch
        lastYaw = currYaw
        lastRoll = currRoll
        lastVelocity = currVelocity

if __name__ == "__main__":
    main()