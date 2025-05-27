import krpc
from time import sleep
from threading import Thread
from CircleQueue import CircleQueue

def main():
    conn = krpc.connect(name="Launch Liquid 1")
    vessel = conn.space_center.active_vessel
    global hasAborted , inFlight, rollRate , pitchRate

    targetAapoapsis = 75000
    
    hasAborted = CircleQueue(3)
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
  #  abortThread.start()
    sleep(10)
    #headingThread.start()
   # gravTurnThread.start()
    
    monitorFuel(vessel)
    if not hasAborted.peek():
        vessel.control.activate_next_stage()
        vessel.control.toggle_action_group(0)
        monitorFuel(vessel)
        vessel.control.activate_next_stage()

    rollThread.join()
   # headingThread.join()
   # gravTurnThread.join()
    inFlight = False
    
   # abortThread.join()

def rollProgram(vessel):
    prevError = 0
    intergral = 0
    targetRoll = -90
    dt = 0.25
    proportinalGain = 0.05
    intergralGain = 0.025
    derivGain = 0.025

    while (inFlight):
        if(hasAborted.peek()):
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
            hasAborted.enqueue(True)
            inFlight = False
            break
        hasAborted.enqueue(False)

def maintainHeading(vessel):
    prevError = 0
    intergral = 0
    targetHeading = 5
    dt = 0.25
    proportinalGain = 0.05
    intergralGain = 0.05
    derivGain = 0.025
    while (vessel.flight().g_force > 0.1):
        if(hasAborted.peek()):
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
    while(inFlight):
        if(hasAborted.peek()):
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
        currVelocity = vessel.flight().speed
        
        pitchRate.enqueue((currPitch - lastPitch) / sampleRate)
        yawRate = (currYaw - lastYaw) / sampleRate
        rollRate.enqueue((currRoll - lastRoll) / sampleRate)
        verticalAcel = (currVelocity - lastVelocity) / sampleRate
        
        lastPitch = currPitch
        lastYaw = currYaw
        lastRoll = currRoll
        lastVelocity = currVelocity

def monitorFuel(vessel):
    while(vessel.resources.amount("LiquidFuel")):
        continue


if __name__ == "__main__":
    main()