import krpc
from time import sleep
from threading import Thread
from CircleQueue import CircleQueue

def main():
    conn = krpc.connect(name="Launch Liquid 1")
    vessel = conn.space_center.active_vessel
    global hasAborted , inFlight, rollRate , pitchRate , currentInput

    targetAapoapsis = 75000
    
    hasAborted = CircleQueue(3)
    rollRate = CircleQueue(5)
    pitchRate = CircleQueue(5)
    #have the output of each controll loop go to a queue to prevent multiple inputs at the same time to damp ossolations?
    currentInput = CircleQueue(10)
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
    #rollThread.start()
    abortThread.start()
    sleep(10)
   # headingThread.start()
    gravTurnThread.start()
    '''
    monitorFuel(vessel)
    vessel.control.throttle = 0
    vessel.control.activate_next_stage()
    sleep(0.5)
    vessel.control.throttle = 1
    sleep(0.5)
    vessel.control.activate_next_stage()
    monitorFuel(vessel)
    sleep(2)
    vessel.control.activate_next_stage()'''

    #rollThread.join()
    #headingThread.join()
    gravTurnThread.join()
    inFlight = False
    
    abortThread.join()

def rollProgram(vessel):
    prevError = 0
    intergral = 0
    targetRoll = -90
    dt = 0.25
    proportinalGain = 0.025
    intergralGain = 0.025
    derivGain = 0.025

    lastOutput = 0

    maxOutput = 1
    minOutput = -1

    while (inFlight):
        if(hasAborted.peek()):
            print("aborting roll program")
            break
        error =   targetRoll - vessel.flight().roll
        proportinal = error
        intergral = (intergral + error) * dt
        derivative = (error - prevError) / dt

        output = (proportinalGain * proportinal) + (intergralGain * intergral) + (derivGain * derivative)
        if(abs(output) > 1):
            if(output < 0):
                output = minOutput
            else:
                output = maxOutput

        if((vessel.flight().roll - 1 < targetRoll < vessel.flight().roll + 1) or (abs(output) < 0.001)) :
            output = 0
            proportinalGain = 0.0125
            intergralGain = 0.0125
            derivGain = 0.0125

        elif((abs(lastOutput) > 0) and output > 0):
            maxOutput = 0.3
            minOutput = -0.3
            proportinalGain /= 2
            intergralGain /= 2
            derivGain /= 2

        vessel.control.roll = output

        prevError = error
        lastOutput = output
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

    proportinalGain = 0.025
    intergralGain = 0.025
    derivGain = 0.025

    targetAngle = [75 , 45 , 30, 15]
    maxAltitude = [25000 , 45000 , 60000, 80000]

    minOutput = -1
    maxOutput = 1

    altitudeIndex = 0

    lastOutput = 0

    while(inFlight):
        if(hasAborted.peek()):
            print("aborting grav turn")
            break
        error = targetRate - pitchRate.dequeue()
        proportinal = error
        intergral = (intergral + error) * dt
        derivative = (error - prevError) / dt
        
        output = (proportinalGain * proportinal) + (intergralGain * intergral) + (derivGain * derivative)


        if (altitudeIndex + 1 < len(maxAltitude)):
            if(vessel.orbit.apoapsis_altitude > maxAltitude[altitudeIndex] and (vessel.orbit.apoapsis_altitude < maxAltitude[altitudeIndex + 1])):
                altitudeIndex += 1
                proportinalGain = 0.025
                intergralGain = 0.025
                derivGain = 0.025

        if(abs(output) > 1):
            if(output < 0):
                output = minOutput
            else:
                output = maxOutput

        if((targetAngle[altitudeIndex] - 1 <= vessel.flight().pitch <= targetAngle[altitudeIndex] + 1) or abs(output) < 0.001):
            output = 0
            proportinalGain = 0.0125
            intergralGain = 0.0125
            derivGain = 0.0125

        elif((abs(lastOutput) > 0) and output > 0):
            maxOutput = 0.3
            minOutput = -0.3
            proportinalGain /= 2
            intergralGain /= 2
            derivGain /= 2
        
        vessel.control.pitch = output
        
        print(output)
        
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
    while(vessel.resources_in_decouple_stage(vessel.parts.stage,False).amount("LiquidFuel") > 0):
        continue


if __name__ == "__main__":
    main()
    quit()