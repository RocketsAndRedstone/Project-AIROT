import krpc
from time import sleep
from threading import Thread
from CircleQueue import CircleQueue

def main():
    conn = krpc.connect(name="Launch Liquid 1")
    vessel = conn.space_center.active_vessel
    global hasAborted , inFlight , currentInput , turning
    
    hasAborted = CircleQueue(3)
    inFlight = CircleQueue(3)
    turning = CircleQueue(3)
    currentInput = CircleQueue(10)
    vessel.control.sas = True

    abortThread = Thread(target=checkAbort, args=(vessel,))
    rollThread = Thread(target=rollProgram, args=(vessel,))
    gravTurnThread = Thread(target=gravTurn , args=(vessel,))

    #engine ignition and pad seperation
    vessel.control.activate_next_stage()
    vessel.control.throttle = 1
    sleep(1)
    vessel.control.activate_next_stage()

    inFlight.enqueue(True)
    
    sleep(5)
    turning.enqueue(False)
    rollThread.start()
    abortThread.start()
    sleep(15)
    turning.enqueue(True)
    gravTurnThread.start()

    
    monitorFuel(vessel , 4)
    vessel.control.throttle = 0
    vessel.control.activate_next_stage()
    sleep(0.5)
    vessel.control.throttle = 1
    sleep(0.5)
    vessel.control.activate_next_stage()
    monitorFuel(vessel , 2)
    vessel.control.throttle = 0

    rollThread.join()

    inFlight.enqueue(False)
    gravTurnThread.join()
    
    abortThread.join()

    sleep(15)

    vessel.control.activate_next_stage()

    entryDecentLanding(vessel)

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

    while (inFlight and not turning.peek()):
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
            inFlight.enqueue(False)
            break
        hasAborted.enqueue(False)

def gravTurn(vessel):
    prevError = 0
    intergral = 0
    dt = 0.25

    proportinalGain = 0.06
    intergralGain = 0.06
    derivGain = 0.06

    targetAngle = [75 , 60 ,45 , 30, 15 , 5]
    maxAltitude = [20000 , 30000 , 40000 , 50000, 60000 , 80000]

    minOutput = -1
    maxOutput = 1

    altitudeIndex = 0

    lastOutput = 0

    while(inFlight.peek()):
        if(hasAborted.peek()):
            print("aborting grav turn")
            break
        error = targetAngle[altitudeIndex] - vessel.flight().pitch
        proportinal = error
        intergral = (intergral + error) * dt
        derivative = (error - prevError) / dt
        
        output = (proportinalGain * proportinal) + (intergralGain * intergral) + (derivGain * derivative)


        if (altitudeIndex + 1 < len(maxAltitude)):
            if(vessel.orbit.apoapsis_altitude > maxAltitude[altitudeIndex] and (vessel.orbit.apoapsis_altitude < maxAltitude[altitudeIndex + 1])):
                altitudeIndex += 1
                proportinalGain = 0.06
                intergralGain = 00.6
                derivGain = 0.06

        if(abs(output) > 1):
            if(output < 0):
                output = minOutput
            else:
                output = maxOutput 

        if((targetAngle[altitudeIndex] - 1 <= vessel.flight().pitch <= targetAngle[altitudeIndex] + 1) or abs(output) < 0.01):
            output = 0
            proportinalGain = 0.03
            intergralGain = 0.03
            derivGain = 0.03

        elif((abs(lastOutput) > 0) and output > 0):
            maxOutput = 0.5
            minOutput = -0.5
            proportinalGain /= 2
            intergralGain /= 2
            derivGain /= 2
        
        vessel.control.pitch = output

        prevError = error
        lastOutput = output
    
        sleep(dt)

def monitorFuel(vessel, stage):
    while(vessel.resources_in_decouple_stage(stage,False).amount("LiquidFuel") > 0):
        continue

def entryDecentLanding(vessel):

    sleep(30)
    
    while(vessel.flight().surface_altitude > 50000):
        continue
    
    vessel.control.sas_mode = vessel.control.sas_mode.retrograde

    while(vessel.flight().surface_altitude > 3500):
        continue

    vessel.control.activate_next_stage()

    while(vessel.flight().surface_altitude > 1500):
        continue

    vessel.control.sas_mode = vessel.control.sas_mode.stability_assist
    vessel.control.activate_next_stage()


if __name__ == "__main__":
    main()
    quit()