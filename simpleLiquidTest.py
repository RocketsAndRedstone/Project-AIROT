import krpc
from time import sleep
from threading import Thread
from CircleQueue import CircleQueue

#A program that controls a rocket on a suborbital trajectory in KSP using the KRPC mod without the use of any autopilot features
def main():
    #Initiate connection to KSP and create control point for the rocket.
    conn = krpc.connect(name="Launch Liquid 1")
    vessel = conn.space_center.active_vessel

    global hasAborted , inFlight , turning
    
    #Initalizes global variables used across active threads
    hasAborted = CircleQueue(3)
    inFlight = CircleQueue(3)
    turning = CircleQueue(3)

    #Enables SAS
    vessel.control.sas = True

    #Initializes the threads that control and monitors the rocket
    abortThread = Thread(target=checkAbort, args=(vessel,))
    rollThread = Thread(target=rollProgram, args=(vessel,))
    gravTurnThread = Thread(target=gravTurn , args=(vessel,))

    #engine ignition and pad seperation
    vessel.control.activate_next_stage()
    vessel.control.throttle = 1
    sleep(1)
    vessel.control.activate_next_stage()
    inFlight.enqueue(True)
    
    #Starts the roll program and monitoring of the abort condition after pad seperation
    sleep(5)
    turning.enqueue(False)
    rollThread.start()
    abortThread.start()

    #Ends the roll program and starts the gravity turn
    sleep(15)
    turning.enqueue(True)
    gravTurnThread.start()

    #Starts monitoring the fuel levels and initiates staging after the fuel in the first stage runs out
    monitorFuel(vessel , 4)
    vessel.control.throttle = 0
    vessel.control.activate_next_stage()
    sleep(0.5)
    vessel.control.throttle = 1
    sleep(0.5)
    vessel.control.activate_next_stage()

    #Monitors fuel level and initates SECO
    monitorFuel(vessel , 2)
    vessel.control.throttle = 0

    #Closes the avtive threads as they are no longer needed
    rollThread.join()
    inFlight.enqueue(False)
    gravTurnThread.join()
    abortThread.join()

    #Seperates the capsule after SECO to ensure craft stability
    sleep(15)
    vessel.control.activate_next_stage()

    #Landing sequence
    entryDecentLanding(vessel)

def rollProgram(vessel):
    #A PID control loop to set the roll of the vehicle based off of the structure shown on the PID control loop Wikipedia page

    #Initalize variables used in the roll PID loop
    prevError = 0
    intergral = 0
    targetRoll = -90
    dt = 0.25
    proportinalGain = 0.025
    intergralGain = 0.025
    derivGain = 0.025
    lastOutput = 0

    #limits for the output, KRPC only uses inputs between +- 1
    maxOutput = 1
    minOutput = -1

    while (inFlight and not turning.peek()):
        #Checks if the abort criteia has been violated and ends the control loop 
        if(hasAborted.peek()):
            print("aborting roll program")
            break

        #Calculations to determine the output to get to the desired roll atitude
        error =   targetRoll - vessel.flight().roll
        proportinal = error
        intergral = (intergral + error) * dt
        derivative = (error - prevError) / dt

        output = (proportinalGain * proportinal) + (intergralGain * intergral) + (derivGain * derivative)

        #Sets the output to +- 1 or +- 0.3 if calculated value is greater than set limit
        if(abs(output) > 1):
            if(output < 0):
                output = minOutput
            else:
                output = maxOutput

        #Sets deadzone and resets gain
        if((vessel.flight().roll - 1 < targetRoll < vessel.flight().roll + 1) or (abs(output) < 0.001)) :
            output = 0
            proportinalGain = 0.0125
            intergralGain = 0.0125
            derivGain = 0.0125

        #Sets the max/min output to +- 0.3 and decreses each gain if the last output is not 0
        elif((abs(lastOutput) > 0) and output > 0):
            maxOutput = 0.3
            minOutput = -0.3
            proportinalGain /= 2
            intergralGain /= 2
            derivGain /= 2

        vessel.control.roll = output

        #resets needed variables for next iteration of the loop
        prevError = error
        lastOutput = output
        sleep(dt)
    
    #Sets the output to 0 and outputs the state of the roll at the end of the roll program
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
    #A PID control loop to set the pitch of the vehicle based off of the structure shown on the PID control loop Wikipedia page

    #Initalize variables used for gravity turn PID loop
    prevError = 0
    intergral = 0
    dt = 0.25
    proportinalGain = 0.06
    intergralGain = 0.06
    derivGain = 0.06
    lastOutput = 0

    #lists to control the target pitch of the craft at set points duting the flight
    targetAngle = [75 , 60 ,45 , 30, 15 , 5]
    maxAltitude = [20000 , 30000 , 40000 , 50000, 60000 , 80000]
    altitudeIndex = 0

    #limits for the output, KRPC only uses inputs of +- 1
    minOutput = -1
    maxOutput = 1

    while(inFlight.peek()):
        #Checks if the abort criteria has been violated and ends the control loop
        if(hasAborted.peek()):
            print("aborting grav turn")
            break

        #Calculations to determine the output to get to the desired pitch angle
        error = targetAngle[altitudeIndex] - vessel.flight().pitch
        proportinal = error
        intergral = (intergral + error) * dt
        derivative = (error - prevError) / dt
        
        output = (proportinalGain * proportinal) + (intergralGain * intergral) + (derivGain * derivative)

        #Checks to see if incresing the altitude index would reset the target altitude
        if (altitudeIndex + 1 < len(maxAltitude)):
            #Increments the index refrence value if the current apoapsis is greater than the current target apoapsis height
            if(vessel.orbit.apoapsis_altitude > maxAltitude[altitudeIndex] and (vessel.orbit.apoapsis_altitude < maxAltitude[altitudeIndex + 1])):
                altitudeIndex += 1
                proportinalGain = 0.06
                intergralGain = 00.6
                derivGain = 0.06

        #sets the output to the set max/min if calculated value is greater than the set limits
        if(abs(output) > 1):
            if(output < 0):
                output = minOutput
            else:
                output = maxOutput 

        #Deadzone for pitch angle and resets gains
        if((targetAngle[altitudeIndex] - 1 <= vessel.flight().pitch <= targetAngle[altitudeIndex] + 1) or abs(output) < 0.01):
            output = 0
            proportinalGain = 0.03
            intergralGain = 0.03
            derivGain = 0.03

        #Decreases gain and max/min output if last input is not 0
        elif((abs(lastOutput) > 0) and output > 0):
            maxOutput = 0.5
            minOutput = -0.5
            proportinalGain /= 2
            intergralGain /= 2
            derivGain /= 2
        
        vessel.control.pitch = output

        #Resets variables for the next iteration of the loop 
        prevError = error
        lastOutput = output    
        sleep(dt)
    
    #Sets the output to 0 and outputs the state of the roll at the end of the roll program
    vessel.control.pitch = 0
    print("Gravity turn complete")

def monitorFuel(vessel, stage):
    #Checks if the fuel level in designated stage is not 0
    while(vessel.resources_in_decouple_stage(stage,False).amount("LiquidFuel") > 0):
        continue

def entryDecentLanding(vessel):
    #Logic for the orientation of the capsule during reentry and the altitudes of when to deploy chutes

    sleep(30)
    #Attitude for reentry
    while(vessel.flight().surface_altitude > 50000):
        continue
    
    vessel.control.sas_mode = vessel.control.sas_mode.retrograde

    while(vessel.flight().surface_altitude > 4000):
        continue

    #Drouge chute deploy
    vessel.control.activate_next_stage()

    while(vessel.flight().surface_altitude > 1500):
        continue

    #Main chute deploy and sets the attitude to what the craft settles into
    vessel.control.sas_mode = vessel.control.sas_mode.stability_assist
    vessel.control.activate_next_stage()


if __name__ == "__main__":
    main()
    quit()