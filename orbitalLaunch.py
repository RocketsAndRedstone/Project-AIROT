#needed imports, includding a couple custom classes
import krpc
from time import sleep
from threading import Thread , Event
from signal import signal , SIGINT
from CircleQueue import CircleQueue
from PID import PID

#main function, holding the launch sequence and hosting the threads
def main():
    #connects to KSP and creates the vessel object that the rest of the code interacts with
    conn = krpc.connect(name="Orbital launch control")
    vessel = conn.space_center.active_vessel

    #declares global variables and queue objects that are used for inter-thread communication
    global CLOCKFREQUENCY , hasAborted , inFlight , targetPitch , interuptEvent , signal
    CLOCKFREQUENCY = 0.25
    targetPitch = CircleQueue(2)
    hasAborted = CircleQueue(1)
    inFlight = CircleQueue(1)
    
    #keyboard interupt setup
    interuptEvent = Event()
    signal(SIGINT , handler)

    stage = 4

    #setups the threads that are used to run various vessel controls and calculations in parallel
    rollThread = Thread(target = rollProgram , args=(vessel,))
    pitchAngleThread = Thread(target = pitchAngle , args=(vessel,))
    gravTurnThread = Thread(target = gravTurn , args=(vessel,))
    headingLockThread = Thread(target= headingLock , args=(vessel,))
    abortThread = Thread(target = monitorAbort , args=(vessel,))
    stageThread = Thread(target = staging , args=(vessel, stage))
    throttleThread = Thread(target=throttleControl , args=(vessel, conn,))

    #sets up conditions that can end the program if triggered regardless of what threads are running
    hasAborted.enqueue(False)
    inFlight.enqueue(False)

    #start of the launch sequence, enablels SAS and starts the engines
    vessel.control.sas = True
    throttleThread.start()
    vessel.control.activate_next_stage()

    #seperates the rocket from the pad
    inFlight.enqueue(True)
    sleep(CLOCKFREQUENCY)
    vessel.control.activate_next_stage()

    #Starts the pitch angle calculations
    pitchAngleThread.start()
    sleep(CLOCKFREQUENCY)
    
    sleep(CLOCKFREQUENCY * 20)

    #Starts the roll program
    rollThread.start()
    sleep(CLOCKFREQUENCY * 40)

    #start's monitoring for an abort
    abortThread.start()
    
    #Starts the pitch over and grav turn
    gravTurnThread.start()
    sleep(CLOCKFREQUENCY * 4)

    #starts monoritong resources for stage seperation and starts guidence towards the wanted heading
    stageThread.start()
    sleep(CLOCKFREQUENCY * 4)
    headingLockThread.start()

    #Ends the threads once they have finished what they needed to do
    abortThread.join()
    stageThread.join()
    pitchAngleThread.join()
    gravTurnThread.join()
    rollThread.join()
    headingLockThread.join()
    inFlight = False

    throttleThread.join()
    
    #Controls the capsule if an abort is triggered, nothing happens if not
    abortContigencys(vessel)
    

def monitorAbort(vessel):
    #Monotors if the abort is happening within the spesified safe altitude
    while (vessel.orbit.periapsis_altitude < -250000 and inFlight.peek()):
        #breaks the loop if a keyboard interupt is detected
        if(interuptEvent.is_set()):
                break
        if (vessel.control.abort):            
            print("Aborted")
            hasAborted.enqueue(True)
            inFlight.enqueue(False)
            break

def rollProgram(vessel):
    #creates the control loop object
    rollPid = PID(0.15 , 0.125 , 0.175 , CLOCKFREQUENCY , 0)
    #inital pitch kick to simplify grav turn calculations
    vessel.control.yaw = 1
    sleep(CLOCKFREQUENCY * 4)
    #ends pitch kick
    vessel.control.yaw = 0

    while (vessel.orbit.periapsis_altitude < 100000 and ((not hasAborted.peek()) and (inFlight.peek()))):
        #breaks the loop if a keyboard interupt is detected
        if(interuptEvent.is_set()):
            break
        #updates control loop with current vessel orientation, limits the returned output to +-1 as limited by krpc and then applies a deadsone
        output = rollPid.updateOutput(vessel.flight().roll)
        output = rollPid.applyLimits(-1 , 1)
        output = rollPid.applyDeadzone(0.5 , vessel.flight().roll)
        vessel.control.roll = output
        sleep(CLOCKFREQUENCY)

    vessel.control.roll = 0

def gravTurn(vessel):
    #creates control loop object
    turnPID = PID(0.5 , 0.25 , 0.15 , CLOCKFREQUENCY , targetPitch.peek())
   
    while((vessel.orbit.periapsis_altitude < 100000) and ((not hasAborted.peek()) and (inFlight.peek()))):
        #breaks the loop if a keyboard interupt is detected
        if(interuptEvent.is_set()):
            break
        #gets the current targeted pitch angle and updates the control loop with it
        pitchTarget = targetPitch.peek()
        turnPID.updateTarget(pitchTarget)
        #updates control loop with current vessel orientation, limits the returned output to +-1 as limited by krpc and then applies a deadsone
        output = turnPID.updateOutput(vessel.flight().pitch)
        output = turnPID.applyLimits(-1 , 1)
        output = turnPID.applyDeadzone(0.5 , vessel.flight().pitch)
        vessel.control.pitch = output
        sleep(CLOCKFREQUENCY)
    
    vessel.control.pitch = 0

def headingLock(vessel):
    #creates control loop object
    headingPID = PID(0.175 , 0.125 , 0.125 , CLOCKFREQUENCY , 90)

    while ((vessel.orbit.periapsis_altitude < 100000) and (not hasAborted.peek())  and inFlight.peek()):
        #breaks the loop if a keyboard interupt is detected
        if (interuptEvent.is_set()):
            break
        #updates control loop with current vessel orientation, limits the returned output to +-1 as limited by krpc and then applies a deadsone
        output = headingPID.updateOutput(vessel.flight().heading)
        output = headingPID.applyLimits(-1 , 1)
        output = headingPID.applyDeadzone(1 , vessel.flight().heading)
        vessel.control.yaw = output

        sleep(CLOCKFREQUENCY)

    vessel.control.yaw = 0

def staging(vessel , stage):
    #monitors first stage fuel
    while (vessel.resources_in_decouple_stage(stage , False).amount("LiquidFuel") > 0.5):
        #breaks the loop if a keyboard interupt is detected
        if (interuptEvent.is_set()):
            break
        sleep(CLOCKFREQUENCY)
        continue

    #stage seperation
    vessel.control.throttle = 0
    sleep(CLOCKFREQUENCY)
    vessel.control.activate_next_stage()
    sleep(CLOCKFREQUENCY * 2)
    vessel.control.throttle = 1
    
    #abort tower jettison logic
    while (vessel.orbit.periapsis_altitude < -250000):
                continue

    vessel.control.activate_next_stage()

def pitchAngle(vessel):
    #inital pitch calculation parameters
    referenceAltitude = 7000
    pitch = 70

    while ((vessel.orbit.periapsis_altitude < 100000) and (inFlight.peek() and (not hasAborted.peek()))):
        #breaks the loop if a keyboard interupt is detected
        if (interuptEvent.is_set()):
            break

        #sets the inital pitch
        if (vessel.flight().surface_altitude < referenceAltitude):
            targetPitch.enqueue(pitch)

        else:
            #calculates the propper pitch angle given the current apoapsis altitude and pitch angle
            pitch -= 3
            if (vessel.orbit.apoapsis_altitude < 90000 and pitch < -8):
                pitch = -8
            elif (vessel.orbit.apoapsis_altitude > 90000):
                pitch = -65
            elif (vessel.orbit.apoapsis_altitude > 110000):
                pitch = 90                
            targetPitch.enqueue(pitch)
            referenceAltitude += 2000

        sleep(CLOCKFREQUENCY)

def throttleControl(vessel , conn):
    #inital throttle up
    dynamicPressureLast = vessel.flight().dynamic_pressure
    vessel.control.throttle = 1 

    #checks dynamic pressue for predetermined point
    while (dynamicPressureLast < 25000 and not interuptEvent.is_set()):
        dynamicPressureLast = vessel.flight().dynamic_pressure
        sleep(CLOCKFREQUENCY)
        continue

    #throttle down for max Q
    vessel.control.throttle = 0.75

    #checks dynamic pressue against static pressure for throttle up
    while (dynamicPressureLast < vessel.flight().static_pressure and not (interuptEvent.is_set() or hasAborted.peek())):
        sleep(CLOCKFREQUENCY)
        continue
    
    #throttle up after max Q
    vessel.control.throttle = 1

    #SECO 1 condition check
    while (vessel.orbit.apoapsis_altitude < 115000 and not (interuptEvent.is_set() or hasAborted.peek())):
        sleep(CLOCKFREQUENCY)
        continue

    #SECO 1
    vessel.control.throttle = 0    
    
    #warps to slightly before apoapsis and reorientates to progreade for SES 2
    conn.space_center.warp_to((conn.space_center.ut + vessel.orbit.time_to_apoapsis - 20))
    vessel.control.sas_mode = vessel.control.sas_mode.prograde

    vessel.control.rcs = True
    
    sleep(CLOCKFREQUENCY * 60)
    
    #SES 2
    vessel.control.throttle = 1

    #checks for propper periapsis height for SECO 2
    while (vessel.orbit.periapsis_altitude < 115000):
        sleep(CLOCKFREQUENCY)
        continue

    #SECO 2
    vessel.control.throttle = 0
    vessel.control.rcs = False


def abortContigencys(vessel):
    #checks if an abort has been triggered and if the vessel is going to reenter the atmosphere
    if((hasAborted.peek()) and vessel.orbit.periapsis_altitude < 70000):
        #checks if the capsule is going up
        while (vessel.flight().vertical_speed > 0 and not interuptEvent.is_set()):
            continue

        vessel.control.activate_next_stage()
        vessel.control.sas = False

        while (vessel.flight().surface_altitude > 3000 and not interuptEvent.is_set()):
            continue

        vessel.control.activate_next_stage()

        while (vessel.flight().surface_altitude > 2000 and not interuptEvent.is_set()):
            continue

        vessel.control.activate_next_stage()

#keyboard interupt handler
def handler(signum , frame):
    print("interupt received")
    interuptEvent.set()
    inFlight.enqueue(False)

#runs main if this file is the program being called and not if called from a seperate program
if (__name__ == "__main__"):
    main()
    quit()