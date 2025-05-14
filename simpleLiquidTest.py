import krpc , time , threading

def main():
    conn = krpc.connect(name="Launch Liquid 1")
    vessel = conn.space_center.active_vessel
    vessel.control.sas = True

    abortThread = threading.Thread(target=checkAbort, args=(vessel,))
    rollThread = threading.Thread(target=rollProgram, args=(vessel,))
    headingThread = threading.Thread(target=maintainHeading, args=(vessel,))

    #engine ignition and pad seperation
    vessel.control.activate_next_stage()
    vessel.control.throttle = 1
    time.sleep(1)
    vessel.control.activate_next_stage()

    time.sleep(5)    
    rollThread.start()
    abortThread.start()

    rollThread.join()
    headingThread.start()
    
    abortThread.join()

def rollProgram(vessel):
    prevError = 0
    intergral = 0
    targetRoll = -90
    dt = 0.25
    proportinalGain = 0.05
    intergralGain = 0.025
    derivGain = 0.025

    while not (targetRoll - 0.15 < vessel.flight().roll < targetRoll + 0.15):
        error =   targetRoll - vessel.flight().roll
        proportinal = error
        intergral = (intergral + error) * dt
        derivative = (error - prevError) / dt

        vessel.control.roll = (proportinalGain * proportinal) + (intergralGain * intergral) + (derivGain * derivative)

        prevError = error
        time.sleep(dt)
    vessel.control.roll = 0

    print("roll program complete at" , vessel.flight().roll)

def checkAbort(vessel):
    while((vessel.flight().surface_altitude > 20) and (vessel.flight().surface_altitude < 5000)):
        if(vessel.flight().pitch < 35):
            vessel.control.abort = True
            print("Aborted")
            break

def maintainHeading(vessel):
    prevError = 0
    intergral = 0
    targetHeading = 180
    dt = 0.25
    proportinalGain = 0.05
    intergralGain = 0.05
    derivGain = 0.05
    while (vessel.flight().g_force > 0.1):
        error = targetHeading - vessel.flight().heading
        proportinal = error
        intergral = (intergral + error) * dt
        derivative = (error - prevError) / dt
        
        vessel.control.yaw = (proportinalGain * proportinal) + (intergralGain * intergral) + (derivGain * derivative)
        
        prevError = error
        time.sleep(dt)
    vessel.control.yaw = 0

if __name__ == "__main__":
    main()