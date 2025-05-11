import krpc , time
conn = krpc.connect(name="Launch Liquid 1")
vessel = conn.space_center.active_vessel
vessel.control.sas = True

#engine ignition and pad seperation
vessel.control.activate_next_stage()
vessel.control.throttle = 1
time.sleep(1)
vessel.control.activate_next_stage()


#posible roll program

time.sleep(5)

prevError = 0
intergral = 0
targetRoll = -90
dt = 0.2
proportinalGain = 0.2
intergralGain = 0.1
derivGain = 0.1

while not (targetRoll - 0.5 < vessel.flight().roll < targetRoll + 0.5):
    error =   targetRoll - vessel.flight().roll
    proportinal = error
    intergral = (intergral + error) * dt
    derivative = (error - prevError) / dt

    vessel.control.roll = (proportinalGain * proportinal) + (intergralGain * intergral) + (derivGain * derivative)

    prevError = error
    time.sleep(dt)


print("roll program complete at" , vessel.flight().roll)