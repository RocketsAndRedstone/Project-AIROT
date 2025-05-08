import krpc , time
conn = krpc.connect(name="Launch Liquid 1")
vessel = conn.space_center.active_vessel
vessel.control.sas = True

vessel.control.activate_next_stage()
vessel.control.throttle = 1
vessel.auto_pilot.engage()
time.sleep(1)
vessel.control.activate_next_stage()
time.sleep(10)
while  vessel.flight().heading != 200:
    vessel.control.yaw = 0.25

#vessel.control.pitch = 1