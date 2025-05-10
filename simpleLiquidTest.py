import krpc , time
conn = krpc.connect(name="Launch Liquid 1")
vessel = conn.space_center.active_vessel
vessel.control.sas = True

vessel.control.activate_next_stage()
vessel.control.throttle = 1
time.sleep(1)
vessel.control.activate_next_stage()


#vessel.control.pitch = 1