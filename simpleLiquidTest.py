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

vessel.auto_pilot.engage()
vessel.auto_pilot.target_roll(90)
vessel.auto_pilot.wait()
print("roll program complete")