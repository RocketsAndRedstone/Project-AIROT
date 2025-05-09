import krpc , time
conn = krpc.connect(name="Launch Liquid 1")
vessel = conn.space_center.active_vessel
vessel.control.sas = True

vessel.control.activate_next_stage()
vessel.control.throttle = 1

#sets up controll for autopilot
vessel.auto_pilot.engage()
vessel.auto_pilot.roll_error(2)
vessel.auto_pilot.heading_error(5)
vessel.auto_pilot.pitch_error(10)
vessel.auto_pilot.pitch_pid_gains(0.25)
vessel.auto_pilot.yaw_pid_gains(0.25)

time.sleep(1)
vessel.control.activate_next_stage()
time.sleep(5)
vessel.auto_pilot.target_roll(90)
time.sleep(5)
vessel.auto_pilot.target_pitch_and_heading(45, 180)


#vessel.control.pitch = 1