import krpc , time
conn = krpc.connect(name="Launch Solid 1")
vessel = conn.space_center.active_vessel
vessel.control.activate_next_stage()

time.sleep(2)

while vessel.flight().surface_altitude > 10:
    if vessel.orbit.time_to_apoapsis < 1:
        print("Appoapsis at", vessel.flight().surface_altitude)
        break

vessel.control.activate_next_stage()

while vessel.flight().surface_altitude > 2500:
    continue

vessel.control.activate_next_stage()