import deadcopter.src.flight_controller as fc

# flight_controller initialisation
copter = fc.copter.DeadCopter(disturbance_level=1e-1,
                              mass=1.8,
                              arm_length=0.225,
                              K_v=1000,
                              voltage_max=16.8,
                              voltage_min=1.68,
                              prop_diameter_in=8)

# simulator initialisation
sim = fc.simulator.Simulator(t_simulation=1,
                             t_sampling=1/125,
                             measurement_noise_multiplier=1e-4)

# System design
Ad, Bd, Cd, K, L, G = sim.system_design(copter)
h = copter.hover_rpm
print(f"K = {K} \n"
      f"L = {L} \n"
      f"h = {h}")

sim_euler_state_cache, sim_euler_state_hat_cache, sim_control_action_cache, sim_state_cache, sim_state_hat_cache, sim_r_cache = sim.simulate(copter)

sim.plot_all(sim_euler_state_cache, sim_euler_state_hat_cache, sim_control_action_cache, sim_state_cache, sim_state_hat_cache, sim_r_cache)
