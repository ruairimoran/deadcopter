import dead

# copter intialisation
copter = dead.copter.copter.DeadCopter(disturbance_level=1e-1,
                                       mass=1.8,
                                       arm_length=0.225,
                                       K_v=1000,
                                       voltage_max=16.8,
                                       voltage_min=1.68,
                                       prop_diameter_in=8)

# simulator intialisation
sim = dead.copter.simulator.Simulator(t_simulation=1,
                                      t_sampling=1/125,
                                      measurement_noise_multiplier=1e-7)

# System design
Ad, Bd, Cd, K, L, G = sim.system_design(copter)
h = copter.hover_rpm
print(f"K = {K} \n"
      f"L = {L} \n"
      f"h = {h}")

euler_state_cache, euler_state_hat_cache, control_action_cache, state_cache, state_hat_cache = sim.simulate(copter)

sim.plot_all(euler_state_cache, euler_state_hat_cache, control_action_cache, state_cache, state_hat_cache)
