import dead

# copter intialisation
copter = dead.copter.copter.DeadCopter(mass=1.5, arm_length=0.225, K_v=1000,
                                       voltage_max=17, voltage_min=15, prop_diameter_in=9)

# simulator intialisation
sim = dead.copter.simulator.Simulator(t_simulation=1.5, t_sampling=0.01, omega_noise_multiplier=1e-4)

# System design
K, L = sim.system_design(copter)
print(f"K = {K} \n"
      f"L = {L}")

euler_state_cache, euler_state_hat_cache, state_cache, state_hat_cache = sim.simulate(copter)

sim.plot_all(euler_state_cache, euler_state_hat_cache, state_cache, state_hat_cache)