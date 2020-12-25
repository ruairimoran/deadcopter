# import dead
#
# # copter intialisation
# copter = dead.copter.copter.DeadCopter(disturbance_level=1e-3,
#                                        mass=1.85, arm_length=0.27, K_v=700,
#                                        voltage_max=11.1, voltage_min=1.11, prop_diameter_in=11)
#
# # simulator intialisation
# sim = dead.copter.simulator.Simulator(t_simulation=3, t_sampling=1/238, measurement_noise_multiplier=1e-4)
#
# # System design
# Ad, Bd, Cd, K_x, K_z, L, G = sim.system_design(copter)
# print(f"K_x = {K_x} \n"
#       f"K_z = {K_z} \n"
#       f"L = {L}")
#
# euler_state_cache, euler_state_hat_cache, state_cache, state_hat_cache = sim.simulate(copter)
#
# sim.plot_all(euler_state_cache, euler_state_hat_cache, state_cache, state_hat_cache)