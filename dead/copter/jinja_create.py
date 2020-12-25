import jinja2
import dead
import datetime

# get current date and time
timestamp = datetime.datetime.utcnow()

#----------------------------------------------------------------------------------------------------------------------#
# run simulator to get constant matrices

# copter intialisation
copter = dead.copter.copter.DeadCopter(disturbance_level=1e-3,
                                       mass=1.85,
                                       arm_length=0.27,
                                       K_v=700,
                                       voltage_max=11.1,
                                       voltage_min=1.11,
                                       prop_diameter_in=11)

# simulator intialisation
sim = dead.copter.simulator.Simulator(t_simulation=3, t_sampling=1/238, measurement_noise_multiplier=1e-4)

# system design
Ad, Bd, Cd, K_x, K_z, L, G = sim.system_design(copter)

#----------------------------------------------------------------------------------------------------------------------#
# setup jinja environment

file_loader = jinja2.FileSystemLoader('../templates')
env = jinja2.Environment(loader=file_loader, autoescape=True)

#----------------------------------------------------------------------------------------------------------------------#
# create "due.ino" from template

due_template = env.get_template('due.ino')
due_output = due_template.render(timestamp=timestamp)
due_output_path = "../../arduino/due/due.ino"
with open(due_output_path, "w") as fh:
    fh.write(due_output)

#----------------------------------------------------------------------------------------------------------------------#
# create "fly.h" from template

fly_template = env.get_template('fly.h')
fly_output = fly_template.render(timestamp=timestamp,
                                 discrete_A=Ad,
                                 discrete_B=Bd,
                                 discrete_C=Cd,
                                 lqr_K_x_gain=K_x,
                                 lqr_K_z_gain=K_z,
                                 kf_gain=L,
                                 equilibrium_G=G)
fly_output_path = "../../arduino/due/fly.h"
with open(fly_output_path, "w") as fh:
    fh.write(fly_output)

#----------------------------------------------------------------------------------------------------------------------#
# create "imu.h" from template

imu_template = env.get_template('imu.h')
imu_output = imu_template.render(timestamp=timestamp)
imu_output_path = "../../arduino/due/imu.h"
with open(imu_output_path, "w") as fh:
    fh.write(imu_output)

#----------------------------------------------------------------------------------------------------------------------#
# create "receiver.h" from template

receiver_template = env.get_template('receiver.h')
receiver_output = receiver_template.render(timestamp=timestamp,
                                           receiver_pin=7,
                                           number_of_rx_channels=8,
                                           frame_change_time=5000,
                                           min_receiver_pwm=1000,
                                           max_receiver_pwm=2000,
                                           max_allowed_tilt_degrees=30,
                                           throttle_channel=1,
                                           rudder_channel=2,
                                           pitch_channel=3,
                                           roll_channel=4
                                           )
receiver_output_path = "../../arduino/due/receiver.h"
with open(receiver_output_path, "w") as fh:
    fh.write(receiver_output)

#----------------------------------------------------------------------------------------------------------------------#
# create "actuators.h" from template

actuators_template = env.get_template('actuators.h')
actuators_output = actuators_template.render(timestamp=timestamp,
                                             front_left_pin=2,
                                             front_right_pin=3,
                                             back_left_pin=4,
                                             back_right_pin=5,
                                             zero_thrust_pwm=1000,
                                             idle_thrust_pwm=1150,
                                             absolute_min_pwm_value=1000,
                                             absolute_max_pwm_value=2000,
                                             servo_range_min=0,
                                             servo_range_max=180)
actuators_output_path = "../../arduino/due/actuators.h"
with open(actuators_output_path, "w") as fh:
    fh.write(actuators_output)

#----------------------------------------------------------------------------------------------------------------------#
