import jinja2
import dead
import datetime
import numpy as np

# get current date and time
timestamp = datetime.datetime.utcnow()

# ----------------------------------------------------------------------------------------------------------------------#
# run simulator to get constant matrices

# copter intialisation
copter = dead.copter.copter.DeadCopter(mass=2,  # mass of entire copter in kg
                                       arm_length=0.225,  # half the distance between two opposite motors in m
                                       K_v=1000,  # Kv rating of the motors
                                       voltage_max=18,  # max voltage of battery in V
                                       voltage_min=15,  # min voltage of battery in V
                                       prop_diameter_in=10)  # propeller diameter in inches

# simulator intialisation
sampling_frequency = 125  # in Hz - sets refresh rate of ESCs
sim = dead.copter.simulator.Simulator(t_simulation=3, t_sampling=1/sampling_frequency)  # do not change

# system design
Ad, Bd, Cd, K, L, wide_G = sim.system_design(copter)  # wide_G has 6 columns
G = np.delete(wide_G, [3, 4, 5], 1)  # deletes last 3 columns of wide_G as last three elements of r are zeros


# reformat python matrices output syntax to C++ array syntax
def reformat_matrix_to_array(python_matrix):
    py_array_format = np.array(python_matrix)                    # arrays with negative values have different
    if str(py_array_format).count(" -") > 0:                     # output syntax to all positive arrays
        bracket_counter = str(py_array_format).count("]") - 2
        return str(py_array_format).replace("[", "{")\
            .replace("]", "}")\
            .replace("  ", ",")\
            .replace(" -", ", -")\
            .replace("}", "},", bracket_counter)\
            .replace(",,", ",")\
            .replace(",,", ",")\
            .replace(",,", ",")
    elif str(py_array_format).count("[-") > 0:                    # catch any other negatives in array
        bracket_counter = str(py_array_format).count("]") - 2
        return str(py_array_format).replace("[", "{")\
            .replace("]", "}")\
            .replace("  ", ",")\
            .replace(" -", ", -")\
            .replace("}", "},", bracket_counter)\
            .replace(",,", ",")\
            .replace(",,", ",")\
            .replace(",,", ",")
    else:                                                         # for all positive arrays
        bracket_counter = str(py_array_format).count("]") - 2
        return str(py_array_format).replace("[", "{") \
            .replace("]", "}") \
            .replace(" ", ",") \
            .replace("}", "},", bracket_counter) \
            .replace(",{", "{")\
            .replace(",,", ",")\
            .replace(",,", ",")\
            .replace(",,", ",")


_Ad = reformat_matrix_to_array(Ad)
_Bd = reformat_matrix_to_array(Bd)
_Cd = reformat_matrix_to_array(Cd)
_K = reformat_matrix_to_array(K)
_L = reformat_matrix_to_array(L)
_G = reformat_matrix_to_array(G)

# ----------------------------------------------------------------------------------------------------------------------#
# setup jinja environment

file_loader = jinja2.FileSystemLoader('../templates')
env = jinja2.Environment(loader=file_loader, autoescape=True)

# ----------------------------------------------------------------------------------------------------------------------#
# create "due.ino" from template

due_template = env.get_template('due.ino')
due_output = due_template.render(timestamp=timestamp,
                                 buzzer_pin=22)
due_output_path = "../../arduino/due/due.ino"
with open(due_output_path, "w") as fh:
    fh.write(due_output)

# ----------------------------------------------------------------------------------------------------------------------#
# create "fly.h" from template

fly_template = env.get_template('fly.h')
fly_output = fly_template.render(timestamp=timestamp,
                                 receiver_min=1065,
                                 receiver_max=1925,
                                 max_angle=10,
                                 discrete_A=_Ad,
                                 discrete_B=_Bd,
                                 discrete_C=_Cd,
                                 lqr_gain=_K,
                                 kf_gain=_L,
                                 equilibrium_G=_G)
fly_output_path = "../../arduino/due/fly.h"
with open(fly_output_path, "w") as fh:
    fh.write(fly_output)

# ----------------------------------------------------------------------------------------------------------------------#
# create "imu.h" from template

imu_template = env.get_template('imu.h')
imu_output = imu_template.render(timestamp=timestamp,
                                 sample_freq=sampling_frequency,
                                 imu_int_pin=6)
imu_output_path = "../../arduino/due/imu.h"
with open(imu_output_path, "w") as fh:
    fh.write(imu_output)

# ----------------------------------------------------------------------------------------------------------------------#
# create "receiver.h" from template

receiver_template = env.get_template('receiver.h')
receiver_output = receiver_template.render(timestamp=timestamp,
                                           receiver_pin=7,
                                           number_of_rx_channels=8,
                                           frame_change_time=3500,
                                           min_receiver_pwm=1070,
                                           max_receiver_pwm=1930,
                                           min_throttle_pwm=1150,
                                           max_throttle_pwm=1850,
                                           max_allowed_tilt_degrees=30,
                                           throttle_channel=3,
                                           rudder_channel=4,
                                           pitch_channel=2,
                                           roll_channel=1
                                           )
receiver_output_path = "../../arduino/due/receiver.h"
with open(receiver_output_path, "w") as fh:
    fh.write(receiver_output)

# ----------------------------------------------------------------------------------------------------------------------#
# create "actuators.h" from template

actuators_template = env.get_template('actuators.h')
actuators_output = actuators_template.render(timestamp=timestamp,
                                             front_left_pin=2,
                                             front_right_pin=3,
                                             back_left_pin=4,
                                             back_right_pin=5,
                                             zero_thrust_pwm=1000,
                                             idle_thrust_pwm=1130,
                                             absolute_min_pwm_value=800,
                                             absolute_max_pwm_value=2000,
                                             servo_range_min=0,
                                             servo_range_max=180)
actuators_output_path = "../../arduino/due/actuators.h"
with open(actuators_output_path, "w") as fh:
    fh.write(actuators_output)

# ----------------------------------------------------------------------------------------------------------------------#
