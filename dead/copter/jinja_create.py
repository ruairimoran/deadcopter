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

file_loader = jinja2.FileSystemLoader('')
env = jinja2.Environment(loader=file_loader, autoescape=True)

#----------------------------------------------------------------------------------------------------------------------#
# create "due.ino" from template

due_template = env.get_template('dead/templates/due.ino')
due_output = due_template.render(timestamp=timestamp)
due_output_path = "arduino/due/due.ino"
with open(due_output_path, "w") as fh:
    fh.write(due_output)

#----------------------------------------------------------------------------------------------------------------------#
# create "observer.h" from template

#----------------------------------------------------------------------------------------------------------------------#
# create "controller.h" from template

#----------------------------------------------------------------------------------------------------------------------#
# create "receiver.h" from template

receiver_template = env.get_template('receiver.h')
receiver_output = receiver_template.render(timestamp=timestamp)
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
