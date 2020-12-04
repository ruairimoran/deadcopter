import jinja2
import datetime

timestamp = datetime.datetime.utcnow()

file_loader = jinja2.FileSystemLoader('')
env = jinja2.Environment(loader=file_loader, autoescape=True)

due_template = env.get_template('due.ino')
due_output = due_template.render(timestamp=timestamp)
due_output_path = "../../arduino/due/due.ino"
with open(due_output_path, "w") as fh:
    fh.write(due_output)

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