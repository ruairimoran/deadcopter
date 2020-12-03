import jinja2
import datetime

timestamp = datetime.datetime.utcnow()

file_loader = jinja2.FileSystemLoader('')
env = jinja2.Environment(loader=file_loader, autoescape=True)

actuators_template = env.get_template('actuators.h')
actuators_output = actuators_template.render(timestamp=timestamp,
                                             FrontRight_pin=2,
                                             FrontLeft_pin=3,
                                             BackLeft_pin=4,
                                             BackRight_pin=5)
actuators_output_path = "../due/actuators.h"
with open(actuators_output_path, "w") as fh:
    fh.write(actuators_output)