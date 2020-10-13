import numpy as np
from subprocess import Popen, PIPE

speeds = np.linspace(60, 511, 512-60)

for i in range(0, len(speeds)):
	p = Popen(['python3 uart_speed_step.py'], stdin=PIPE, shell=True)
	test_name = "cmd-"+str(speeds[i])+"\n"
	use_default_conf = "y\n"
	# p.communicate(input=test_name.encode())
	speed = str(speeds[i])+"\n"
	# p.communicate(input=speed.encode())
	cmd = test_name+use_default_conf+speed
	p.communicate(input=cmd.encode())