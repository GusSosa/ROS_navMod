# 2d-spine-control-hardware
Code for the test setup for the 2D tensegrity spine vertebra. Includes ROS / computer vision, code for the PSoC microcontroller, and controller calculations.

Usage:

For writing the PSoC code - use PSoC creator. No particular instructions there.
This repo has a nice .gitignore so you can open the workspace directly in this subfolder.

For ROS / actually operating the test:

	Writing the inverse kinematics commands: do the following, in order. Be sure to source devel/setup.bash in each.
		1) Plug in the PSoC and run >> dmesg. Look for the device name. You'll see a line like
			"ttyACM0 : USB ACM Device"
			the name of the PSoC device file is /dev/ttyACM0 in this case. Sub in below if different.
		2) open a new terminal >> roscore
		3) open a new terminal >> rosrun spine_controller serial_rx_echo.py /dev/ttyACM0
		4) open a new terminal >> rosrun spine_controller serial_tx_fromtopic.py /dev/ttyACM0 invkin_tx_commands
		5) open a new terminal >> rosrun spine_controller invkin_tx_totopic.py path-to-data-file

		For the last line, an example would be, if you're in the ...spine_controller/src directory,
		>> rosrun spine_controller invkin_tx_totopic.py ../data/invkin_results_2d_28-Nov-2018_15-25-20.csv 

		You'll see output in the serial_rx_echo window.
		To shut down: Ctrl-C in each window, in the reverse order of startup.
		To manually control the PSoC / send it commands, at any time, open another terminal and source devel/setup.bash, then
			>> rosrun spine_controller serial_tx_cmdline.py /dev/ttyACM0
			(hint: sending 'w' prints a welcome message from the PSoC w/ list of commands.)