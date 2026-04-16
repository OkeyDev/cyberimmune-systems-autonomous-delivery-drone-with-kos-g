#! /usr/bin/bash

source default.env
rm -f mavproxy/MAVProxy/mav.parm
rm -f mavproxy/MAVProxy/mav.tlog
rm -f mavproxy/MAVProxy/mav.tlog.raw
rm -f ardupilot/eeprom.bin
rm -rf ardupilot/logs
rm -rf ardupilot/terrain
tmux kill-session -t flight_controller
tmux new-session -d -s flight_controller
tmux send-keys -t flight_controller "cd planner; ./APM_Planner.AppImage" Enter
tmux split-window -h -l 66% -t flight_controller
if [[ $* == *"--no-server"* ]]
	then
		tmux send-keys -t flight_controller "cd kos; ./cross-build.sh --target sim --mode offline --role inspector" Enter
		tmux split-window -h -l 50% -t flight_controller
		tmux send-keys -t flight_controller "cd kos; ./cross-build.sh --target sim --mode offline --role deliverer" Enter
	else
		tmux send-keys -t flight_controller "cd kos; ./cross-build.sh --target sim --mode online --role inspector --mqtt-username inspector --mqtt-password secret" Enter
		tmux split-window -h -l 50% -t flight_controller
		tmux send-keys -t flight_controller "cd kos; ./cross-build.sh --target sim --mode online --role deliverer --mqtt-username deliverer --mqtt-password secret" Enter
fi
tmux split-window -v -t flight_controller
if [[ $* == *"--with-obstacles"* ]]
	then
		tmux send-keys -t flight_controller "cd ardupilot; ./run_in_terminal_window.sh ArduCopter sitl/arducopter -S --model + --speedup 1 --slave 0 --serial5=tcp:5765:wait --serial6=tcp:5766:wait --serial7=tcp:5767:wait --defaults copter_obstacle.parm --sim-address=127.0.0.1 --home=60.0024590,27.8573238,0.00,13 -I0 --sysid 2" Enter
		tmux select-pane -t flight_controller:0.1
		tmux split-window -v -t flight_controller
		tmux send-keys -t flight_controller "cd ardupilot; ./run_in_terminal_window.sh ArduCopter sitl/arducopter -S --model + --speedup 1 --slave 0 --serial5=tcp:5775:wait --serial6=tcp:5776:wait --serial7=tcp:5777:wait --defaults copter_obstacle.parm --sim-address=127.0.0.1 --home=60.0025203,27.8573521,0.00,193 -I1 --sysid 1" Enter
	else
		tmux send-keys -t flight_controller "cd ardupilot; ./run_in_terminal_window.sh ArduCopter sitl/arducopter -S --model + --speedup 1 --slave 0 --serial5=tcp:5765:wait --serial6=tcp:5766:wait --serial7=tcp:5767:wait --defaults copter.parm --sim-address=127.0.0.1 --home=60.0024590,27.8573238,0.00,13 -I0 --sysid 2" Enter
		tmux select-pane -t flight_controller:0.1
		tmux split-window -v -t flight_controller
		tmux send-keys -t flight_controller "cd ardupilot; ./run_in_terminal_window.sh ArduCopter sitl/arducopter -S --model + --speedup 1 --slave 0 --serial5=tcp:5775:wait --serial6=tcp:5776:wait --serial7=tcp:5777:wait --defaults copter.parm --sim-address=127.0.0.1 --home=60.0025203,27.8573521,0.00,193 -I1 --sysid 1" Enter
fi
tmux select-pane -t flight_controller:0.0
tmux split-window -v -t flight_controller
tmux send-keys -t flight_controller "cd mavproxy/MAVProxy; mavproxy.py --out='127.0.0.1:14550' --out='127.0.0.1:14551' --master='tcp:127.0.0.1:5760' --master='tcp:127.0.0.1:5770' --sitl='127.0.0.1:5501' --sitl='127.0.0.1:5502'" Enter
tmux select-pane -t flight_controller:0.0
tmux attach -t flight_controller
