#!/bin/bash
session="martian-mines"

tmux new-session -d -s $session

# Window 0: Start Docker and enter the container
window=0
tmux rename-window -t $session:$window 'docker-main'
tmux send-keys -t $session:$window 'cd /home/hf/Documents/martian-mines/docker && sudo docker compose up -d && sudo docker compose exec martian-mines /bin/bash' C-m

# Window 1: Additional Docker session
window=1
tmux new-window -t $session:$window -n 'docker-extra'
tmux send-keys -t $session:$window 'sleep 10 && cd /home/hf/Documents/martian-mines/docker && sudo docker compose exec martian-mines /bin/bash' C-m

# Window 2: MAVLink Router for PX4 (unchanged)
window=2
tmux new-window -t $session:$window -n 'mavlink'
tmux send-keys -t $session:$window 'sudo mavlink-routerd /dev/ttyTHS0:921600 -e 127.0.0.1:14550 -e 10.42.1.1:14550' C-m

# Window 3: ROS 2 environment setup and launch
window=3
tmux new-window -t $session:$window -n 'ros2-main'
tmux send-keys -t $session:$window 'cd /home/hf/Documents/martian-mines && source install/setup.bash && ros2 launch martian_mines rviz.launch.py' C-m

tmux attach-session -t $session
