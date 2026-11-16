import os
import time

class TmuxSend:

    # listwindows = [ 'windowname', ... ]
    def __init__(self, sessionname, listwindows):
        self.nwindows = len(listwindows)
        self.sessionname = sessionname
        os.system(f'tmux -2 new-session -d -s {self.sessionname}')  # Start tmux session

        # Rename the first window
        os.system(f'tmux rename-window -t {self.sessionname}:0 \'{listwindows[0]}\'')   # window 0
        for i in range(1, self.nwindows):
            os.system(f'tmux new-window -t {self.sessionname}:{i} -n \'{listwindows[i]}\'')    

    def ros2_launch(self, wid, package, launch_file, params=''):
        """
        Run a ROS 2 launch file in a specified tmux window.
        This is equivalent to 'ros2 launch <package> <launch_file>.launch.py'
        """
        os.system(f'tmux send-keys -t {self.sessionname}:{wid} "cd $ROS_WS/src/{package}" C-m')
        os.system(f'tmux send-keys -t {self.sessionname}:{wid} "ros2 launch {package} {launch_file}.launch.py {params}" C-m')

    def ros2_node_kill(self, node_name):
        """
        Kill a ROS 2 node.
        Equivalent to 'ros2 node kill <node_name>'
        """
        wid = 0  # Specify the tmux window where the command will run
        os.system(f'tmux send-keys -t {self.sessionname}:{wid} "ros2 node kill {node_name}" C-m')

    def python(self, wid, mdir, script_name, params=''):
        """
        Run a Python script in a specified tmux window.
        """
        os.system(f'tmux send-keys -t {self.sessionname}:{wid} "cd $MARRTINOROBOT2_APPS/{mdir}" C-m')
        os.system(f'tmux send-keys -t {self.sessionname}:{wid} "python3 {script_name} {params}" C-m')

    def cmd(self, wid, command, sleeptime=0.1, blocking=False):
        """
        Send a custom command to a specified tmux window.
        """
        if blocking:
            os.system(f'tmux send-keys -t {self.sessionname}:{wid} "{command}; sleep 1; tmux wait-for -S tmux-end" C-m')
            self.waitfor('tmux-end')            
        else:
            os.system(f'tmux send-keys -t {self.sessionname}:{wid} "{command}" C-m')
            time.sleep(sleeptime)
    

    def waitfor(self, wait_label):
        """
        Wait for a tmux synchronization event.
        """
        os.system(f'tmux wait-for {wait_label}')

    def killall(self, wid):
        """
        Kill all processes in a tmux window.
        """
        self.send_ctrl_c(wid)
        time.sleep(3)
        self.send_ctrl_backslash(wid)

    def send_ctrl_c(self, wid):
        """
        Send Ctrl+C to the specified tmux window to terminate the current process.
        """
        os.system(f'tmux send-keys -t {self.sessionname}:{wid} C-c')

    def send_ctrl_backslash(self, wid):
        """
        Send Ctrl+\ to the specified tmux window to forcefully kill the current process.
        """
        os.system(f'tmux send-keys -t {self.sessionname}:{wid} C-\\')

    def quitall(self, window_range=None):
        """
        Kill all processes in the given range of tmux windows.
        """
        if window_range is None:
            window_range = range(0, self.nwindows)

        # Send Ctrl+C to all windows in the range
        for i in window_range:
            self.send_ctrl_c(i)
            time.sleep(1)

        # Send Ctrl+\ to all windows in the range
        for i in window_range:
            self.send_ctrl_backslash(i)
            time.sleep(1)
