from os import path




def setup_env(env_path):

    with open(env_path,"w+") as f:
        robot_id = input(">Robot ID:")
        robot_ip = input(">Robot IP address:")
        robot_ssh_user = input(">SSH username:")
        robot_ssh_pass = input(">SSH password:")

        f.write("RID={}\nROS_IP={}\nROS_SSH_USERNAME={}\nROS_SSH_PASSWORD={}\n".format(robot_id, 
                            robot_ip,
                            robot_ssh_user,
                            robot_ssh_pass))
        f.write("ROS_WS_PORT=9090\n")
        f.write("ROS_WS_HOST=localhost\n")
