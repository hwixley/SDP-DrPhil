from publishers import cmd_vel_pub

def send_command(linear,angular):
    twist = Twist()
    twist.linear.x = linear 
    twist.angular.z = angular
    cmd_vel_pub.publish(twist)