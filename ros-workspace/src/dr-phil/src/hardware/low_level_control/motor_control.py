from publishers import cmd_vel_pub

def motor_command(linear,angular):
    twist = Twist()
    twist.linear.x = linear 
    twist.angular.z = angular
    cmd_vel_pub.publish(twist)