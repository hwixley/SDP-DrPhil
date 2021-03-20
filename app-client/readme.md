# App Client

This script is to be run on a remote with access to the internet, and when configured properly, will communicate with the robot via ssh proxy over websockets and communicate commands.

## Environment variables required
in order for the script to run it requires a .env file to be places in the script directory with the following variables set to appropriate values (replace all {}'s)

``` dosini
RID={test1}
ROS_WS_PORT={9090}
ROS_WS_HOST={localhost}
```