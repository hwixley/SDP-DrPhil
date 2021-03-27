# App Client

This script is to be run on a remote with access to the internet, and when configured properly, will communicate with the robot via ssh proxy over websockets and communicate commands.

## Installing the dependencies

pip3 install -r requirements.txt

## Running the app-client script

cmd_listener.py is the main entry point. It tries to establish an ssh tunnel by default. To make it run on localhost, use the optional argument.
```
optional arguments:
-l, --localhost  Runs the script on localhost
```
## App Client installer build

This creates a standalone Linux executable for the app-client

Dependency: pyinstaller

In order to build the executable, run:
```
pyinstaller --clean --onefile \
            --hidden-import=pkg_resources \
            --name app-client \
            --distpath bundled-app/ \
            --workpath bundled-app/build/ \
            --specpath bundled-app/ \
            cmd_listener.py
```
The bundled app executable can then be found in bundled-app/app-client.
In order for the executable to run it requires a configured .env file placed in the same directory.

## Environment variables required
in order for the script to run it requires a .env file to be placed in the script directory with the following variables set to appropriate values (replace all {}'s)

``` dosini
RID={test1}
ROS_WS_PORT={9090}
ROS_WS_HOST={localhost}
ROS_IP=
ROS_SSH_USERNAME=
ROS_SSH_PASSWORD=
```