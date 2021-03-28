import os, sys
from dotenv import load_dotenv
from first_time_setup import setup_env





def setup():
    """Sets up env variables and returns them all as a tuple

    Returns:
        [type]: [description]
    """
    if getattr(sys, 'frozen', False):
        # If the application is run as a bundle, the PyInstaller bootloader
        # extends the sys module by a flag frozen=True and sets the app 
        # path into variable sys.executable
        env_path = os.path.join(os.path.dirname(sys.executable),".env")
    elif __file__:
        env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),".env")

    if not os.path.isfile(env_path):
        print("Executing fist time setup")
        setup_env(env_path)

    print("Loading .env file from: {}".format(env_path))
    load_dotenv(dotenv_path=env_path)

    ROS_WS_PORT = int(os.environ.get("ROS_WS_PORT","9090"))
    ROS_WS_HOST = os.environ.get("ROS_WS_HOST","localhost")
    RID = os.environ["RID"]
    ROS_IP = os.environ["ROS_IP"]
    ROS_SSH_USERNAME = os.environ["ROS_SSH_USERNAME"]
    ROS_SSH_PASSWORD = os.environ["ROS_SSH_PASSWORD"]

    return ROS_WS_PORT,ROS_WS_HOST,RID,ROS_IP,ROS_SSH_USERNAME,ROS_SSH_PASSWORD