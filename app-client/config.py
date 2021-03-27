import os, sys
from dotenv import load_dotenv

if getattr(sys, 'frozen', False):
    # If the application is run as a bundle, the PyInstaller bootloader
    # extends the sys module by a flag frozen=True and sets the app 
    # path into variable sys.executable
    env_path = os.path.join(os.path.dirname(sys.executable),".env")
else:
    env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),".env")

print("Loading .env file from: {}".format(env_path))
load_dotenv(dotenv_path=env_path)

ROS_WS_PORT = int(os.environ.get("ROS_WS_PORT","9090"))
ROS_WS_HOST = os.environ.get("ROS_WS_HOST","localhost")
RID = os.environ["RID"]
ROS_IP = os.environ["ROS_IP"]
ROS_SSH_USERNAME = os.environ["ROS_SSH_USERNAME"]
ROS_SSH_PASSWORD = os.environ["ROS_SSH_PASSWORD"]

# config = {
#   "apiKey": os.environ['API_KEY'],
#   "authDomain": os.environ['AUTH_DOMAIN'],
#   "databaseURL": os.environ['DATABASE_URL'],
#   "storageBucket": os.environ['STORAGE_BUCKET']
# }

# project_id = os.environ.get("PROJECT_ID",None)