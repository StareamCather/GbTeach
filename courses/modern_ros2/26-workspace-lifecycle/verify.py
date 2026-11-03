"""Load the course's private install environment before checking its package."""

import os

if __name__ == "__main__":
    os.execv("/bin/bash", ["/bin/bash", "-c", ". ros_ws/install/local_setup.bash && exec python3 lab.py"])
