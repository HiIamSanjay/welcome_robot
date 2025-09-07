# #!/bin/bash

# # This script waits for the system to settle and the network to connect,
# # then launches the full robot AI suite, each in a separate terminal window.

# echo "Startup script initiated. Waiting for 10 seconds for system to settle..."
# sleep 10

# # This line explicitly tells GUI applications to use the main physical screen.
# export DISPLAY=:0

# # --- Wait for Network Connection ---
# while ! ping -c 1 -W 1 8.8.8.8 > /dev/null; do
#     echo "Waiting for network connection..."
#     sleep 1
# done

# echo "✅ Network is online. Starting robot applications in new terminals..."

# # --- Define the base command for convenience ---
# SETUP_CMD="cd /home/welcome/Desktop/Robot_files; source /home/welcome/myvenv/bin/activate;"

# # --- Launch Applications in Separate Terminals ---
# gnome-terminal --title="Camera Manager" -- /bin/bash -c "$SETUP_CMD python3 camera_manager.py; exec bash" &
# sleep 2

# gnome-terminal --title="Robot Server" -- /bin/bash -c "$SETUP_CMD python3 robot_server.py; exec bash" &
# sleep 1

# gnome-terminal --title="Eye Animation" -- /bin/bash -c "$SETUP_CMD python3 eye_animation.py; exec bash" &

# gnome-terminal --title="Main GUI" -- /bin/bash -c "$SETUP_CMD python3 main_screen_GUI.py; exec bash" &

# echo "🚀 All robot applications launched."

# # -------------------------------------------------------------------
# # ### THE FIX IS HERE ###
# # This command is crucial. It keeps this script running, which allows the
# # background GUI processes (the new terminals) to remain stable.
# wait
# # -------------------------------------------------------------------