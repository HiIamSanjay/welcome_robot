Multi-Emotion Face Simulator
A dynamic and expressive face simulator created with Python and Pygame. This application displays a face that can transition between multiple emotional states, each with unique animations. It's designed to run in fullscreen, making it ideal for projects involving character displays or virtual assistants on devices like the Raspberry Pi.

✨ Features
Neutral State: The default state features eyes that randomly look around and blink, with occasional changes in shape for a more lively feel.

Happy Emotion: A bouncy and smiling face with squinting eyes.

Angry Emotion: Shaking, slanted eyes and a jagged, frowning mouth.

Sad Emotion: Droopy eyelids, a downward gaze, a quivering frown, and a single, glistening tear.

Pamper Emotion: Glowing, bouncing eyes, a gentle smile, and cute blush marks.

Smooth Transitions: All emotions fade in and out smoothly.

⌨️ Controls
The application is controlled via keyboard shortcuts:

Key	Action
H	Toggle Happy emotion
A	Toggle Angry emotion
S	Toggle Sad emotion
P	Toggle Pamper emotion
ESC	Quit the application

Export to Sheets
🖥️ Requirements
Python 3.x

Pygame library

NumPy library

🍓 Setup and Installation on Raspberry Pi 4
These instructions will guide you through setting up the project on a Raspberry Pi 4 running the standard Raspberry Pi OS.

1. Open the Terminal
First, open a terminal window on your Raspberry Pi.

2. Update Your System
It's always a good practice to ensure your system's package list and installed packages are up to date.

Bash

sudo apt update
sudo apt upgrade -y
3. Install Dependencies
The script requires Pygame for the graphics and NumPy for some mathematical calculations.

Bash

pip3 install pygame numpy
4. Get the Code
Create a new file named eye_animation.py:

Bash

nano eye_animation.py
Copy the final, polished Python code from our conversation and paste it into the nano editor.

Save the file by pressing Ctrl+X, then Y, then Enter.

▶️ Running the Application
Navigate to the directory containing your script and run it using python3.

Bash

python3 eye_animation.py
The animation should now launch in fullscreen. Press the ESC key to exit at any time.

🧠 Understanding the Code (A Deeper Dive)
This section breaks down the core concepts used in the script for those who want to learn how it works.

The State Machine
At its heart, this program is a simple state machine. The current_emotion variable holds the program's current state (e.g., "neutral", "happy", "sad"). The main drawing section of the game loop is just a big if/elif/else block that checks this variable and calls the correct drawing function for the current state. This is a fundamental pattern for managing different modes or behaviors in games and applications.

Animation Principles
The animation is created using two primary techniques:

1. Progress-Based Animation
When an emotion begins, the emotion_anim_progress variable animates from 0.0 to 1.0 over a short period (EMOTION_ANIM_DURATION). This progress value is used to smoothly introduce an animation. For example, the sad eyes droop further as the progress increases:

Python

# The droop height depends directly on the animation progress
droop_height = (EYE_HEIGHT / cfg["droop_factor"]) * math.sin(math.pi / 2 * progress)
2. Time-Based Animation (The Power of Sine Waves)
For continuous animations like bouncing, glowing, or quivering, we use the current time combined with the math.sin() function. A sine wave naturally produces a smooth oscillating value between -1 and 1. By feeding it the ever-increasing current time, we get a continuous, smooth back-and-forth motion.

We control it like this: amplitude * math.sin(time * frequency)

amplitude: Controls how big the movement is (e.g., how high the mouth floats).

frequency: Controls how fast the movement is.

Here is how the sad mouth floats up and down:

Python

# The mouth's vertical position changes smoothly over time
mouth_float_offset = math.sin(current_time * cfg["mouth_float_speed"]) * cfg["mouth_float_amp"] * progress
Drawing with Pygame
The face is not a single image; it's composed of simple geometric shapes drawn each frame:

pygame.draw.rect: Used for the eyes. We pass the border_radius parameter to get the rounded corners.

pygame.draw.polygon: Used for more complex shapes like the smile and the tear. It takes a list of vertex points [(x1, y1), (x2, y2), ...] and connects them to form a filled shape.

pygame.draw.arc: Used to draw the simple curve of the frown and the happy mouth in "pamper" mode.

The Game Loop and Event Handling
The entire program runs inside a while running: loop, often called a "game loop". In each iteration of the loop, the program performs three key tasks:

Event Handling: It checks for user input using for event in pygame.event.get():. This is how it knows if you've pressed a key (like 'H' for happy) or tried to close the window.

State Update: It updates all the variables that control the animation, like emotion_anim_progress or the positions of the neutral eyes.

Drawing: It clears the screen (screen.fill(BG_COLOR)) and redraws everything in its new position based on the updated state variables. Finally, pygame.display.flip() shows the newly drawn frame to the user.

🔧 Configuration and Customization
The code is designed to be easily configurable. At the top of the eye_animation.py script, you will find several dictionaries that hold the animation parameters for each emotion. By changing these values, you can easily tweak the feel of the animations without altering the core logic.

Python

# Example:
SAD_CONFIG = {
    "gaze_x_amp": -20, 
    "gaze_y_amp": 30,
    "droop_factor": 1.5,
    "mouth_float_speed": 8, # Make the mouth quiver faster or slower
    "mouth_float_amp": 3,   # Make the mouth move up and down more or less
    "tear_start_progress": 0.3, 
    "tear_size": 40
}
📄 License
This project is open-source and can be considered under the MIT License. You are free to use, modify, and distribute it for any purpose.