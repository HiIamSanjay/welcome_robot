# -*- coding: utf-8 -*-
"""
Multi-Emotion Face Simulator (Modified for IPC)
------------------------------------------------
Receives commands via UDP to change emotional states.
"""
import pygame
import time
import random
import numpy as np
import math
import socket # MODIFICATION: Import socket library

# --- Pygame Initialization ---
pygame.init()
info = pygame.display.Info()
WIDTH, HEIGHT = info.current_w, info.current_h
screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN)
pygame.display.set_caption("Multi-Emotion Face Simulator")
clock = pygame.time.Clock()

# ### MODIFICATION START ###
# --- IPC Configuration ---
UDP_HOST = '127.0.0.1'  # Listen only on the local machine
UDP_PORT = 12345        # The port for communication

# --- Socket Setup ---
# Create and bind the UDP socket to listen for commands
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((UDP_HOST, UDP_PORT))
server_socket.setblocking(False)  # IMPORTANT: Prevents the animation from freezing while waiting for a message
print(f"✅ Eye Animation is listening for commands on port {UDP_PORT}")
# ### MODIFICATION END ###

# --- Configuration ---

# Colors
HAPPY_COLOR = (0, 255, 0) # Green for happy emotion
EYE_COLOR = (0, 136, 255) # Blue for neutral/default
LID_COLOR = (0, 0, 0)
BG_COLOR = (0, 0, 0)
SMILE_COLOR = HAPPY_COLOR # Smile will also be green
ANGRY_COLOR = (255, 0, 0)
PAMPER_COLOR = (255, 100, 100)
TEAR_COLOR = (173, 216, 230)

# Eye Parameters
EYE_WIDTH, EYE_HEIGHT = 180, 180
EYE_CORNER_RADIUS = 45
EYE_Y_POS = HEIGHT // 2 - EYE_HEIGHT // 2 - 100
EYE_SPACING = 300

# Animation Timings
EMOTION_ANIM_DURATION = 0.3
EMOTION_HOLD_DURATION = 3.0
BLINK_DURATION = 0.1

# Emotion-Specific Animation Parameters
HAPPY_CONFIG = {
    "bounce_speed": 5, "bounce_amp": 8,
    "scale_speed": 4, "scale_amp": 0.08,
    "squint": 0.4
}
SAD_CONFIG = {
    "gaze_x_amp": -20, "gaze_y_amp": 30,
    "droop_factor": 1.5,
    "mouth_float_speed": 8, "mouth_float_amp": 3,
    "tear_start_progress": 0.3, "tear_size": 40
}
PAMPER_CONFIG = {
    "glow_speed": 4, "glow_amp": 0.1,
    "bounce_speed": 3, "bounce_amp": 6,
    "stretch_speed": 6, "stretch_amp": 0.05
}

# --- State Management Variables ---
current_emotion = "neutral" 
emotion_anim_progress = 0
emotion_start_time = 0
mode = 1
mode_start_time = time.time()
next_mode_time = time.time() + random.uniform(8, 15)

# ### MODIFICATION START ###
is_nodding = False # New state for nodding behavior
# ### MODIFICATION END ###

# Neutral State Movement
positions = {"center": 0, "left": -40, "right": 40}
movement_order = ["center", "left", "center", "right", "center"]
current_index = 0
current_position, start_position, target_position = 0, 0, 0
moving = False
move_start_time = 0
last_change_time = time.time()
move_duration = 0.4
rest_duration = random.uniform(3, 6)

# Neutral State Blinking
blinking = False
lid_progress = 0
blink_start_time = time.time()
blink_interval = random.uniform(3, 6)


# --- Data Generation Functions ---
# (No changes needed in data generation functions)
def generate_smile_data(progress, y_offset=0):
    if progress <= 0: return None
    num_points, max_thickness, min_ratio = 50, 20, 0.25
    x_vals = np.linspace(-1, 1, num_points)
    y_center = -(x_vals**4 * 0.8 + x_vals**2 * 0.2 - 0.7) * 50 * progress
    thickness_profile = (1 - x_vals**2) * (1 - min_ratio) + min_ratio
    thickness = thickness_profile * max_thickness * progress
    y_upper, y_lower = y_center - thickness/2, y_center + thickness/2
    smile_width, smile_y_pos = 320, HEIGHT // 2 + 100 + y_offset
    upper_points = [(WIDTH//2 + x*smile_width/2, smile_y_pos + y) for x,y in zip(x_vals, y_upper)]
    lower_points = [(WIDTH//2 + x*smile_width/2, smile_y_pos + y) for x,y in zip(x_vals, y_lower)]
    polygon_points = upper_points + lower_points[::-1]
    cap_radius = thickness[0] / 2
    left_cap_pos = (upper_points[0][0], upper_points[0][1] + cap_radius)
    right_cap_pos = (upper_points[-1][0], upper_points[-1][1] + cap_radius)
    end_caps_data = [(left_cap_pos, cap_radius), (right_cap_pos, cap_radius)]
    return polygon_points, end_caps_data

def generate_angry_mouth_points(progress, offset=(0,0)):
    if progress <= 0: return None
    points = [(-1, 0.5), (-0.5, -0.5), (0, 0.5), (0.5, -0.5), (1, 0.5)]
    mouth_width, mouth_height, mouth_y_pos = 150, 25 * progress, HEIGHT // 2 + 80
    line_points = [(WIDTH/2 + p[0]*mouth_width/2 + offset[0], mouth_y_pos + p[1]*mouth_height + offset[1]) for p in points]
    return line_points


# --- Component Drawing Functions ---
# (No changes needed in component drawing functions)
def draw_rounded_eye(surface, x, y, width, height, radius, color=EYE_COLOR, offset=(0,0)):
    pygame.draw.rect(surface, color, (x + offset[0], y + offset[1], width, height), border_radius=radius)

def draw_circle_eye(surface, x, y, diameter, color=EYE_COLOR, offset=(0,0)):
    pygame.draw.ellipse(surface, color, (x + offset[0], y + offset[1], diameter, diameter))

def draw_lids(surface, x, y, width, height, blink_progress=0, squint_progress=0):
    if blink_progress > 0:
        lid_height = int(height * blink_progress)
        pygame.draw.rect(surface, LID_COLOR, (x, y, width, lid_height), border_radius=EYE_CORNER_RADIUS)
        pygame.draw.rect(surface, LID_COLOR, (x, y + height - lid_height, width, lid_height), border_radius=EYE_CORNER_RADIUS)
    if squint_progress > 0:
        squint_height = int(height * squint_progress)
        pygame.draw.rect(surface, LID_COLOR, (x, y + height - squint_height, width, squint_height), border_radius=EYE_CORNER_RADIUS)

def draw_frown(surface, x, y, width, height, progress, color, offset=(0,0)):
    if progress <= 0: return
    frown_rect = pygame.Rect(x - width//2 + offset[0], y + offset[1], width, height)
    pygame.draw.arc(surface, color, frown_rect, 0, math.pi, int(10 * progress))

def draw_tear(surface, x, y, size, progress, offset=(0,0)):
    cfg = SAD_CONFIG
    if progress < cfg["tear_start_progress"]: return
    current_size = size * ((progress - cfg["tear_start_progress"]) / (1 - cfg["tear_start_progress"]))
    if current_size <= 0: return
    glisten_factor = abs(math.sin(time.time() * 5)) * 0.5 + 0.75
    glisten_color = (min(255, int(TEAR_COLOR[0] * glisten_factor)),
                     min(255, int(TEAR_COLOR[1] * glisten_factor)),
                     min(255, int(TEAR_COLOR[2] * glisten_factor)))
    body_height, radius = current_size, current_size / 2
    triangle_points = [
        (x + offset[0], y + offset[1]),
        (x - radius + offset[0], y + body_height * 0.7 + offset[1]),
        (x + radius + offset[0], y + body_height * 0.7 + offset[1]),
    ]
    circle_center = (x + offset[0], y + body_height * 0.7 + offset[1])
    pygame.draw.polygon(surface, glisten_color, triangle_points)
    pygame.draw.circle(surface, glisten_color, circle_center, radius)

def draw_blush(surface, x, y, radius_x, radius_y):
    pygame.draw.ellipse(surface, PAMPER_COLOR, (x, y, radius_x, radius_y))


# --- Full Emotion Drawing Functions ---

# ### MODIFICATION START ###
# Modified draw_neutral_emotion to handle nodding
def draw_neutral_emotion():
    """Draws the default neutral face, adding a nod if is_nodding is True."""
    y_offset = 0
    # If nodding is active, calculate a vertical offset using a sine wave
    if is_nodding:
        y_offset = 20 * math.sin(time.time() * 5) # 20 is amplitude, 10 is speed

    left_style, right_style = 'normal', 'normal'
    current_move = movement_order[current_index]
    if mode == 2: left_style, right_style = 'enlarged', 'circle'
    elif current_move == 'left': left_style, right_style = 'circle', 'enlarged'
    elif current_move == 'right': left_style, right_style = 'enlarged', 'circle'
    
    for i, style in enumerate([left_style, right_style]):
        dx = -EYE_SPACING if i == 0 else EYE_SPACING
        # Add the main y_offset to all eye positions
        base_x = WIDTH // 2 + dx - EYE_WIDTH // 2 + current_position
        base_y = EYE_Y_POS + y_offset 

        if style == 'normal':
            draw_rounded_eye(screen, base_x, base_y, EYE_WIDTH, EYE_HEIGHT, EYE_CORNER_RADIUS)
            draw_lids(screen, base_x, base_y, EYE_WIDTH, EYE_HEIGHT, lid_progress)
        elif style == 'enlarged':
            ew, eh = int(EYE_WIDTH * 1.3), int(EYE_HEIGHT * 1.3)
            ex, ey = base_x - (ew - EYE_WIDTH) // 2, base_y - (eh - EYE_HEIGHT) // 2
            draw_rounded_eye(screen, ex, ey, ew, eh, EYE_CORNER_RADIUS)
            draw_lids(screen, ex, ey, ew, eh, lid_progress)
        elif style == 'circle':
            d = min(EYE_WIDTH, EYE_HEIGHT)
            ex, ey = base_x + (EYE_WIDTH - d) // 2, base_y + (EYE_HEIGHT - d) // 2
            draw_circle_eye(screen, ex, ey, d)
            draw_lids(screen, ex, ey, d, d, lid_progress)
# ### MODIFICATION END ###

def draw_happy_emotion(current_time, start_time, progress):
    """Draws the bouncy, smiling happy face."""
    cfg = HAPPY_CONFIG
    time_elapsed = current_time - start_time
    bounce_offset = cfg["bounce_amp"] * math.sin(time_elapsed * cfg["bounce_speed"]) * progress
    scale_effect = 1 + cfg["scale_amp"] * math.sin(time_elapsed * cfg["scale_speed"]) * progress
    
    new_width, new_height = int(EYE_WIDTH * scale_effect), int(EYE_HEIGHT * scale_effect)

    for dx in [-EYE_SPACING, EYE_SPACING]:
        base_x = WIDTH // 2 + dx - EYE_WIDTH // 2
        new_x = base_x - (new_width - EYE_WIDTH) // 2
        new_y = EYE_Y_POS - (new_height - EYE_HEIGHT) // 2
        draw_rounded_eye(screen, new_x, new_y + bounce_offset, new_width, new_height, EYE_CORNER_RADIUS, color=HAPPY_COLOR)
        draw_lids(screen, new_x, new_y + bounce_offset, new_width, new_height, squint_progress=progress * cfg["squint"])

    smile_data = generate_smile_data(progress, y_offset=bounce_offset)
    if smile_data:
        poly_points, end_caps = smile_data
        pygame.draw.polygon(screen, SMILE_COLOR, poly_points)
        for pos, radius in end_caps: pygame.draw.circle(screen, SMILE_COLOR, pos, radius)

def draw_angry_emotion(progress):
    """Draws the shaking, slanted-eye angry face."""
    shake_offset = (random.uniform(-2, 2), random.uniform(-2, 2)) if progress > 0.5 else (0,0)
    for i, dx in enumerate([-EYE_SPACING, EYE_SPACING]):
        eye_x = WIDTH // 2 + dx - EYE_WIDTH // 2
        slant = 60 * progress
        x_off, y_off = shake_offset
        if i == 0: # Left Eye
            p1, p2 = (eye_x + x_off, EYE_Y_POS + y_off), (eye_x + x_off + EYE_WIDTH, EYE_Y_POS + y_off + slant)
            p3, p4 = (eye_x + x_off + EYE_WIDTH, EYE_Y_POS + y_off + EYE_HEIGHT), (eye_x + x_off, EYE_Y_POS + y_off + EYE_HEIGHT)
        else: # Right Eye
            p1, p2 = (eye_x + x_off, EYE_Y_POS + y_off + slant), (eye_x + x_off + EYE_WIDTH, EYE_Y_POS + y_off)
            p3, p4 = (eye_x + x_off + EYE_WIDTH, EYE_Y_POS + y_off + EYE_HEIGHT), (eye_x + x_off, EYE_Y_POS + y_off + EYE_HEIGHT)
        pygame.draw.polygon(screen, ANGRY_COLOR, [p1, p2, p3, p4])

    mouth_points = generate_angry_mouth_points(progress, offset=shake_offset)
    if mouth_points:
        pygame.draw.lines(screen, ANGRY_COLOR, False, mouth_points, 12)

def draw_sad_emotion(current_time, start_time, progress):
    """Draws the sad face with a downward gaze, droopy lids, and a tear."""
    cfg = SAD_CONFIG
    gaze_x = cfg["gaze_x_amp"] * progress
    gaze_y = cfg["gaze_y_amp"] * progress
    gaze_offset = (gaze_x, gaze_y)
    
    droop_height = (EYE_HEIGHT / cfg["droop_factor"]) * math.sin(math.pi / 2 * progress)

    for dx in [-EYE_SPACING, EYE_SPACING]:
        eye_x = WIDTH // 2 + dx - EYE_WIDTH // 2
        draw_rounded_eye(screen, eye_x, EYE_Y_POS, EYE_WIDTH, EYE_HEIGHT, EYE_CORNER_RADIUS, offset=gaze_offset)
        lid_rect = pygame.Rect(eye_x + gaze_offset[0], EYE_Y_POS + gaze_offset[1], EYE_WIDTH, droop_height)
        pygame.draw.rect(screen, LID_COLOR, lid_rect, border_top_left_radius=EYE_CORNER_RADIUS, border_top_right_radius=EYE_CORNER_RADIUS)

    mouth_float_offset = math.sin(current_time * cfg["mouth_float_speed"]) * cfg["mouth_float_amp"] * progress
    frown_x, frown_y = WIDTH // 2, EYE_Y_POS + EYE_HEIGHT + 90
    draw_frown(screen, frown_x, frown_y + mouth_float_offset, 100, 50, progress, EYE_COLOR, offset=gaze_offset)
    
    tear_x = WIDTH // 2 - EYE_SPACING
    tear_y = EYE_Y_POS + EYE_HEIGHT + gaze_offset[1]
    draw_tear(screen, tear_x, tear_y, cfg["tear_size"], progress, offset=(gaze_offset[0], 0))

def draw_pamper_emotion(current_time, start_time, progress):
    """Draws the pamper face with glowing eyes, blush, and a happy mouth."""
    cfg = PAMPER_CONFIG
    time_elapsed = current_time - start_time
    
    for i, dx in enumerate([-EYE_SPACING, EYE_SPACING]):
        eye_x = WIDTH // 2 + dx - EYE_WIDTH // 2
        
        glow_scale = 1 + cfg["glow_amp"] * math.sin(time_elapsed * cfg["glow_speed"])
        bounce_offset = cfg["bounce_amp"] * math.sin(time_elapsed * cfg["bounce_speed"])
        stretch = 1 + cfg["stretch_amp"] * math.sin(time_elapsed * cfg["stretch_speed"])
        
        new_width, new_height = int(EYE_WIDTH * glow_scale), int(EYE_HEIGHT * stretch)
        eye_rect = pygame.Rect(eye_x - (new_width - EYE_WIDTH)//2, EYE_Y_POS + bounce_offset, new_width, new_height)
        glow_color = (min(255, int(EYE_COLOR[0] * glow_scale)),
                      min(255, int(EYE_COLOR[1] * glow_scale)),
                      min(255, int(EYE_COLOR[2] * glow_scale)))
        pygame.draw.ellipse(screen, glow_color, eye_rect, 0)

        blush_x = eye_x + (EYE_WIDTH // 2) - 40
        blush_y = EYE_Y_POS + EYE_HEIGHT + 10
        draw_blush(screen, blush_x, blush_y, 80, 40)
    
    mouth_x, mouth_y = WIDTH // 2, EYE_Y_POS + EYE_HEIGHT + 70
    smile_offset = 7 * math.sin(time_elapsed * 4)
    arc_rect = pygame.Rect(mouth_x - 120//2, mouth_y + smile_offset, 120, 60)
    pygame.draw.arc(screen, PAMPER_COLOR, arc_rect, math.pi, math.pi * 2, 5)

# --- Main Game Loop ---
running = True
while running:
    # ### MODIFICATION START ###
    # --- Command Handling ---
    try:
        data, addr = server_socket.recvfrom(1024)  # buffer size is 1024 bytes
        command = data.decode('utf-8')
        print(f"Received command: {command}")
        is_nodding = (command == "nodding")
        # Map commands to states
        if command in ["happy", "angry", "sad", "pamper", "fear", "disgust", "surprise"]:
            current_emotion = command
            emotion_start_time = time.time()
            emotion_anim_progress = 0
        elif command in ["nodding", "neutral", "greet"]:
            current_emotion = "neutral" # These use the neutral draw function
    
    except BlockingIOError:
        # This is normal and expected, means no message has arrived
        pass
    except Exception as e:
        print(f"Socket error: {e}")
    # ### MODIFICATION END ###

    # --- Event Handling ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            running = False
        if event.type == pygame.KEYDOWN:
            key_map = {'h': "happy", 'a': "angry", 's': "sad", 'p': "pamper"}
            key_name = pygame.key.name(event.key)
            if key_name in key_map:
                emotion = key_map[key_name]
                current_emotion = emotion if current_emotion != emotion else "neutral"
                is_nodding = False # Keyboard overrides nodding
                emotion_start_time = time.time()
                emotion_anim_progress = 0

    # --- Time and State Updates ---
    now = time.time()
    dt = clock.tick(60) / 1000.0

    if current_emotion != "neutral":
        if emotion_anim_progress < 1.0:
            emotion_anim_progress += dt / EMOTION_ANIM_DURATION
        emotion_anim_progress = min(emotion_anim_progress, 1.0)
        # Don't revert to neutral automatically if the command was to be happy
        # Let the main app decide when to go back to neutral
        # Reset neutral state variables
        current_position, moving, blinking, lid_progress = 0, False, False, 0
    else:
        # Neutral state logic (also applies to nodding)
        emotion_anim_progress = 0
        if now > next_mode_time:
            mode, mode_start_time = random.choice([1, 2]), time.time()
            mode_duration = 3 + random.uniform(0.5, 1.5) if mode == 2 else 0
            next_mode_time = time.time() + random.uniform(10, 15)
        
        if mode == 2 and now - mode_start_time > mode_duration:
            mode = 1

        # Eye movement logic
        if not moving and now - last_change_time >= rest_duration:
            current_index = (current_index + 1) % len(movement_order)
            start_position, target_position = current_position, positions[movement_order[current_index]]
            moving, move_start_time = True, now
        if moving:
            elapsed_time = now - move_start_time
            if elapsed_time >= move_duration:
                current_position, moving = target_position, False
                last_change_time, rest_duration = now, random.uniform(3, 6)
            else:
                t = elapsed_time / move_duration
                smoothed_t = t * t * (3 - 2 * t)
                current_position = start_position + (target_position - start_position) * smoothed_t
        
        # Blinking logic
        if not blinking and now - blink_start_time > blink_interval:
            blinking, lid_progress, lid_direction, blink_start_time = True, 0, 1, now
        if blinking:
            lid_progress += lid_direction * (dt / BLINK_DURATION)
            if lid_progress >= 1:
                lid_progress, lid_direction = 1, -1
            elif lid_progress <= 0:
                lid_progress, blinking = 0, False

    # --- Drawing ---
    screen.fill(BG_COLOR)

    if current_emotion == "happy":
        draw_happy_emotion(now, emotion_start_time, emotion_anim_progress)
    elif current_emotion == "angry":
        draw_angry_emotion(emotion_anim_progress)
    elif current_emotion == "sad":
        draw_sad_emotion(now, emotion_start_time, emotion_anim_progress)
    elif current_emotion == "pamper":
        draw_pamper_emotion(now, emotion_start_time, emotion_anim_progress)
    else: # This handles both "neutral" and "nodding"
        draw_neutral_emotion()

    pygame.display.flip()

# --- Quit ---
server_socket.close() # MODIFICATION: Close the socket
pygame.quit()
