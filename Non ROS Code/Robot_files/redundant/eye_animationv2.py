# -*- coding: utf-8 -*-
"""
Multi-Emotion Face Simulator (v2)
---------------------------------
Receives commands via UDP to change emotional states.
Includes new Fear and Surprise emotions.
"""
import pygame
import time
import random
import numpy as np
import math
import socket

# --- Pygame Initialization ---
pygame.init()
info = pygame.display.Info()
WIDTH, HEIGHT = info.current_w, info.current_h
screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN)
pygame.display.set_caption("Multi-Emotion Face Simulator v2")
clock = pygame.time.Clock()

# --- IPC Configuration ---
UDP_HOST = '127.0.0.1'
UDP_PORT = 12345
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((UDP_HOST, UDP_PORT))
server_socket.setblocking(False)
print(f"✅ Eye Animation v2 is listening for commands on port {UDP_PORT}")

# --- Configuration ---

# Colors
EYE_COLOR = (0, 136, 255)
HAPPY_COLOR = (0, 136, 255)
GREET_COLOR = (0, 255, 0)
LID_COLOR = (0, 0, 0)
BG_COLOR = (0, 0, 0)
SMILE_COLOR = HAPPY_COLOR
ANGRY_COLOR = (255, 0, 0)
PAMPER_COLOR = (255, 100, 100)
TEAR_COLOR = (173, 216, 230)
SURPRISE_COLOR = (255, 255, 0) # Bright Yellow
FEAR_COLOR = (180, 150, 255)   # Light Purple

# Eye Parameters
EYE_WIDTH, EYE_HEIGHT = 180, 180
EYE_CORNER_RADIUS = 45
EYE_Y_POS = HEIGHT // 2 - EYE_HEIGHT // 2 - 100
EYE_SPACING = 300

# Animation Timings
EMOTION_ANIM_DURATION = 0.3
BLINK_DURATION = 0.1

# Emotion-Specific Animation Parameters
HAPPY_CONFIG = {"bounce_speed": 5, "bounce_amp": 8, "scale_speed": 4, "scale_amp": 0.08, "squint": 0.4}
SAD_CONFIG = {"gaze_x_amp": -20, "gaze_y_amp": 30, "droop_factor": 1.5, "mouth_float_speed": 8, "mouth_float_amp": 3, "tear_start_progress": 0.3, "tear_size": 40}
PAMPER_CONFIG = {"glow_speed": 4, "glow_amp": 0.1, "bounce_speed": 3, "bounce_amp": 6, "stretch_speed": 6, "stretch_amp": 0.05}
FEAR_CONFIG = {"wobble_speed": 15, "wobble_amp": 5, "pupil_scale": 0.8} # MODIFIED: smaller pupil
SURPRISE_CONFIG = {"scale_amp": 1.3, "mouth_size": 80} # MODIFIED: larger mouth

# --- State Management Variables ---
current_emotion = "neutral"
emotion_anim_progress = 0
emotion_start_time = 0
is_nodding = False

# --- RESTORED: Neutral State Dynamic Behavior ---
mode = 1
mode_start_time = time.time()
next_mode_time = time.time() + random.uniform(8, 15)

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
def generate_smile_data(progress, y_offset=0):
    if progress <= 0: return None
    num_points, max_thickness, min_ratio = 50, 20, 0.25
    x_vals = np.linspace(-1, 1, num_points)
    y_center = -(x_vals**4 * 0.8 + x_vals**2 * 0.2 - 0.7) * 50 * progress
    thickness_profile = (1 - x_vals**2) * (1 - min_ratio) + min_ratio
    thickness = thickness_profile * max_thickness * progress
    y_upper, y_lower = y_center - thickness/2, y_center + thickness/2
    smile_width, smile_y_pos = 320, HEIGHT // 2 + 100 + y_offset
    upper_points = [(WIDTH//2 + x*smile_width/2, smile_y_pos + y) for x, y in zip(x_vals, y_upper)]
    lower_points = [(WIDTH//2 + x*smile_width/2, smile_y_pos + y) for x, y in zip(x_vals, y_lower)]
    polygon_points = upper_points + lower_points[::-1]
    cap_radius = thickness[0] / 2
    left_cap_pos = (upper_points[0][0], upper_points[0][1] + cap_radius)
    right_cap_pos = (upper_points[-1][0], upper_points[-1][1] + cap_radius)
    return polygon_points, [(left_cap_pos, cap_radius), (right_cap_pos, cap_radius)]

def generate_angry_mouth_points(progress, offset=(0, 0)):
    if progress <= 0: return None
    points = [(-1, 0.5), (-0.5, -0.5), (0, 0.5), (0.5, -0.5), (1, 0.5)]
    mouth_width, mouth_height, mouth_y_pos = 150, 25 * progress, HEIGHT // 2 + 80
    return [(WIDTH/2 + p[0]*mouth_width/2 + offset[0], mouth_y_pos + p[1]*mouth_height + offset[1]) for p in points]

# --- NEW: Function for Fear Mouth ---
def generate_fear_mouth_points(progress, current_time):
    if progress <= 0: return None
    num_points = 50
    mouth_width = 140
    amplitude = 8 * progress
    frequency = 15
    # A sine wave for the y-coordinates to make it wavy
    x_vals = np.linspace(-1, 1, num_points)
    y_vals = np.sin(x_vals * frequency + current_time * 10) * amplitude
    
    mouth_y_pos = HEIGHT // 2 + 100
    points = [(WIDTH/2 + x * mouth_width/2, mouth_y_pos + y) for x, y in zip(x_vals, y_vals)]
    return points

# --- Component Drawing Functions ---
def draw_rounded_eye(surface, x, y, width, height, radius, color=EYE_COLOR, offset=(0, 0)):
    pygame.draw.rect(surface, color, (x + offset[0], y + offset[1], width, height), border_radius=radius)

def draw_circle_eye(surface, x, y, diameter, color=EYE_COLOR, offset=(0, 0)):
    pygame.draw.ellipse(surface, color, (x + offset[0], y + offset[1], diameter, diameter))

def draw_lids(surface, x, y, width, height, blink_progress=0, squint_progress=0):
    if blink_progress > 0:
        lid_height = int(height * blink_progress)
        pygame.draw.rect(surface, LID_COLOR, (x, y, width, lid_height), border_radius=EYE_CORNER_RADIUS)
        pygame.draw.rect(surface, LID_COLOR, (x, y + height - lid_height, width, lid_height), border_radius=EYE_CORNER_RADIUS)
    if squint_progress > 0:
        squint_height = int(height * squint_progress)
        pygame.draw.rect(surface, LID_COLOR, (x, y + height - squint_height, width, squint_height), border_radius=EYE_CORNER_RADIUS)

def draw_frown(surface, x, y, width, height, progress, color, offset=(0, 0)):
    if progress <= 0: return
    frown_rect = pygame.Rect(x - width//2 + offset[0], y + offset[1], width, height)
    pygame.draw.arc(surface, color, frown_rect, 0, math.pi, int(10 * progress))

def draw_tear(surface, x, y, size, progress, offset=(0, 0)):
    cfg = SAD_CONFIG
    if progress < cfg["tear_start_progress"]: return
    current_size = size * ((progress - cfg["tear_start_progress"]) / (1 - cfg["tear_start_progress"]))
    if current_size <= 0: return
    glisten_factor = abs(math.sin(time.time() * 5)) * 0.5 + 0.75
    glisten_color = tuple(min(255, int(c * glisten_factor)) for c in TEAR_COLOR)
    body_height, radius = current_size, current_size / 2
    triangle_points = [(x + offset[0], y + offset[1]), (x - radius + offset[0], y + body_height * 0.7 + offset[1]), (x + radius + offset[0], y + body_height * 0.7 + offset[1])]
    circle_center = (x + offset[0], y + body_height * 0.7 + offset[1])
    pygame.draw.polygon(surface, glisten_color, triangle_points)
    pygame.draw.circle(surface, glisten_color, circle_center, radius)

def draw_blush(surface, x, y, radius_x, radius_y):
    pygame.draw.ellipse(surface, PAMPER_COLOR, (x, y, radius_x, radius_y))

# --- Full Emotion Drawing Functions ---

def draw_neutral_emotion():
    y_offset = 20 * math.sin(time.time() * 10) if is_nodding else 0
    left_style, right_style = 'normal', 'normal'
    current_move = movement_order[current_index]
    if mode == 2:
        left_style, right_style = 'enlarged', 'circle'
    elif current_move == 'left':
        left_style, right_style = 'circle', 'enlarged'
    elif current_move == 'right':
        left_style, right_style = 'enlarged', 'circle'

    for i, style in enumerate([left_style, right_style]):
        dx = -EYE_SPACING if i == 0 else EYE_SPACING
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

def draw_happy_emotion(current_time, start_time, progress):
    cfg = HAPPY_CONFIG
    time_elapsed = current_time - start_time
    bounce = cfg["bounce_amp"] * math.sin(time_elapsed * cfg["bounce_speed"]) * progress
    scale = 1 + cfg["scale_amp"] * math.sin(time_elapsed * cfg["scale_speed"]) * progress
    new_w, new_h = int(EYE_WIDTH * scale), int(EYE_HEIGHT * scale)
    for dx in [-EYE_SPACING, EYE_SPACING]:
        base_x = WIDTH // 2 + dx - EYE_WIDTH // 2
        new_x = base_x - (new_w - EYE_WIDTH) // 2
        new_y = EYE_Y_POS - (new_h - EYE_HEIGHT) // 2
        draw_rounded_eye(screen, new_x, new_y + bounce, new_w, new_h, EYE_CORNER_RADIUS, color=HAPPY_COLOR)
        draw_lids(screen, new_x, new_y + bounce, new_w, new_h, squint_progress=progress * cfg["squint"])
    smile_data = generate_smile_data(progress, y_offset=bounce)
    if smile_data:
        poly, caps = smile_data
        pygame.draw.polygon(screen, SMILE_COLOR, poly)
        for pos, rad in caps: pygame.draw.circle(screen, SMILE_COLOR, pos, rad)

def draw_greet_emotion(progress):
    for dx in [-EYE_SPACING, EYE_SPACING]:
        base_x = WIDTH // 2 + dx - EYE_WIDTH // 2
        draw_rounded_eye(screen, base_x, EYE_Y_POS, EYE_WIDTH, EYE_HEIGHT, EYE_CORNER_RADIUS, color=GREET_COLOR)
        draw_lids(screen, base_x, EYE_Y_POS, EYE_WIDTH, EYE_HEIGHT, squint_progress=progress * 0.4)
    smile_data = generate_smile_data(progress)
    if smile_data:
        poly, caps = smile_data
        pygame.draw.polygon(screen, GREET_COLOR, poly)
        for pos, rad in caps: pygame.draw.circle(screen, GREET_COLOR, pos, rad)

def draw_angry_emotion(progress):
    shake = (random.uniform(-3, 3), random.uniform(-3, 3)) if progress > 0.5 else (0, 0)
    for i, dx in enumerate([-EYE_SPACING, EYE_SPACING]):
        eye_x = WIDTH // 2 + dx - EYE_WIDTH // 2
        slant = 60 * progress
        p1 = (eye_x + shake[0], EYE_Y_POS + shake[1] + (slant if i == 1 else 0))
        p2 = (eye_x + shake[0] + EYE_WIDTH, EYE_Y_POS + shake[1] + (slant if i == 0 else 0))
        p3 = (eye_x + shake[0] + EYE_WIDTH, EYE_Y_POS + shake[1] + EYE_HEIGHT)
        p4 = (eye_x + shake[0], EYE_Y_POS + shake[1] + EYE_HEIGHT)
        pygame.draw.polygon(screen, ANGRY_COLOR, [p1, p2, p3, p4])
    mouth = generate_angry_mouth_points(progress, offset=shake)
    if mouth: pygame.draw.lines(screen, ANGRY_COLOR, False, mouth, 12)

def draw_sad_emotion(current_time, start_time, progress):
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
    cfg = PAMPER_CONFIG
    time_elapsed = current_time - start_time
    for i, dx in enumerate([-EYE_SPACING, EYE_SPACING]):
        eye_x = WIDTH // 2 + dx - EYE_WIDTH // 2
        glow_scale = 1 + cfg["glow_amp"] * math.sin(time_elapsed * cfg["glow_speed"])
        bounce_offset = cfg["bounce_amp"] * math.sin(time_elapsed * cfg["bounce_speed"])
        stretch = 1 + cfg["stretch_amp"] * math.sin(time_elapsed * cfg["stretch_speed"])
        new_width, new_height = int(EYE_WIDTH * glow_scale), int(EYE_HEIGHT * stretch)
        eye_rect = pygame.Rect(eye_x - (new_width - EYE_WIDTH) // 2, EYE_Y_POS + bounce_offset, new_width, new_height)
        glow_color = tuple(min(255, int(c * glow_scale)) for c in EYE_COLOR)
        pygame.draw.ellipse(screen, glow_color, eye_rect, 0)
        blush_x = eye_x + (EYE_WIDTH // 2) - 40
        blush_y = EYE_Y_POS + EYE_HEIGHT + 10
        draw_blush(screen, blush_x, blush_y, 80, 40)
    mouth_x, mouth_y = WIDTH // 2, EYE_Y_POS + EYE_HEIGHT + 70
    smile_offset = 7 * math.sin(time_elapsed * 4)
    arc_rect = pygame.Rect(mouth_x - 120//2, mouth_y + smile_offset, 120, 60)
    pygame.draw.arc(screen, PAMPER_COLOR, arc_rect, math.pi, math.pi * 2, 5)

def draw_surprise_emotion(progress):
    cfg = SURPRISE_CONFIG
    scale = 1 + (cfg["scale_amp"] - 1) * progress
    new_size = int(EYE_WIDTH * scale)
    y_offset = (new_size - EYE_HEIGHT) // 2
    for dx in [-EYE_SPACING, EYE_SPACING]:
        eye_x = WIDTH // 2 + dx - new_size // 2
        eye_y = EYE_Y_POS - y_offset
        pygame.draw.ellipse(screen, SURPRISE_COLOR, (eye_x, eye_y, new_size, new_size))
    mouth_size = cfg["mouth_size"] * progress
    if mouth_size > 0:
        mouth_x = WIDTH // 2
        mouth_y = HEIGHT // 2 + 120
        pygame.draw.ellipse(screen, SURPRISE_COLOR, (mouth_x - mouth_size / 2, mouth_y - mouth_size / 2, mouth_size, mouth_size), width=10)

def draw_fear_emotion(current_time, progress):
    cfg = FEAR_CONFIG
    for dx in [-EYE_SPACING, EYE_SPACING]:
        wobble_x = cfg["wobble_amp"] * math.sin(current_time * cfg["wobble_speed"] + dx) * progress
        wobble_y = cfg["wobble_amp"] * math.cos(current_time * cfg["wobble_speed"] + dx) * progress
        base_x = WIDTH // 2 + dx - EYE_WIDTH // 2
        base_y = EYE_Y_POS
        draw_rounded_eye(screen, base_x, base_y, EYE_WIDTH, EYE_HEIGHT, EYE_CORNER_RADIUS, color=FEAR_COLOR, offset=(wobble_x, wobble_y))
        
        # MODIFIED: Smaller pupil
        pupil_size = (EYE_WIDTH / 4) * progress * cfg["pupil_scale"] 
        pupil_x = base_x + (EYE_WIDTH / 2) + wobble_x
        pupil_y = base_y + (EYE_HEIGHT / 2) + wobble_y
        pygame.draw.circle(screen, BG_COLOR, (pupil_x, pupil_y), pupil_size)

    # MODIFIED: Add the new wavy mouth
    mouth_points = generate_fear_mouth_points(progress, current_time)
    if mouth_points:
        pygame.draw.lines(screen, FEAR_COLOR, False, mouth_points, 8)


# --- Main Game Loop ---
running = True
while running:
    # Command Handling
    try:
        data, addr = server_socket.recvfrom(1024)
        command = data.decode('utf-8').lower()
        print(f"Received command: {command}")
        is_nodding = (command == "nodding")
        if command in ["happy", "angry", "sad", "pamper", "fear", "surprise", "greet", "neutral"]:
            current_emotion = command
            emotion_start_time = time.time()
            emotion_anim_progress = 0
            if command != "neutral":
                is_nodding = False
    except BlockingIOError:
        pass
    except Exception as e:
        print(f"Socket error: {e}")

    # --- RESTORED: Event Handling with Keyboard Shortcuts ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            running = False
        if event.type == pygame.KEYDOWN:
            # --- MODIFIED: Added full key map ---
            key_map = {
                'h': "happy", 'a': "angry", 's': "sad", 'p': "pamper", 
                'g': "greet", 'f': "fear", 'w': "surprise", 'n': "neutral"
            }
            key_name = pygame.key.name(event.key)
            if key_name in key_map:
                emotion = key_map[key_name]
                # Logic to toggle emotion on/off or switch
                if key_name == 'n':
                    current_emotion = "neutral"
                else:
                    current_emotion = emotion if current_emotion != emotion else "neutral"
                
                is_nodding = False
                emotion_start_time = time.time()
                emotion_anim_progress = 0

    # --- RESTORED: Time and State Updates for Neutral Mode ---
    now = time.time()
    dt = clock.tick(60) / 1000.0
    if current_emotion != "neutral":
        if emotion_anim_progress < 1.0:
            emotion_anim_progress += dt / EMOTION_ANIM_DURATION
        emotion_anim_progress = min(emotion_anim_progress, 1.0)
        current_position, moving, blinking, lid_progress = 0, False, False, 0 # Reset neutral state
    else:
        emotion_anim_progress = 0
        if now > next_mode_time:
            mode, mode_start_time = random.choice([1, 2]), time.time()
            mode_duration = 3 + random.uniform(0.5, 1.5) if mode == 2 else 0
            next_mode_time = time.time() + random.uniform(10, 15)

        if mode == 2 and now - mode_start_time > mode_duration:
            mode = 1
        
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
        if not blinking and now - blink_start_time > blink_interval:
            blinking, lid_progress, lid_direction, blink_start_time = True, 0, 1, now
        if blinking:
            lid_progress += lid_direction * (dt / BLINK_DURATION)
            if lid_progress >= 1: lid_progress, lid_direction = 1, -1
            elif lid_progress <= 0: lid_progress, blinking = 0, False

    # --- Drawing ---
    screen.fill(BG_COLOR)
    if current_emotion == "happy": draw_happy_emotion(now, emotion_start_time, emotion_anim_progress)
    elif current_emotion == "greet": draw_greet_emotion(emotion_anim_progress)
    elif current_emotion == "angry": draw_angry_emotion(emotion_anim_progress)
    elif current_emotion == "sad": draw_sad_emotion(now, emotion_start_time, emotion_anim_progress)
    elif current_emotion == "pamper": draw_pamper_emotion(now, emotion_start_time, emotion_anim_progress)
    elif current_emotion == "surprise": draw_surprise_emotion(emotion_anim_progress)
    elif current_emotion == "fear": draw_fear_emotion(now, emotion_anim_progress)
    else: draw_neutral_emotion()

    pygame.display.flip()

# --- Quit ---
server_socket.close()
pygame.quit()