import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import threading
import time


import numpy as np
import random
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque

xml_path = 'maze.xml'  # xml file (assumes this is in the same folder as this file)
simend = 1000  # simulation time
print_camera_config = 0  # set to 1 to print camera config

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)  # MuJoCo data
cam = mj.MjvCamera()  # Abstract camera
opt = mj.MjvOption()  # Visualization options

def init_controller(model, data):
    # Initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    # Put the controller here. This function is called inside the simulation.
    pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # Update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # Update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # Compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # No buttons down: nothing to do
    if not button_left and not button_middle and not button_right:
        return

    # Get current window size
    width, height = glfw.get_window_size(window)

    # Get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT

    # Determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx / height, dy / height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 * yoffset, scene, cam)

def simulation_loop():
    global model, data, cam, scene, context, xml_path

    # Get the full path
    dirname = os.path.dirname(__file__)
    abspath = os.path.join(dirname, xml_path)
    xml_path = abspath



    # Init GLFW, create window, make OpenGL context current, request v-sync
    glfw.init()
    window = glfw.create_window(1200, 900, "Maze Simulation", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    # Initialize visualization data structures
    mj.mjv_defaultCamera(cam)
    mj.mjv_defaultOption(opt)
    scene = mj.MjvScene(model, maxgeom=10000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

    # Install GLFW mouse and keyboard callbacks
    glfw.set_key_callback(window, keyboard)
    glfw.set_cursor_pos_callback(window, mouse_move)
    glfw.set_mouse_button_callback(window, mouse_button)
    glfw.set_scroll_callback(window, scroll)

    # Example on how to set camera configuration
    # cam.azimuth = 90
    # cam.elevation = -45
    # cam.distance = 2
    # cam.lookat = np.array([0.0, 0.0, 0])

    # Initialize the controller
    init_controller(model, data)

    # Set the controller
    mj.set_mjcb_control(controller)

    while not glfw.window_should_close(window):
        time_prev = data.time

        while data.time - time_prev < 1.0 / 60.0:
            mj.mj_step(model, data)

        if data.time >= simend:
            break

        # Get framebuffer viewport
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

        # Print camera configuration (help to initialize the view)
        if print_camera_config == 1:
            print('cam.azimuth =', cam.azimuth, ';', 'cam.elevation =', cam.elevation, ';', 'cam.distance = ', cam.distance)
            print('cam.lookat = np.array([', cam.lookat[0], ',', cam.lookat[1], ',', cam.lookat[2], '])')

        # Update scene and render
        mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)

        # Swap OpenGL buffers (blocking call due to v-sync)
        glfw.swap_buffers(window)

        # Process pending GUI events, call GLFW callbacks
        glfw.poll_events()

    glfw.terminate()

# Run the simulation in a separate thread
def start_simulation():
    thread = threading.Thread(target=simulation_loop)
    thread.start()


start_simulation() #Using separate thread for simulation evvironment


destination = np.array([27, 27, 0.5])


def _get_obs():
  return data.qpos[:3].copy()

def move_forward():
    data.ctrl[1] = 0
    data.ctrl[0] = 1
    time.sleep(1)
    data.ctrl[0] = 0

def move_backward():
    data.ctrl[1] = 0
    data.ctrl[0] = -1
    time.sleep(1)
    data.ctrl[0] = 0

def move_left():
    data.ctrl[1] = 10
    time.sleep(1)
    data.ctrl[1] = 0

def move_right():
    data.ctrl[1] = -10
    time.sleep(1)
    data.ctrl[1] = 0

def _calculate_distance_to_target():
    """Calculate Euclidean distance from the current position to the target."""
    current_position = _get_obs()
    return np.linalg.norm(destination - current_position)

def reset():
    mj.mj_resetData(model, data)
    mj.mj_forward(model, data)
    
    # Reset previous distance
    prev_distance = _calculate_distance_to_target()
    
    # Reset the start time
    start_time = time.time()
    
    return _get_obs(), start_time, prev_distance

######################################################################################################################



#Neural network model
class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, output_dim)
    
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

# Hyperparameters
alpha = 0.001  # Learning rate
gamma = 0.9    # Discount factor
epsilon = 1.0  # Exploration rate
epsilon_decay = 0.995
min_epsilon = 0.01
batch_size = 64
memory_size = 10000
target_update = 10  # Updates the target network every 10 episodes
num_episodes = 1000  # Number of episodes to train
max_steps = 100  # Maximum steps per episode

# Initializing DQN and target DQN
input_dim = 3  #State is 3-dimensional (x, y, z)
output_dim = 4  # Number of actions
policy_net = DQN(input_dim, output_dim)
target_net = DQN(input_dim, output_dim)
target_net.load_state_dict(policy_net.state_dict())
target_net.eval()

# Optimizer and loss function
optimizer = optim.Adam(policy_net.parameters(), lr=alpha)
loss_fn = nn.MSELoss()

# Experience replay memory
memory = deque(maxlen=memory_size)

# Function to add experience to memory
def remember(state, action, reward, next_state, done):
    memory.append((state, action, reward, next_state, done))

# Function to sample a batch from memory
def replay():
    if len(memory) < batch_size:
        return
    batch = random.sample(memory, batch_size)
    state_batch, action_batch, reward_batch, next_state_batch, done_batch = zip(*batch)
    
    state_batch = torch.tensor(state_batch, dtype=torch.float32)
    action_batch = torch.tensor(action_batch, dtype=torch.long)
    reward_batch = torch.tensor(reward_batch, dtype=torch.float32)
    next_state_batch = torch.tensor(next_state_batch, dtype=torch.float32)
    done_batch = torch.tensor(done_batch, dtype=torch.float32)
    
    q_values = policy_net(state_batch).gather(1, action_batch.unsqueeze(1)).squeeze(1)
    next_q_values = target_net(next_state_batch).max(1)[0]
    target_q_values = reward_batch + gamma * next_q_values * (1 - done_batch)
    
    loss = loss_fn(q_values, target_q_values)
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

# Helper function to preprocess the state
def preprocess_state(obs):
    return obs

for episode in range(num_episodes):
    # Reset environment
    obs, start_time, prev_distance = reset()
    state = preprocess_state(obs)

    print(f"\nStarting Episode {episode}")
    print(f"Initial State: {state}, Initial Distance to Target: {prev_distance}")

    for step in range(max_steps):
        state_tensor = torch.tensor([state], dtype=torch.float32)
        
        # Choose action
        if random.uniform(0, 1) < epsilon:
            action = np.random.randint(0, output_dim)  # Explore
            print(f"Action chosen randomly: {action}")
        else:
            with torch.no_grad():
                action = policy_net(state_tensor).argmax().item()  # Exploit
            print(f"Action chosen by exploiting: {action}")

        # Take action
        if action == 0:
            move_forward()
            print("Moved forward")
        elif action == 1:
            move_backward()
            print("Moved backward")
        elif action == 2:
            move_left()
            print("Moved left")
        elif action == 3:
            move_right()
            print("Moved right")

        # Observe new state
        new_obs = _get_obs()
        new_state = preprocess_state(new_obs)
        current_distance = _calculate_distance_to_target()

        # Calculate reward
        reward = prev_distance - current_distance
        print(f"Reward: {reward}, New Distance to Target: {current_distance}")

        # Check if the agent has reached the destination
        done = current_distance < 0.1
        if done:
            reward += 10  # Give a bonus reward for reaching the target
            print(f"Destination reached in episode {episode}, step {step}")

        # Remember experience
        remember(state, action, reward, new_state, done)

        # Update state and distance
        state = new_state
        prev_distance = current_distance

        # Replay and train the network
        replay()

        if done:
            break

    # Updates target network periodically
    if episode % target_update == 0:
        target_net.load_state_dict(policy_net.state_dict())
        print(f"Target network updated at episode {episode}")

    # Decay epsilon
    epsilon = max(min_epsilon, epsilon * epsilon_decay)
    print(f"End of Episode {episode}, Epsilon: {epsilon}")
































