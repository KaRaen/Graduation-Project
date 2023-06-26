from __future__ import print_function
import os, sys, time, datetime, json, random
import numpy as np
import pandas as pd
from keras.models import Sequential
from keras.layers.core import Dense, Activation
from keras.optimizers import SGD, Adam, RMSprop
from keras.layers import PReLU
import matplotlib.pyplot as plt
from keras.models import model_from_json
import serial
import pandas as pd
import warnings

ser = serial.Serial('COM15', 9600)  # replace with your HC-05's serial port
ser.flushInput()

class Qmaze(object):

    def __init__(self, maze, rat=(0, 2)):
        self._maze = np.array(maze)
        nrows, ncols = self._maze.shape
        self.target = goal  # target cell where the "cheese" is

        self.free_cells = [(r, c) for r in range(nrows) for c in range(ncols) if self._maze[r, c] == 1.0]
        self.free_cells.remove(self.target)
        if self._maze[self.target] == 0.0:
            raise Exception("Invalid maze: target cell cannot be blocked!")
        if not rat in self.free_cells:
            raise Exception("Invalid Rat Location: must sit on a free cell")
        self.reset(rat)

    def reset(self, rat):
        self.rat = rat
        self.maze = np.copy(self._maze)
        nrows, ncols = self.maze.shape
        row, col = rat
        self.maze[row, col] = rat_mark
        self.state = (row, col, 'start')
        self.min_reward = -0.5 * self.maze.size
        self.total_reward = 0
        self.visited = set()

    def update_state(self, action):
        nrows, ncols = self.maze.shape
        nrow, ncol, nmode = rat_row, rat_col, mode = self.state

        if self.maze[rat_row, rat_col] > 0.0:
            self.visited.add((rat_row, rat_col))  # mark visited cell

        valid_actions = self.valid_actions()

        if not valid_actions:
            nmode = 'blocked'
        elif action in valid_actions:
            nmode = 'valid'
            if action == LEFT:
                ncol -= 1
            elif action == UP:
                nrow -= 1
            if action == RIGHT:
                ncol += 1
            elif action == DOWN:
                nrow += 1
        else:  # invalid action, no change in rat position
            mode = 'invalid'

        # new state
        self.state = (nrow, ncol, nmode)

    def get_reward(self):
        rat_row, rat_col, mode = self.state
        nrows, ncols = self.maze.shape
        if rat_row == goal[0] and rat_col == goal[1]:
            return 1.0
        if mode == 'blocked':
            return self.min_reward - 1
        if (rat_row, rat_col) in self.visited:
            return -0.25
        if mode == 'invalid':
            return -0.75
        if mode == 'valid':
            return -0.04

    def act(self, action):
        self.update_state(action)
        reward = self.get_reward()
        self.total_reward += reward
        status = self.game_status()
        envstate = self.observe()
        return envstate, reward, status

    def observe(self):
        canvas = self.draw_env()
        envstate = canvas.reshape((1, -1))
        return envstate

    def draw_env(self):
        canvas = np.copy(self.maze)
        nrows, ncols = self.maze.shape
        # clear all visual marks
        for r in range(nrows):
            for c in range(ncols):
                if canvas[r, c] > 0.0:
                    canvas[r, c] = 1.0
        # draw the rat
        row, col, valid = self.state
        canvas[row, col] = rat_mark
        return canvas

    def game_status(self):
        if self.total_reward < self.min_reward:
            return 'lose'
        rat_row, rat_col, mode = self.state
        nrows, ncols = self.maze.shape
        if rat_row == goal[0] and rat_col == goal[1]:
            return 'win'

        return 'not_over'

    def valid_actions(self, cell=None):
        if cell is None:
            row, col, mode = self.state
        else:
            row, col = cell
        actions = [0, 1, 2, 3]
        nrows, ncols = self.maze.shape
        if row == 0:
            actions.remove(1)
        elif row == nrows - 1:
            actions.remove(3)

        if col == 0:
            actions.remove(0)
        elif col == ncols - 1:
            actions.remove(2)

        if row > 0 and self.maze[row - 1, col] == 0.0:
            actions.remove(1)
        if row < nrows - 1 and self.maze[row + 1, col] == 0.0:
            actions.remove(3)

        if col > 0 and self.maze[row, col - 1] == 0.0:
            actions.remove(0)
        if col < ncols - 1 and self.maze[row, col + 1] == 0.0:
            actions.remove(2)
        return actions

def get_robot_route(self):
    robot_route = []
    direction = 'DOWN'
    optimal_route = self
    if optimal_route != None:
        for index, i in enumerate(optimal_route):
            if direction == 'DOWN':
                if i == "MOVE_LEFT":  # LEFT
                    robot_route.append("GO RIGHT")
                    direction = 'LEFT'
                if i == "MOVE_RIGHT":  # RIGHT
                    robot_route.append("GO LEFT")
                    direction = 'RIGHT'
                if i == "MOVE_DOWN":  # DOWN
                    robot_route.append("GO FORWARD")
                    direction = 'DOWN'
                if i == "MOVE_UP":  # UP
                    robot_route.append("GO BACKWARD")
                    direction = 'UP'

            elif direction == 'LEFT':
                if i == "MOVE_LEFT":  # LEFT
                    robot_route.append("GO FORWARD")
                    direction = 'LEFT'
                if i == "MOVE_RIGHT":  # RIGHT
                    robot_route.append("GO BACKWARD")
                    direction = 'RIGHT'
                if i == "MOVE_DOWN":  # DOWN
                    robot_route.append("GO LEFT")
                    direction = 'DOWN'
                if i == "MOVE_UP":  # UP
                    robot_route.append("GO RIGHT")
                    direction = 'UP'

            elif direction == 'RIGHT':
                if i == "MOVE_LEFT":  # LEFT
                    robot_route.append("GO BACKWARD")
                    direction = 'LEFT'
                if i == "MOVE_RIGHT":  # RIGHT
                    robot_route.append("GO FORWARD")
                    direction = 'RIGHT'
                if i == "MOVE_DOWN":  # DOWN
                    robot_route.append("GO RIGHT")
                    direction = 'DOWN'
                if i == "MOVE_UP":  # UP
                    robot_route.append("GO LEFT")
                    direction = 'UP'

            elif direction == 'UP':
                if i == "MOVE_LEFT":  # LEFT
                    robot_route.append("GO LEFT")
                    direction = 'LEFT'
                if i == "MOVE_RIGHT":  # RIGHT
                    robot_route.append("GO RIGHT")
                    direction = 'RIGHT'
                if i == "MOVE_DOWN":  # DOWN
                    robot_route.append("GO BACKWARD")
                    direction = 'DOWN'
                if i == "MOVE_UP":  # UP
                    robot_route.append("GO FORWARD")
                    direction = 'UP'
        return (robot_route)
    else:
        print("Can not find route state invalid")
        return

def convert_action(action):
    if action == 0:
        return "MOVE_LEFT"
    elif action == 1:
        return "MOVE_UP"
    elif action == 2:
        return "MOVE_RIGHT"
    elif action == 3:
        return "MOVE_DOWN"
    else:
        return "STOP"

def run_model():

    global reward, envstate, goal_string,qmaze
    with open("model" + goal_string + ".json", "r") as jfile:
        model = model_from_json(json.load(jfile))
    model.load_weights("model" + goal_string + ".h5")
    model.compile("sgd", "mse")

    while reward < 1.0:
        prev_envstate = envstate
        q = model.predict(prev_envstate)
        action = np.argmax(q[0])
        envstate, reward, game_over = qmaze.act(action)
        steps.append(convert_action(action))

    model.reset_states()

def tgian():
    now = datetime.datetime.now()
    today = now.date()
    currentTime = now.strftime("%H:%M:%S")
    currentTime = currentTime.replace(':', '_')
    second = now.strftime("%S")
    today = str(today)
    second = int(second)
    return today, currentTime, second


def create_file(today):
    directory = today + "\\"
    parent_dir = "D:\\GRADUATION\\Determination\\Database"
    path = os.path.join(parent_dir, directory)
    k = path + today
    if not os.path.exists(path):
        os.mkdir(path)
    else:
        print("Da cap nhat")

    return k

def convert_to_Arduino():

    global dir_path,steps_robot
    for z in range (len(steps_robot)):
        if steps_robot[z]=='GO FORWARD':
            dir_path.append(1)
        elif steps_robot[z] == 'GO RIGHT':
            dir_path.append(2)
        elif steps_robot[z] == 'GO LEFT':
            dir_path.append(3)
    return dir_path

def Control_Robot():

    global key_last,reward,steps,num_step,room,arduino_to_excel,name,dir,dir_old, current_step, prev_step,first,envstate,dir_path,goal,goal_string,maze,qmaze,i,j,steps_robot,rat_cell
    train=0
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            response_data = json.loads(line)
            value_1 = response_data['theta']
            value_2 = response_data['spin']
            value_3 = response_data['mode']
            value_4 = response_data['pass']
            value_5 = response_data['prev_step']
            value_6= response_data['dis_obstacle']
            value_7=response_data['room']
            value_8=response_data['x']
            value_9=response_data['y']
            warnings.filterwarnings("ignore")
            data_from_arduino = {'mode': [value_3], 'prev_step':[0],'x': [value_8], 'y':[value_9],'theta':[value_1], 'obstacle':[value_6]}
            store_data_to_dataframe = pd.DataFrame(data=data_from_arduino, index=[currentTime])
            arduino_to_excel = arduino_to_excel.append(store_data_to_dataframe)
            arduino_to_excel.to_excel(excel_writer=name + '.xlsx')
            room = value_7

            if first==0:
                if room == 1:
                    goal = (3, 3)
                if room == 2:
                    goal = (7, 2)
                if room == 3:
                    goal = (5, 1)
                goal_string = str(goal)
                goal_string = goal_string.replace(" ", "")
                if (room!=0):
                    if train==0:
                        print(goal_string)
                        print (rat_cell)
                        qmaze = Qmaze(maze)
                        qmaze.reset(rat_cell)
                        envstate = qmaze.observe()
                        run_model()
                        print(steps)
                        steps_robot = get_robot_route(steps)
                        print("Đường đi mô phỏng")
                        print(steps_robot)
                        convert_to_Arduino()
                        print("Đường đi thực tế")
                        print(dir_path)
                        print(len(dir_path))
                        train=1
                        first = 1
                        key_last=0
                    if train==1:
                        if i==0:
                            print("Đã tìm được đường đi tối ưu")
                            i=1
                else:
                    if j == 0:
                        print("Chưa có phòng được chọn")
                        j = 1

            elif first==1:

                print(f"prev_step:{value_5},obstacle:{value_6}")
                prev_step = value_5

                if prev_step < len(dir_path):
                    if value_3 == 0:
                        if value_2 == 1:
                            dir_old = dir_old
                        else:
                            dir_old = 0
                    if value_3 == 6:
                        if current_step !=prev_step:
                            dir_old = dir_path[prev_step-1]
                            current_step=prev_step
                    elif value_3 !=0:
                        dir_old = 1
                if prev_step >= len(dir_path):
                    dir_old=0
                    if room == 0:
                        print("haha")
                        key_last=1
                        dir_path=[]
                        steps_robot=[]
                        steps=[]
                        reward = 0
                        envstate = 0
                        num_step = 0
                        train = 0
                        first = 0
                        i = 0
                        j = 0
                        rat_cell = goal

                if dir_old==1:
                    if value_6 < 1000 and value_6 > 100:
                        dir_old=0
                    elif value_6>1000:
                        dir_old=dir_old

                if dir != dir_old or value_4 == 0 or key_last==1:
                    dir = dir_old
                    return_data = {'received_data': dir, 'nums_step': len(dir_path),'key_last':key_last}
                    return_str = json.dumps(return_data)
                    ser.write(return_str.encode('utf-8') + b'\n')


[today, currentTime, second] = tgian()
init_table = {'mode': [0],'prev_step':[0], 'x': [0], 'y':[0],'theta':[0], 'obstacle':[0]}
arduino_to_excel = pd.DataFrame(data=init_table, index=[currentTime])
name = create_file(today)
room=0
dir_path = []
dir =0
dir_old =0
num_step =0
current_step =0
prev_step =0
envstate=0
first=0
i=0
j=0
rat_cell = (0, 2)
key_last=0
maze = np.array([
    [0., 1., 1., 1., 0.],
    [0., 1., 1., 1., 0.],
    [0., 1., 1., 1., 0.],
    [1., 1., 1., 1., 1.],
    [1., 1., 1., 1., 1.],
    [1., 1., 1., 1., 1.],
    [1., 1., 1., 1., 1.],
    [1., 1., 1., 1., 1.]
])

visited_mark = 0.8  # Cells visited by the rat will be painted by gray 0.8
rat_mark = 0.5  # The current rat cell will be painteg by gray 0.5
LEFT = 0
UP = 1
RIGHT = 2
DOWN = 3

# Actions dictionary
actions_dict = {
    LEFT: 'left',
    UP: 'up',
    RIGHT: 'right',
    DOWN: 'down',
}

num_actions = len(actions_dict)
epsilon = 0.1
reward = 0

steps = []
steps_robot = []

goal=(0,0)
goal_string = str(goal)
goal_string = goal_string.replace(" ", "")

Control_Robot()