from utils import *
import math
import json
import numpy as np
import sys
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

class MyRobot1(RCJSoccerRobot):

    def calc_ball_pos(self):

        self.prev_last_ball_x = self.last_ball_x
        self.prev_last_ball_y = self.last_ball_y

        if self.valid_2 and self.valid_3:
            self.valid_2 = False
            self.valid_3 = False
        else:
            if self.is_new_ball_data():
                ball_data = self.get_new_ball_data()
                ball_dir = ball_data["direction"]
                ball_dis = abs(0.01666666/(abs(ball_dir[2])/math.sqrt(1 - ball_dir[2]**2)))

                robot_x = round(self.robot_pos[0], MAX_FRACTION)
                robot_y = round(self.robot_pos[1], MAX_FRACTION)

                ball_x = ball_dis * ball_dir[0]
                ball_y = ball_dis * ball_dir[1]
                if self.name[0] == 'B':
                    theta = -(math.pi / 2 - self.heading_rad)
                else:
                    theta = (math.pi / 2 + self.heading_rad)
                dx = ball_x * math.cos(theta) - ball_y * math.sin(theta)
                dy = ball_x * math.sin(theta) + ball_y * math.cos(theta)
                if self.valid_2:
                    self.valid_2 = False
                    self.x3 = self.robot_pos[0]
                    self.y3 = self.robot_pos[1]
                    self.dx3 = dx
                    self.dy3 = dy
                elif self.valid_3:
                    self.valid_3 = False
                    self.x2 = self.robot_pos[0]
                    self.y2 = self.robot_pos[1]
                    self.dx2 = dx
                    self.dy2 = dy
                else:
                    self.last_ball_x = self.robot_pos[0] + dx
                    self.last_ball_y = self.robot_pos[1] + dy
                    self.last_ball_dx = self.last_ball_x - self.prev_last_ball_x
                    self.last_ball_dy = self.last_ball_y - self.prev_last_ball_y
                    return
            else:
                if self.valid_2:
                    self.valid_2 = False
                    self.last_ball_x = self.x2 + self.dx2
                    self.last_ball_y = self.y2 + self.dy2
                    self.last_ball_dx = self.last_ball_x - self.prev_last_ball_x
                    self.last_ball_dy = self.last_ball_y - self.prev_last_ball_y
                    return
                elif self.valid_3:
                    self.valid_3 = False
                    self.last_ball_x = self.x3 + self.dx3
                    self.last_ball_y = self.y3 + self.dy3
                    self.last_ball_dx = self.last_ball_x - self.prev_last_ball_x
                    self.last_ball_dy = self.last_ball_y - self.prev_last_ball_y
                    return
                else:
                    self.last_ball_x = self.prev_last_ball_x + self.last_ball_dx
                    self.last_ball_y = self.prev_last_ball_y + self.last_ball_dy
                    if abs(self.last_ball_x) > 0.65:
                        self.last_ball_dx = -self.last_ball_dx / 1.2
                        self.last_ball_dy = self.last_ball_dy / 1.2
                    if abs(self.last_ball_y) > 0.75:
                        self.last_ball_dx = self.last_ball_dx / 1.2
                        self.last_ball_dy = -self.last_ball_dy / 1.2
                    return

        try:
            alpha2 = self.dy2 / self.dx2
        except ZeroDivisionError:
            alpha2 = self.dy2 / sys.float_info.epsilon

        betha2 = self.y2 - alpha2 * self.x2

        try:
            alpha3 = self.dy3 / self.dx3
        except ZeroDivisionError:
            alpha3 = self.dy3 / sys.float_info.epsilon

        betha3 = self.y3 - alpha3 * self.x3

        A = np.array([[-alpha2, 1], [-alpha3, 1]])
        B = np.array([betha2, betha3])

        # Solve the system
        solution = np.linalg.solve(A, B)

        self.last_ball_x = float(solution[0])
        self.last_ball_y = float(solution[1])
        self.last_ball_dx = self.last_ball_x - self.prev_last_ball_x
        self.last_ball_dy = self.last_ball_y - self.prev_last_ball_y

        return

    def run(self):

        self.x2 = 0.0
        self.y2 = 0.0
        self.dx2 = 0.0
        self.dy2 = 0.0
        self.valid_2 = False
        self.x3 = 0.0
        self.y3 = 0.0
        self.dx3 = 0.0
        self.dy3 = 0.0
        self.valid_3 = False
        self.last_ball_x = 0.0
        self.last_ball_y = 0.0
        self.prev_last_ball_x = 0.0
        self.prev_last_ball_y = 0.0
        self.last_ball_dx = 0.0
        self.last_ball_dy = 0.0

        target_x = 0.0
        target_y = 0.0

        self.team_emitter = self.robot.getDevice("team emitter")
        self.team_receiver = self.robot.getDevice("team receiver")
        self.team_receiver.enable(TIME_STEP)

        while self.robot.step(TIME_STEP) != -1:

            while self.team_receiver.getQueueLength() > 0:
                packet = self.team_receiver.getString()
                self.team_receiver.nextPacket()
                data = json.loads(packet)

                if len(data.items()) != 5:
                    continue

                id = 2
                for key, value in data.items():

                    if key == 'id':
                        id = value
                        if id == 2:
                            self.valid_2 = True
                        else:
                            self.valid_3 = True
                        continue
                    if key == 'x':
                        if id == 2:
                            self.x2 = value
                        else:
                            self.x3 = value
                        continue
                    if key == 'y':
                        if id == 2:
                            self.y2 = value
                        else:
                            self.y3 = value
                        continue
                    if key == 'dx':
                        if id == 2:
                            self.dx2 = value
                        else:
                            self.dx3 = value
                        continue
                    if key == 'dy':
                        if id == 2:
                            self.dy2 = value
                        else:
                            self.dy3 = value
                        continue

            self.robot_pos = self.get_gps_coordinates()
            self.heading_rad = self.get_compass_heading()

            self.calc_ball_pos()
            if self.last_ball_x > 0.65:
                self.last_ball_x = 0.65
            if self.last_ball_x < -0.65:
                self.last_ball_x = -0.65
            if self.last_ball_y > 0.75:
                self.last_ball_y = 0.75
            if self.last_ball_y < -0.75:
                self.last_ball_y = -0.75
            self.last_ball_x = round(self.last_ball_x, MAX_FRACTION)
            self.last_ball_y = round(self.last_ball_y, MAX_FRACTION)
            data = {"x": self.last_ball_x, "y": self.last_ball_y}
            #print(data)
            packet = json.dumps(data)
            self.team_emitter.send(packet)

            if self.name[0] == 'B':
                target_x = ((0.8 - self.robot_pos[1]) * self.last_ball_x) / (0.8 - self.last_ball_y)
                if abs(target_x) < 0.3:
                    target_y = 0.5
                    target_rad = math.pi / 2
                else:
                    if target_x < 0:
                        target_x = max(target_x, -0.5)
                    else:
                        target_x = min(target_x, 0.5)
                    target_y = 0.7
                    target_rad = 0
                if goto_xy(self, target_x, max(self.last_ball_y, target_y)):
                    head_to(self, target_rad)
            else:
                target_x = ((0.8 + self.robot_pos[1]) * self.last_ball_x) / (0.8 + self.last_ball_y)
                if abs(target_x) < 0.3:
                    target_y = -0.5
                    target_rad = math.pi / 2
                else:
                    if target_x < 0:
                        target_x = max(target_x, -0.5)
                    else:
                        target_x = min(target_x, 0.5)
                    target_y = -0.7
                    target_rad = 0
                if goto_xy(self, target_x, min(self.last_ball_y, target_y)):
                    head_to(self, target_rad)
