from utils import *
import math
import random
import json
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

class MyRobot3(RCJSoccerRobot):

    def defence(self):
        if self.name[0] == 'B':
            if random.randint(1, 2) == 1:
                if goto_xy(self, -0.35, 0.35):
                    head_to(self, -math.pi / 6)
            else:
                if goto_xy(self, -0.45, 0.45):
                    head_to(self, -math.pi / 6)
        else:
            if random.randint(1, 2) == 1:
                if goto_xy(self, -0.35, -0.35):
                    head_to(self, math.pi / 6)
            else:
                if goto_xy(self, -0.45, -0.45):
                    head_to(self, math.pi / 6)

    def run(self):

        self.id = 3

        last_ball_x = 0.0
        last_ball_y = 0.0
        last_ball_valid = False

        self.team_emitter = self.robot.getDevice("team emitter")
        self.team_receiver = self.robot.getDevice("team receiver")
        self.team_receiver.enable(TIME_STEP)

        while self.robot.step(TIME_STEP) != -1:

            self.robot_pos = self.get_gps_coordinates()
            self.heading_rad = self.get_compass_heading()

            ball_data = send_ball_info(self)

            while self.team_receiver.getQueueLength() > 0:
                packet = self.team_receiver.getString()
                self.team_receiver.nextPacket()
                data = json.loads(packet)

                if len(data.items()) != 2:
                    continue

                last_ball_valid = True

                for key, value in data.items():

                    if key == 'x':
                        last_ball_x = value
                        continue
                    if key == 'y':
                        last_ball_y = value
                        continue

            if ball_data:
                theta = 0
                if self.ball_dir[0] > 0.075:
                    if abs(self.heading_rad) > math.pi / 2 and self.ball_dis < 0.25:
                        if last_ball_x > 0:
                            if self.name[0] == 'B':
                                theta = math.pi / 3
                            else:
                                theta = -math.pi / 3
                        else:
                            if self.name[0] == 'B':
                                theta = -math.pi / 3
                            else:
                                theta = math.pi / 3
                elif self.ball_dir[0] < -0.075:
                    if abs(self.heading_rad) < math.pi / 2 and self.ball_dis < 0.25:
                        if last_ball_x > 0:
                            if self.name[0] == 'B':
                                theta = math.pi / 3
                            else:
                                theta = -math.pi / 3
                        else:
                            if self.name[0] == 'B':
                                theta = -math.pi / 3
                            else:
                                theta = math.pi / 3
                ghost_ball_x = self.ball_dir[0] * math.cos(theta) - self.ball_dir[1] * math.sin(theta)
                ghost_ball_y = self.ball_dir[0] * math.sin(theta) + self.ball_dir[1] * math.cos(theta)
                self.ball_dir[0] = ghost_ball_x
                self.ball_dir[1] = ghost_ball_y

                direction = get_direction(self.ball_dir)

                if self.ball_dir[0] > 0:
                    if direction == 0:
                        left_speed = SPEED_MAX
                        right_speed = SPEED_MAX
                    else:
                        left_speed = direction * SPEED_MAX
                        right_speed = direction * -SPEED_MAX
                else:
                    if direction == 0:
                        left_speed = -SPEED_MAX
                        right_speed = -SPEED_MAX
                    else:
                        left_speed = direction * -SPEED_MAX
                        right_speed = direction * SPEED_MAX

                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
            else:
                if self.name[0] == 'B':
                    if last_ball_y < 0:
                        if last_ball_x > 0:
                            goto_xy(self, last_ball_x + MARGIN_METER, last_ball_y + MARGIN_METER)
                        else:
                            goto_xy(self, last_ball_x - MARGIN_METER, last_ball_y + MARGIN_METER)
                    else:
                        self.defence()
                else:
                    if last_ball_y > 0:
                        if last_ball_x > 0:
                            goto_xy(self, last_ball_x + MARGIN_METER, last_ball_y - MARGIN_METER)
                        else:
                            goto_xy(self, last_ball_x - MARGIN_METER, last_ball_y - MARGIN_METER)
                    else:
                        self.defence()
