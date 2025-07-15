import math
import json

SPEED_MAX = 10
MARGIN_RAD = 0.025 * math.pi
MAX_FRACTION = 5
MARGIN_METER = 0.05

def get_direction(ball_vector: list) -> int:
    """Get direction to navigate robot to face the ball

    Args:
        ball_vector (list of floats): Current vector of the ball with respect
            to the robot.

    Returns:
        int: 0 = forward, -1 = right, 1 = left
    """
    if -0.13 <= ball_vector[1] <= 0.13:
        return 0
    return -1 if ball_vector[1] < 0 else 1

def head_to(self, rad):
    error_rad = (self.heading_rad + rad) % (2.0 * math.pi)
    if error_rad > math.pi:
        error_rad = -(2.0 * math.pi - error_rad)
    if abs(error_rad) > MARGIN_RAD:
        if error_rad > 0:
            self.left_motor.setVelocity(-min(SPEED_MAX, SPEED_MAX * error_rad))
            self.right_motor.setVelocity(min(SPEED_MAX, SPEED_MAX * error_rad))
        elif error_rad < 0:
            self.left_motor.setVelocity(min(SPEED_MAX, -SPEED_MAX * error_rad))
            self.right_motor.setVelocity(-min(SPEED_MAX, -SPEED_MAX * error_rad))
        return 0
    else:
        return 1

def goto_xy(self, x, y):
    global_x = x - self.robot_pos[0]
    global_y = y - self.robot_pos[1]
    dist = math.sqrt(global_x ** 2 + global_y ** 2)
    if dist < MARGIN_METER:
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        return 1
    if self.name[0] == 'B':
        theta = (math.pi / 2 - self.heading_rad)
    else:
        theta = -(self.heading_rad + math.pi / 2)
    local_x = global_x * math.cos(theta) - global_y * math.sin(theta)
    local_y = global_x * math.sin(theta) + global_y * math.cos(theta)

    if local_x > 0:
        if abs(local_y) <= MARGIN_METER:
            self.left_motor.setVelocity(SPEED_MAX)
            self.right_motor.setVelocity(SPEED_MAX)
        elif local_y > 0:
            self.left_motor.setVelocity(SPEED_MAX)
            self.right_motor.setVelocity(-SPEED_MAX)
        else:
            self.left_motor.setVelocity(-SPEED_MAX)
            self.right_motor.setVelocity(SPEED_MAX)
    else:
        if abs(local_y) <= MARGIN_METER:
            self.left_motor.setVelocity(-SPEED_MAX)
            self.right_motor.setVelocity(-SPEED_MAX)
        elif local_y > 0:
            self.left_motor.setVelocity(-SPEED_MAX)
            self.right_motor.setVelocity(SPEED_MAX)
        else:
            self.left_motor.setVelocity(SPEED_MAX)
            self.right_motor.setVelocity(-SPEED_MAX)
    return 0

def send_ball_info(self):
    if self.is_new_ball_data():
        ball_data = self.get_new_ball_data()
        self.ball_dir = ball_data["direction"]
        self.ball_dis = 0.0256 * (math.sqrt(1 - self.ball_dir[2]**2) / abs(self.ball_dir[2]))
        #print(f"{self.id} {self.ball_dis}")
        robot_x = round(self.robot_pos[0], MAX_FRACTION)
        robot_y = round(self.robot_pos[1], MAX_FRACTION)

        ball_x = self.ball_dis * self.ball_dir[0]
        ball_y = self.ball_dis * self.ball_dir[1]
        if self.name[0] == 'B':
            theta = -(math.pi / 2 - self.heading_rad)
        else:
            theta = (math.pi / 2 + self.heading_rad)
        dx = ball_x * math.cos(theta) - ball_y * math.sin(theta)
        dy = ball_x * math.sin(theta) + ball_y * math.cos(theta)
        ball_dx = round(dx, MAX_FRACTION)
        ball_dy = round(dy, MAX_FRACTION)

        data = {"id": self.id, "x": robot_x, "y": robot_y, "dx": ball_dx, "dy": ball_dy}
        packet = json.dumps(data)
        self.team_emitter.send(packet)
        return 1
    else:
        return 0
