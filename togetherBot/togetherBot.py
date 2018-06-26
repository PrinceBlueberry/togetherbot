import math
import time

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket


class Vector3:
    def __init__(self, data):
        self.data = data

    def __sub__(self, value):
        return Vector3([self.data[0]-value.data[0], self.data[1]-value.data[1], self.data[2]-value.data[2]])

    def __mul__(self, value):
        return (self.data[0]*value.data[0] + self.data[1]*value.data[1] + self.data[2]*value.data[2])
    

class obj:
    def __init__(self):
        self.location = Vector3([0, 0, 0])
        self.velocity = Vector3([0, 0, 0])
        self.rotation = Vector3([0, 0, 0])
        self.rvelocity = Vector3([0, 0, 0])

        self.local_location = Vector3([0, 0, 0])
        self.boost = 0


class ATBA:
    def __init__(self):
        self.expired = False

    def execute(self, agent):
        target_location = agent.ball
        target_speed = velocity2D(agent.ball) + (distance2D(agent.ball, agent.me)/1.5)
        
        return agent.ATBAController(target_location, target_speed)


class GetBoost:
    def __init__(self):
        self.expired = False

    def execute(self, agent):
        #hard coded boost locations
        boost = []
        boost.append(Vector3([3584, 0, 0]))
        boost.append(Vector3([3072, 4096, 0]))
        boost.append(Vector3([-3072, 4096, 0]))
        boost.append(Vector3([-3584, 0, 0]))
        boost.append(Vector3([-3072, -4096, 0]))
        boost.append(Vector3([3072, -4096, 0]))

        closest_boost = Vector3([9000, 90000, 0])
        for x in boost:
            if distance2D(x, agent.me.location) < distance2D(closest_boost, agent.me.location):
                closest_boost = x

        return agent.simple_controller(closest_boost, 1400)


class TogetherBot(BaseAgent):
    #boost_pads[] = get_field_info()

    def initialize_agent(self):
        self.me = obj()
        self.ball = obj()
        self.start = time.time()

        self.state = ATBA()
        self.expired = True

    def get_output(self, game: GameTickPacket) -> SimpleControllerState:
        controller_state = SimpleControllerState()
        self.preprocess(game)
        self.state = self.get_state()
        return self.state.execute(self)

    def simple_controller(self, target_location, target_speed):
        controller_state = SimpleControllerState()
        local_location = [(target_location - self.me.location) * self.me.matrix[0], (target_location - self.me.location) * self.me.matrix[1]]
        angle_to_target = math.atan2(local_location[1], local_location[0])
        if angle_to_target > 0.1:
            controller_state.steer = controller_state.yaw = 1
        elif angle_to_target < -0.1:
            controller_state.steer = controller_state.yaw = -1
        else:
            controller_state.steer = controller_state.yaw = 0

        current_speed = velocity2D(self.me)
        if target_speed > current_speed:
            controller_state.throttle = 1.0
            if target_speed > 1400 and -0.5 < abs(angle_to_target) < 0.5 and current_speed < 2250:
                controller_state.boost = True
            else:
                controller_state.boost = False
        elif target_speed < current_speed:
            controller_state.throttle = 0

        return controller_state

    def ATBAController(self, target_object, target_speed):
        location = target_object.local_location
        controller_state = SimpleControllerState()
        angle_to_ball = math.atan2(location.data[1], location.data[0])

        current_speed = velocity2D(self.me)

        #steering
        if angle_to_ball > 0.1:
            controller_state.steer = controller_state.yaw = 1
        elif angle_to_ball < -0.1:
            controller_state.steer = controller_state.yaw = -1
        else:
            controller_state.steer = controller_state.yaw = 0

        #throttle
        if target_speed > current_speed:
            controller_state.throttle = 1.0
            if target_speed > 1400 and abs(angle_to_ball) < 1 and current_speed < 2250:
                controller_state.boost = True
        elif target_speed < current_speed:
            controller_state.throttle = 0

        #dodging
        time_difference = time.time() - self.start
        if time_difference > 2.2 and distance2D(target_object.location, self.me.location) > 1000 and abs(angle_to_ball) < 1.3:
            self.start = time.time()
        elif time_difference <= 0.1:
            controller_state.jump = True
            controller_state.pitch = -1
        elif 0.1 <= time_difference <= 0.15:
            controller_state.jump = False
            controller_state.pitch = -1
        elif 0.15 < time_difference < 1:
            controller_state.jump = True
            controller_state.yaw = controller_state.steer
            controller_state.pitch = -1

        return controller_state

    def preprocess(self, game):
        self.me.location.data = [game.game_cars[self.index].physics.location.x, game.game_cars[self.index].physics.location.y, game.game_cars[self.index].physics.location.z]
        self.me.velocity.data = [game.game_cars[self.index].physics.velocity.x, game.game_cars[self.index].physics.velocity.y, game.game_cars[self.index].physics.velocity.z]
        self.me.rotation.data = [game.game_cars[self.index].physics.rotation.pitch, game.game_cars[self.index].physics.rotation.yaw, game.game_cars[self.index].physics.rotation.roll]
        self.me.rvelocity.data = [game.game_cars[self.index].physics.angular_velocity.x, game.game_cars[self.index].physics.angular_velocity.y, game.game_cars[self.index].physics.angular_velocity.z]
        self.me.matrix = rotator_to_matrix(self.me)
        self.me.boost = game.game_cars[self.index].boost
        
        self.ball.location.data = [game.game_ball.physics.location.x, game.game_ball.physics.location.y, game.game_ball.physics.location.z]
        self.ball.velocity.data = [game.game_ball.physics.velocity.x, game.game_ball.physics.velocity.y, game.game_ball.physics.velocity.z]
        self.ball.rotation.data = [game.game_ball.physics.rotation.pitch, game.game_ball.physics.rotation.yaw, game.game_ball.physics.rotation.roll]
        self.ball.rvelocity.data = [game.game_ball.physics.angular_velocity.x, game.game_ball.physics.angular_velocity.y, game.game_ball.physics.angular_velocity.z]

        self.ball.local_location.data = to_local(self.ball, self.me)

    def get_state(self):
        go_state = self.state
        if self.expired:
            if self.me.boost < 7:
                go_state = GetBoost()
            else:
                go_state = ATBA()

        return go_state


def to_local(target_object, our_object):
    x = (target_object.location - our_object.location) * our_object.matrix[0]
    y = (target_object.location - our_object.location) * our_object.matrix[1]
    z = (target_object.location - our_object.location) * our_object.matrix[2]
    return [x, y, z]


def rotator_to_matrix(our_object):
    r = our_object.rotation.data
    CR = math.cos(r[2])
    SR = math.sin(r[2])
    CP = math.cos(r[0])
    SP = math.sin(r[0])
    CY = math.cos(r[1])
    SY = math.sin(r[1])

    matrix = []
    matrix.append(Vector3([CP*CY, CP*SY, SP]))
    matrix.append(Vector3([CY*SP*SR-CR*SY, SY*SP*SR+CR*CY, -CP * SR]))
    matrix.append(Vector3([-CR*CY*SP-SR*SY, -CR*SY*SP+SR*CY, CP*CR]))
    return matrix


def velocity2D(target_object):
    return math.sqrt(target_object.velocity.data[0]**2 + target_object.velocity.data[1]**2)


def distance2D(target_object, our_object):
    if isinstance(target_object, Vector3):
        difference = target_object - our_object
    else:
        difference = target_object.location - our_object.location
    return math.sqrt(difference.data[0]**2 + difference.data[1]**2)

