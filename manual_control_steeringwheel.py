#!/usr/bin/env python

# Copyright (c) 2019 Intel Labs
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control with steering wheel Logitech G29.

To drive start by preshing the brake pedal.
Change your wheel_config.ini according to your steering wheel.

To find out the values of your steering wheel use jstest-gtk in Ubuntu.

"""

from __future__ import print_function

import glob
import math
import os
import sys

import numpy

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla
import argparse
import logging
import random
from HUD.CameraManager import CameraManager
from HUD.HUD import HUD
from Helpers.HelperFunctions import get_actor_display_name, find_weather_presets
from Sensors.CollisionSensor import CollisionSensor
from Sensors.GnssSensor import GnssSensor
from Sensors.LaneInvasionSensor import LaneInvasionSensor

if sys.version_info >= (3, 0):

    from configparser import ConfigParser

else:

    from ConfigParser import RawConfigParser as ConfigParser

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


class World(object):
    def __init__(self, carla_world, hud, actor_filter):
        self.world = carla_world
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.restart()
        self.world.on_tick(hud.on_world_tick)

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()


# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # initialize Joysticks
        pygame.joystick.init()
        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        for joystick in joysticks:
            print(joystick.get_name())
            print(joystick.get_id())
            if joystick.get_name().__contains__("wheel"):
                wheel_id = joystick.get_id()
            elif joystick.get_name().lower().__contains__("pedal"):
                pedal_id = joystick.get_id()
            else:
                gear_shift_id = joystick.get_id()

        # Steering wheel joystick
        self._joystick_steering_wheel = pygame.joystick.Joystick(wheel_id)
        # Gear Shift joystick
        self._joystick_gear_shift = pygame.joystick.Joystick(gear_shift_id)
        # Pedals joystick
        self._joystick_pedals = pygame.joystick.Joystick(pedal_id)

        self._joystick_steering_wheel.init()
        self._joystick_gear_shift.init()
        self._joystick_pedals.init()

        self._parser = ConfigParser()
        self._parser.read('wheel_config.ini')
        self._steer_idx = int(
            self._parser.get('T300 Steering Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('FANATEC ClubSportPedal', 'throttle'))
        self._brake_idx = int(self._parser.get('FANATEC ClubSportPedal', 'brake'))
        self._reverse_idx = int(self._parser.get('T500 RS Gear Shift', 'reverse'))
        self._handbrake_idx = int(
            self._parser.get('G29 Racing Wheel', 'handbrake'))

    def parse_events(self, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    world.restart()
                elif event.button == 1:
                    world.hud.toggle_info()
                elif event.button == 2:
                    world.camera_manager.toggle_camera()
                elif event.button == 3:
                    world.next_weather()
                elif event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == 23:
                    world.camera_manager.next_sensor()

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif K_0 < event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification('%s Transmission' %
                                               ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                self._parse_pedals()
                self._parse_gear_shift()
                self._control.reverse = self._control.gear < 0
                self._calculate_slip_angle(world)
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick_steering_wheel.get_numaxes()
        jsInputs = [float(self._joystick_steering_wheel.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick_steering_wheel.get_button(i)) for i in
                     range(self._joystick_steering_wheel.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        # K1 = 1.0  # 0.55
        # steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        self._control.steer = jsInputs[self._steer_idx]
        # toggle = jsButtons[self._reverse_idx]
        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

    def _parse_gear_shift(self):
        numAxes = self._joystick_gear_shift.get_numaxes()
        jsInputs = [float(self._joystick_gear_shift.get_axis(i)) for i in range(numAxes)]
        jsButtons = [float(self._joystick_gear_shift.get_button(i)) for i in
                     range(self._joystick_gear_shift.get_numbuttons())]

    def _parse_pedals(self):
        numAxes = self._joystick_pedals.get_numaxes()
        jsInputs = [float(self._joystick_pedals.get_axis(i)) for i in range(numAxes)]

        # throttleCmd = 0.0000152592547 * jsInputs[self._throttle_idx]
        throttleCmd = (1 + jsInputs[self._throttle_idx]) / 2
        brakeCmd = (1 + jsInputs[self._brake_idx]) / 2

        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd

    def _calculate_slip_angle(self, world):
        ego_vel_vect = world.player.get_velocity()
        trans_mat = numpy.array(world.player.get_transform().get_matrix()).reshape(4, 4)
        rot_mat = trans_mat[0:3, 0:3]
        inv_rot_mat = rot_mat.T
        vel_vec = numpy.array([ego_vel_vect.x,
                               ego_vel_vect.y,
                               ego_vel_vect.z]).reshape(3, 1)
        vel_in_v = inv_rot_mat @ vel_vec
        longitudinal_velocity = vel_in_v.item(0)
        lateral_velocity = vel_in_v.item(1)
        print("Longitudinal (forward): %f \n Lateral (sideways): %f" % (longitudinal_velocity, lateral_velocity))
        # Slip angle is in rads
        if longitudinal_velocity != 0:
            slip_angle = -math.atan((lateral_velocity / longitudinal_velocity))
        else:
            slip_angle = 0
        print("Slip angle: %f" % math.degrees(slip_angle))
        return slip_angle

    # def _calculate_slip_ratio(self, world):
    #     road_condition = 'Dry Tarmac'
    #     vehicle_physics = world.player.get_physics_control()
    #     mass = vehicle_physics.mass
    #     gravity = 9.81
    #     parser = ConfigParser()
    #     parser.read('Magic_Formula_Config.ini')
    #     B = float(self.parser.get(road_condition, 'B'))
    #     C = float(self.parser.get(road_condition, 'C'))
    #     D = float(self.parser.get(road_condition, 'D'))
    #     E = float(self.parser.get(road_condition, 'E'))
    #     friction_coeff = D * np.sin(C * np.arctan(B * (1 - E) * (wheel_slip)) + (E / B * np.arctan(B * (wheel_slip))))
    #     longitudinal_force = - friction_coeff * mass * gravity

    # def _calc_align_moment(self, world):
    #     front_lateral = 0
    #     normal_load = world.player.get_physics_control().mass * 9.81
    #     camber = 0
    #     slip_angle = np.degrees(self._calculate_slip_angle(world))
    #     a1 = -2.72
    #     a2 = -2.28
    #     a3 = -1.86
    #     a4 = -2.73
    #     a5 = 0.110
    #     a6 = -0.070
    #     a7 = 0.643
    #     a8 = -4.04
    #     a9 = 0.015
    #     a10 = -0.066
    #     a11 = 0.945
    #     a12 = 0.030
    #     a13 = 0.070
    #     c = 2.4
    #     d = a1 * math.pow(normal_load, 2) + a2 * normal_load
    #     # bcd = a3 * np.sin(a4 * np.arctan(a5 * mass))
    #     bcd = (a3 * math.pow(normal_load, 2) + a4 * normal_load) / math.pow(np.e, (a5 * normal_load))
    #     b = bcd / c * d
    #     e = a6 * math.pow(normal_load, 2) + a7 * normal_load + a8
    #
    #     sh = a9 * camber
    #     sv = (a10 * math.pow(normal_load, 2) + a11 * normal_load) * camber
    #     delta_b = -a12 * abs(camber) * b
    #     delta_e = (e / 1 - a13 * abs(camber)) - e
    #
    #     phi = (1 - e) * (slip_angle + sh) + (e / b) * np.arctan(b * (slip_angle + sh))
    #     return d * np.sin(c * np.arctan(b * phi)) + sv

    def _calc_front_lateral(self, world):
        tire_cornering_stiffness = 0
        surface_friction_coefficient = 0
        normal_load = world.player.get_physics_control().mass
        slip_angle = self._calculate_slip_angle(world)
        first_part = -tire_cornering_stiffness * np.tan(slip_angle)
        second_part = (math.pow(tire_cornering_stiffness, 2) / 3 * surface_friction_coefficient * normal_load)
        third_part = np.tan(slip_angle) * abs(np.tan(slip_angle))
        fourth_part = (
                    math.pow(tire_cornering_stiffness, 3) / 27 * math.pow((surface_friction_coefficient * normal_load),
                                                                          2))
        fifth_part = math.pow(np.tan(slip_angle), 3)
        return first_part + second_part * third_part - fourth_part * fifth_part

    # def _calc_pneumatic_trail(self, world):
    #     pneumatic_trail_zero_slip = 0
    #     tire_cornering_stiffness = 0
    #     surface_friction_coefficient = 0
    #     front_normal_load = world.player.get_physics_control().mass / 2
    #
    #     slip_angle = self._calculate_slip_angle(world)
    #     return (pneumatic_trail_zero_slip - np.sign(slip_angle) *
    #             ((pneumatic_trail_zero_slip * tire_cornering_stiffness) /
    #              3 * surface_friction_coefficient * front_normal_load) * np.tan(slip_angle))

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(args.width, args.height)
        world = World(client.get_world(), hud, args.filter)
        controller = DualControl(world, args.autopilot)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(world, clock):
                return
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:
        if world is not None:
            world.destroy()
        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
