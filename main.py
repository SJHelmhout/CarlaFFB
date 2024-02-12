import carla
import random

import evdev
import pygame
import numpy as np
from evdev import ecodes, InputDevice, ff



def main():
    # Connect to the client and retrieve the world object
    client = carla.Client('localhost', 2000)
    world = client.get_world()

    # Set up the simulator in synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True  # Enables synchronous mode
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # Set up the TM in synchronous mode
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_synchronous_mode(True)

    # Set a seed so behaviour can be repeated if necessary
    traffic_manager.set_random_device_seed(0)
    random.seed(0)

    # We will aslo set up the spectator, so we can see what we do
    spectator = world.get_spectator()

    # Retrieve the map's spawn points
    spawn_points = world.get_map().get_spawn_points()

    # Select some models from the blueprint library
    models = ['dodge', 'audi', 'model3', 'mini', 'mustang', 'lincoln', 'prius', 'nissan', 'crown', 'impala']
    blueprints = []
    for vehicle in world.get_blueprint_library().filter('*vehicle*'):
        if any(model in vehicle.id for model in models):
            blueprints.append(vehicle)

    # Set a max number of vehicles and prepare a list for those we spawn
    max_vehicles = 50
    max_vehicles = min([max_vehicles, len(spawn_points)])
    vehicles = []

    # Take a random sample of the spawn points and spawn some vehicles
    for i, spawn_point in enumerate(random.sample(spawn_points, max_vehicles)):
        temp = world.try_spawn_actor(random.choice(blueprints), spawn_point)
        if temp is not None:
            vehicles.append(temp)

    # Parse the list of spawned vehicles and give control to the TM through set_autopilot()
    for vehicle in vehicles:
        vehicle.set_autopilot(True)
        # Randomly set the probability that a vehicle will ignore traffic lights
        traffic_manager.ignore_lights_percentage(vehicle, random.randint(0, 50))

    # # Adding NPC's
    # vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
    # spawn_points = world.get_map().get_spawn_points()
    # for i in range(0, 20):
    #     world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
    #
    # ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
    #
    # # Add sensors
    # # Create a transform to place the camera on top of the vehicle
    # camera_init_trans = carla.Transform(carla.Location(z=1.5))
    #
    # # We create the camera through a blueprint that defines its properties
    # camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    #
    # # We spawn the camera and attach it to our ego vehicle
    # camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)
    #
    # # Start camera with PyGame callback
    # camera.listen(lambda image: image.save_to_disk('out/%06d.png' % image.frame))
    #
    # # Animate vehicles with traffic manager
    # for vehicle in world.get_actors().filter('*vehicle*'):
    #     vehicle.set_autopilot(True)
    #
    # # Assign Ego Vehicle
    # ego_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
    #
    # ego_bp.set_attribute('role_name', 'hero')
    #
    # ego_vehicle = world.spawn_actor(ego_bp, random.choice(spawn_points))


if __name__ == '__main__':
    # main()
    for name in evdev.list_devices():
        dev = InputDevice(name)
        if ecodes.EV_FF in dev.capabilities():
            break

    print(dev.name)

    rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0xffff)
    effect_type = ff.EffectType(ff_rumble_effect=rumble)
    duration_ms = 1000

    effect = ff.Effect(
        ecodes.FF_RUMBLE, -1, 0,
        ff.Trigger(0, 0),
        ff.Replay(duration_ms, 0),
        effect_type
    )

    repeat_count = 1
    effect_id = dev.upload_effect(effect)
    dev.write(ecodes.EV_FF, effect_id, repeat_count)
    dev.erase_effect(effect_id)
