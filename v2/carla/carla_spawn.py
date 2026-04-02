#!/usr/bin/env python3
import argparse
import random
import time

import carla


def spawn_vehicle(host: str, port: int, vehicle_filter: str = "vehicle.tesla.model3") -> carla.Actor:
    client = carla.Client(host, port)
    client.set_timeout(10.0)

    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    # Get vehicle blueprint
    vehicle_bp = blueprint_library.filter(vehicle_filter)[0]
    vehicle_bp.set_attribute("role_name", "hero")

    # Get a random spawn point
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points available in the map")

    spawn_point = random.choice(spawn_points)

    # Spawn the vehicle
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print(f"Spawned vehicle: {vehicle.type_id} at {spawn_point.location}")

    return vehicle


def setup_spectator(world: carla.World, vehicle: carla.Actor):
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    spectator.set_transform(carla.Transform(
        transform.location + carla.Location(x=-10, z=5),
        carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)
    ))


def main():
    parser = argparse.ArgumentParser(description="Spawn a controllable vehicle in Carla")
    parser.add_argument("--host", default="localhost", help="Carla server host")
    parser.add_argument("--port", type=int, default=2000, help="Carla server port")
    parser.add_argument("--vehicle", default="vehicle.tesla.model3", help="Vehicle blueprint filter")
    parser.add_argument("--keep-alive", action="store_true", help="Keep script running to prevent vehicle despawn")
    args = parser.parse_args()

    print(f"Connecting to Carla server at {args.host}:{args.port}...")

    vehicle = spawn_vehicle(args.host, args.port, args.vehicle)

    client = carla.Client(args.host, args.port)
    world = client.get_world()
    setup_spectator(world, vehicle)

    print(f"\nVehicle spawned successfully!")
    print(f"  ID: {vehicle.id}")
    print(f"  Type: {vehicle.type_id}")
    print(f"  Role: {vehicle.attributes.get('role_name', 'unknown')}")

    if args.keep_alive:
        print("\nKeeping script alive. Press Ctrl+C to destroy the vehicle and exit.")
        try:
            while True:
                time.sleep(1)
                setup_spectator(world, vehicle)
        except KeyboardInterrupt:
            print("\nDestroying vehicle...")
            vehicle.destroy()
            print("Done.")
    else:
        print("\nVehicle will persist until Carla server is restarted or another script destroys it.")
        print("Run with --keep-alive to follow the vehicle with the spectator camera.")


if __name__ == "__main__":
    main()
