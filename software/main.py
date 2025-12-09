# main.py - orchestrator (dry-run and connect mode)
import argparse
import logging
import time
from flight_control import connect_vehicle, arm_and_takeoff, goto_location, return_to_launch, land_now, check_gps_lock, check_battery, failsafe_routine
from comms import receive_coordinates_from_ambulance

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("main")

def ambulance_command_handler(obj):
    lat = obj.get('lat')
    lon = obj.get('lon')
    if lat and lon:
        logger.info(f"Handler: commanding drone to a point ahead of ambulance {lat},{lon}")
        # small offset: go ~0.0005 degrees north
        goto_location(lat + 0.0005, lon, 20)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', type=str, default=None, help='connection string for vehicle (SITL or real)')
    args = parser.parse_args()

    # start listening to ambulance commands
    receive_coordinates_from_ambulance(handler=ambulance_command_handler)

    if args.connect:
        logger.info("Connecting to vehicle...")
        connect_vehicle(args.connect)
        if not check_gps_lock():
            logger.warning("GPS lock issue")
        if not check_battery():
            logger.warning("Battery low")
        logger.info("Arm and takeoff to 20m")
        arm_and_takeoff(20)
        # hover for demo then RTL
        time.sleep(10)
        return_to_launch()
    else:
        logger.info("No connection specified - DRY RUN mode. Simulating mission.")
        for i in range(3):
            logger.info(f"Simulated step {i+1}/3")
            time.sleep(1)
        logger.info("Simulation done.")

if __name__ == '__main__':
    main()
