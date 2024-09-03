# for computing the wheel calibration parameters
import numpy as np
import os
import sys
sys.path.insert(0, "../util")
# Import robot



def calibrateWheelRadius():
    # Compute the robot scale parameter using a range of wheel velocities.
    # For each wheel velocity, the robot scale parameter can be computed
    # by comparing the time and distance driven to the input wheel velocities.

    ##########################################
    # Feel free to change the range / step
    ##########################################
    wheel_velocities_range = range(20, 80, 15) # Ticks/s
    time = 5 #Fixed drive time in seconds
    scale =0

    for vel in wheel_velocities_range:
        print("Driving at {} ticks/s".format(vel))

        # Drive the robot at the given speed for the given time
        # TODO: Insert command for robot to drive at vel for time 

        while True:
            dist = input("Enter the distance travelled in meters: ")

            # Check that input is a number
            try: dist = float(dist)
            except ValueError: print("Distance must be a number")

            
            uInput = input(f"Confirm robot travelled {dist}m? [y/N]")
            if uInput == 'y':
                scale += dist/(vel*time)
                print(f"Recording that robot drove {dist}m in {time} seconds at wheel speed {vel}")
                break
    
    scale = scale / len(wheel_velocities_range) # Average the measurements
    print(f"The scale is estimated to be {scale:.6f} m/tick")
    return scale






def calibrateBaseline(scale):
    # Compute the robot basline parameter using a range of wheel velocities.
    # For each wheel velocity, the robot baseline parameter can be computed by
    # comparing the time elapsed and rotation completed to the input wheel
    # velocities to find out the distance between the wheels.

    ##########################################
    # Feel free to change the range / step
    ##########################################
    wheel_velocities_range = range(30, 60, 10)
    time = 5 # Fxied time in seconds
    baseline = 0

    for vel in wheel_velocities_range:
        print("Driving at {} ticks/s.".format(vel))

        # Rotate the robot at the given speed for the given time
        # TODO: Insert command for robot to rotate at vel for time 

        while True:
            theta = input("Input the angle in degrees: ")
            try: theta = float(theta)
            except ValueError:
                print("Angle must be a number")

            uInput = input(f"Confirm robot turned {theta}degrees? [y/N]")
            if uInput == 'y':
                baseline += (scale * vel * time * theta)/(360*np.pi) # TODO: Check this formula
                print(f"Recording that robot turned {theta}degrees in {time} seconds at wheel speed {vel}")
                break
    
    bseline = baseline / len(wheel_velocities_range) # Take average
    print(f"The baseline is estimated to be {baseline:.6f}m")
    return baseline
            
           

            

if __name__ == "__main__":
    # import argparse

    # parser = argparse.ArgumentParser()
    # parser.add_argument("--ip", metavar='', type=str, default='localhost')
    # parser.add_argument("--port", metavar='', type=int, default=40000)
    # args, _ = parser.parse_known_args()

    # ppi = PenguinPi(args.ip,args.port)

    # calibrate pibot scale and baseline
    dataDir = "{}/param/".format(os.getcwd())

    print('Calibrating scale...\n')
    scale = calibrateWheelRadius()
    fileNameS = "{}scale.txt".format(dataDir)
    np.savetxt(fileNameS, np.array([scale]), delimiter=',')

    print('Calibrating PiBot baseline...\n')
    baseline = calibrateBaseline(scale)
    fileNameB = "{}baseline.txt".format(dataDir)
    np.savetxt(fileNameB, np.array([baseline]), delimiter=',')

    print('Finished calibration')