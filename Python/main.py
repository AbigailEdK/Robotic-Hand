from handPipeline import run_calibration, load_calibration_from_csv, calibrate_position
import asyncio
import time
import pybullet as p
import pybullet_data
import os
from bluetoothUtils import connect_ble, disconnect_ble, get_full_hand_frame_async, send_command
from handClass import Hand
import numpy as np
import collections

# region Pybullet Setup
# urdf_dir = r"H:\Y4 S2\Skripsie\SkripsieCode\TestRig\Python\GeorgeControl"
urdf_dir = r"/Users/abigail/Desktop/Robotic Hand/Python/George"
urdf_path = os.path.join(urdf_dir, "GeorgeHands.urdf")
# endregion

# region Simulation Parameters
debug = True
samplesToTake = 100
mode = "simulation"  # "simulation" or "plot"
# endregion

# region main
async def main():
    if mode == "simulation":
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) # To load other files from pybullet_data
        robot = p.loadURDF(urdf_path, useFixedBase=True)
        
        # 1. Connect BLE
        if not await connect_ble():
            return
        
        hand = Hand(sample_rate=100)

        hand = await run_calibration(hand, num_samples=samplesToTake, sample_types=["flat"], recording=False)

        await send_command("POLL")

        frame_times = collections.deque(maxlen=100)
        last_frame_time = None
        smoothed_freq = None
        counter = 0
        start_time = time.perf_counter()

        try:
            while True:
                frame = await get_full_hand_frame_async()

                if frame is not None:
                    
                    counter += 1

                    # > Measure incoming sample rate
                    now = time.perf_counter()

                    if last_frame_time is not None:
                        dt = now - last_frame_time
                        frame_times.append(dt)

                        # > Update sample rate estimate every 5 frames
                        if len(frame_times) > 5:
                            avg_dt = sum(frame_times) / len(frame_times)
                            smoothed_freq = 1.0 / avg_dt
                            
                            hand.updateSampleRate(smoothed_freq)
                    
                    last_frame_time = now
                    
                    joint_angles = hand.getJointAngles(frame)
                    set_hand_joint_angles(joint_angles, robot)

                # > Step the simulation
                p.stepSimulation()
                await asyncio.sleep(0.001)  # Non-blocking sleep for event loop

        except Exception as e:
            print(f"Stopping simulation ({type(e).__name__}: {e}).")
        
        finally:
            await disconnect_ble()
            p.disconnect()
            current_time = time.strftime("%Y%m%d_%H%M%S")
            file_name = f"palm_data_{current_time}.csv"
            hand.savePalmDataToCSV(file_name)

    elif mode == "plot":
        hand.plotPalmDataFromCSV("palm_data.csv")        
# endregion

# region Pybullet
def set_hand_joint_angles(angles_dict, robot):
    if angles_dict is None or not p.isConnected():
        return
    
    for joint_index, angle in angles_dict.items():
        try:
            p.setJointMotorControl2( # sends a command to the PyBullet motor at each joint:
                bodyUniqueId=robot,
                jointIndex=joint_index, 
                controlMode=p.POSITION_CONTROL, # move the joint to a specific angle 
                targetPosition=angle, # the desired rotation
                force=5.0 # maximum torque the motor can apply to reach the target
            )
        except Exception as e:
            # If PyBullet connection is lost, skip joint control
            break
# endregion

if __name__ == "__main__":
    result = asyncio.run(main())  # Single event loop for everything

