from handPipelineDebug import run_calibration, load_calibration_from_csv
import asyncio
import time
import pybullet as p
import pybullet_data
import os
from bluetoothUtilsDebug import connect_ble, disconnect_ble, get_full_hand_frame_async, send_command
from handDebug import Hand
import numpy as np
import collections

# region Pybullet Setup
urdf_dir = r"H:\Y4 S2\Skripsie\SkripsieCode\TestRig\Python\GeorgeControl"
# urdf_dir = r"/Users/abigail/Desktop/Skripsie Code/TestRig/Python/GeorgeControl"
urdf_path = os.path.join(urdf_dir, "GeorgeHands.urdf")
# endregion

# region Simulation Parameters
debug = True
samplesToTake = 5
# endregion

# region main
async def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) # To load other files from pybullet_data
        robot = p.loadURDF(urdf_path, useFixedBase=True)
        
        # 1. Connect BLE
        if not await connect_ble():
            return
    
    hand = Hand(sample_rate=100)

    hand = await calibrateHand(hand, num_samples=200)

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
                
                joint_angles = hand.frame_to_joint_angles(frame)
                
                set_hand_joint_angles(joint_angles, robot)

            # > Step the simulation
            p.stepSimulation()
            await asyncio.sleep(0.001)  # Non-blocking sleep for event loop

    except Exception as e:
        print(f"Stopping simulation ({type(e).__name__}: {e}).")
    
    finally:
        await disconnect_ble()
        hand.saveSmoothedDataToCSV('current_simulation_data.csv')
        p.disconnect()
# endregion
    
