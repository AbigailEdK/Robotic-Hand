# receive frame
# unwrap frame
# send packets of data to palm and finger objects
# get angles out of palm and finger objects
# assemble angles into frame
# send frame to robotic hand

# region Imports 
import numpy as np
import time
import csv
from handClass import Hand
from bluetoothUtils import get_full_hand_frame_async, send_command
import asyncio
# endregion


# region Calibration Vars
flatSamples = []
flatTimes = []
rightSamples = []
rightTimes = []
leftSamples = []
leftTimes = []
upsideDownSamples = []
upsideDownTimes = []

CSV_FILENAME = "palm_calibration.csv"
# endregion

# region Calibration Fns
async def calibrate_position(prompt, command, num_samples):
    print(prompt)
    await asyncio.to_thread(input, "Press Enter to start...")

    await send_command(command)

    samples = []
    timestamps = []

    if command == "CALIB1":
        print(f"Starting sensors...")
    else:
        print(f"Collecting {num_samples} samples...")
    
    for i in range(num_samples):
        frame = await get_full_hand_frame_async()
        
        # if command == "CALIB1":
        #     print(f"Collecting {num_samples} samples...")

        samples.append(frame)
        timestamps.append(time.perf_counter())
        
        # Print progress update (overwrite same line)
        print(f"\rProcessing Sample {i+1}/{num_samples}", end='', flush=True)

    # New line after progress is complete
    print()  
    await send_command(command)
    return np.array(samples), np.array(timestamps)

async def run_calibration(hand, num_samples, sample_types, recording=False):
    """Run full splay, roll, flex, and fist calibration, and save to CSV."""
    global flatSamples, flatTimes
    global rightSamples, rightTimes
    global leftSamples, leftTimes
    global upsideDownSamples, upsideDownTimes
    global fistSamples, fistTimes

    if "flat" in sample_types:
        flatSamples, flatTimes = await calibrate_position(
            "Calibrating flat position. Place palm flat and hold still.", command="CALIB1", num_samples=num_samples
        )
        print("Flat position recorded.")
    else: 
        flatSamples = np.zeros(num_samples)
        flatTimes = np.zeros(num_samples)

    if "right" in sample_types:
        rightSamples, rightTimes = await calibrate_position(
            "Calibrating right tilt position. Tilt hand to the right and hold still.", command="CALIB2", num_samples=num_samples
        )
        print("Right tilt position recorded.")
    else: 
        rightSamples = np.zeros(num_samples)
        rightTimes = np.zeros(num_samples)

    if "left" in sample_types:
        leftSamples, leftTimes = await calibrate_position(
            "Calibrating left tilt position. Tilt hand to the left and hold still.", command="CALIB2", num_samples=num_samples
        )
        print("Left tilt position recorded.")
    else: 
        leftSamples = np.zeros(num_samples)
        leftTimes = np.zeros(num_samples)

    if "upside_down" in sample_types:
        upsideDownSamples, upsideDownTimes = await calibrate_position(
            "Calibrating upside down position. Turn hand upside down and hold still.", command="CALIB2", num_samples=num_samples
        )
        print("Upside down position recorded.")
    else: 
        upsideDownSamples = np.zeros(num_samples)
        upsideDownTimes = np.zeros(num_samples)

    if "fist" in sample_types:
        fistSamples, fistTimes = await calibrate_position(
            "Calibrating fist position. Make a fist and hold still.", command="CALIB2", num_samples=num_samples
        )
        print("Fist position recorded.")
    else: 
        fistSamples = np.zeros(num_samples)
        fistTimes = np.zeros(num_samples)

    # # Save all samples to CSV
    # save_calibration_to_csv(
    #     CSV_FILENAME,
    #     flatSamples, flatTimes,
    #     rightSamples, rightTimes,
    #     leftSamples, leftTimes,
    #     upsideDownSamples, upsideDownTimes
    # )

    if not recording:
        hand.calibrate(sample_types, flatSamples, flatTimes, rightSamples, rightTimes, leftSamples, leftTimes, upsideDownSamples, upsideDownTimes, fistSamples, fistTimes)

    print("Calibration complete.")
    return hand

# region Save/Load CSV
# def save_calibration_to_csv(filename, splay, splay_t, fist, fist_t, roll, roll_t, flex, flex_t, move, move_t):
def save_calibration_to_csv(filename, flat, flat_t, right, right_t, left, left_t, upside_down, upside_down_t):
    """Save all calibration samples to a CSV file."""
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        # Header
        writer.writerow(["Type", "Timestamp", *[f"Val{i}" for i in range(len(flat[0]))]])
        # Helper to write
        def write_samples(name, samples, times):
            for sample, t in zip(samples, times):
                writer.writerow([name, t, *sample])
        write_samples("flat", flat, flat_t)
        write_samples("right", right, right_t)
        write_samples("left", left, left_t)
        write_samples("upside_down", upside_down, upside_down_t)

    print(f"Calibration data saved to {filename}. Completing calibration.")

def load_calibration_from_csv(hand, snip, filename = "palm_calibration.csv"):
    """Load calibration samples directly from a CSV and update global variables."""
    print(f"Loading calibration data from {filename}...")
    
    global flatSamples, flatTimes
    global rightSamples, rightTimes
    global leftSamples, leftTimes
    global upsideDownSamples, upsideDownTimes

    flatSamples, flatTimes = [], []
    rightSamples, rightTimes = [], []
    leftSamples, leftTimes = [], []
    upsideDownSamples, upsideDownTimes = [], []

    with open(filename, mode='r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row["Type"]
            t = float(row["Timestamp"])
            sample = np.array([float(row[f"Val{i}"]) for i in range(len(row) - 2)])
            if name == "flat":
                flatSamples.append(sample)
                flatTimes.append(t)
            elif name == "right":
                rightSamples.append(sample)
                rightTimes.append(t)
            elif name == "left":
                leftSamples.append(sample)
                leftTimes.append(t)
            elif name == "upside_down":
                upsideDownSamples.append(sample)
                upsideDownTimes.append(t)

    # Convert to numpy arrays
    flatSamples, flatTimes = np.array(flatSamples), np.array(flatTimes)
    rightSamples, rightTimes = np.array(rightSamples), np.array(rightTimes)
    leftSamples, leftTimes = np.array(leftSamples), np.array(leftTimes)
    upsideDownSamples, upsideDownTimes = np.array(upsideDownSamples), np.array(upsideDownTimes)

    # hand.calibrate(splaySamples, splayTimes, fistSamples, fistTimes, rollSamples, rollTimes, flexSamples, flexTimes, moveSamples, moveTimes)
    # hand.calibrate(flatSamples, flatTimes, rightSamples, rightTimes, leftSamples, leftTimes, upsideDownSamples, upsideDownTimes)
    # hand.calibrationBetaTuningCSV(flatSamples[:snip], flatTimes[:snip], rightSamples[:snip], rightTimes[:snip], leftSamples[:snip], leftTimes[:snip], upsideDownSamples[:snip], upsideDownTimes[:snip])

    return hand

# endregion
    


    