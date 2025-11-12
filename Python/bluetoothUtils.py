# ble_utils.py
import asyncio
from bleak import BleakClient, BleakScanner
import threading

DEVICE_NAME = "ESP32S3_BLE"
CHARACTERISTIC_UUID = "abcd1234-5678-90ab-cdef-1234567890ab"
WRITE_CHAR_UUID = "abcd5678-1234-90ab-cdef-1234567890ab"

hand_frame_lock = threading.Lock()
hand_frame_buffer = []

full_frame_length = 9 + 6*5

# Global client for persistent connection
ble_client = None

def handle_notification(sender, data: bytes):
    """BLE callback: decode data and append to buffer."""
    global hand_frame_buffer
    try:
        s = data.decode('utf-8').strip()
        floats = [float(x) for x in s.split(',') if x]

        with hand_frame_lock:
            hand_frame_buffer.extend(floats)
    except Exception as e:
        print("Error decoding notification:", e)

def get_full_hand_frame():
    """Return a full hand frame if available, else None."""
    global hand_frame_buffer
    with hand_frame_lock:
        if len(hand_frame_buffer) >= full_frame_length:
            frame = hand_frame_buffer[:full_frame_length]
            hand_frame_buffer = hand_frame_buffer[full_frame_length:]
            return frame
        else:
            return None

async def get_full_hand_frame_async():
    """Async version to await until a full frame is available."""
    while True:
        frame = get_full_hand_frame()
        if frame is not None:
            return frame
        await asyncio.sleep(0.001)

async def find_device_by_name(name: str):
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == name:
            return d.address
    return None

async def connect_ble(device_name=DEVICE_NAME, uuid=CHARACTERISTIC_UUID):
    """Persistent BLE connection."""
    global ble_client
    address = await find_device_by_name(device_name)
    if address is None:
        print(f"Device {device_name} not found.")
        return False

    ble_client = BleakClient(address)
    await ble_client.connect()
    if ble_client.is_connected:
        print(f"Connected to {address}")
        await ble_client.start_notify(uuid, handle_notification)
        return True
    return False

async def disconnect_ble(uuid=CHARACTERISTIC_UUID):
    """Disconnect BLE client with robust error handling."""
    global ble_client
    if ble_client:
        try:
            if ble_client.is_connected:
                # Stop notifications first
                try:
                    await asyncio.wait_for(ble_client.stop_notify(uuid), timeout=1.0)
                except (asyncio.TimeoutError, Exception):
                    pass  # Ignore timeout or other errors during stop_notify
                
                # Disconnect
                try:
                    await asyncio.wait_for(ble_client.disconnect(), timeout=2.0)
                    print("Disconnected BLE")
                except (asyncio.TimeoutError, Exception):
                    print("BLE disconnect timeout (forced cleanup)")
            
            # Always clear the client reference
            ble_client = None
            
            # Give background threads time to clean up
            await asyncio.sleep(0.1)
            
        except Exception as e:
            print(f"BLE disconnect error (continuing anyway): {e}")
            ble_client = None

async def send_command(command: str, uuid=WRITE_CHAR_UUID):
    """Send a command over BLE using persistent client."""
    global ble_client
    if ble_client is None or not ble_client.is_connected:
        print("BLE client not connected!")
        return False
    try:
        await ble_client.write_gatt_char(uuid, command.encode('utf-8'))
        print(f"Command sent: {command}")
        return True
    except Exception as e:
        print(f"Error sending command: {e}")
        return False
