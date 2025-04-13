import os
import json
from datetime import datetime
from subprocess import call

# Simulated MAVROS GPS data pull
def get_gps_data():
    return {
        "latitude": 12.345678,
        "longitude": 98.765432,
        "altitude": 42.0,
        "timestamp": datetime.utcnow().isoformat()
    }

def capture_image():
    timestamp = datetime.utcnow().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"image_{timestamp}.jpg"
    jsonname = f"image_{timestamp}.json"
    
    call(["libcamera-jpeg", "-o", filename])
    
    gps = get_gps_data()
    with open(jsonname, 'w') as f:
        json.dump(gps, f, indent=4)
    
    print(f"[+] Captured {filename} with GPS data")

if __name__ == "__main__":
    capture_image()
