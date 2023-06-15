import sys
import time
sys.path.append("build/")
from mtsp_drones_gym import Payload, Workspace

ws = Workspace(True)
ws.add_drone(x=0, y=0, radius=0.1, capacity=1)
ws.add_drone(x=-1, y=0, radius=0.1, capacity=1)
ws.add_payload(1, 1.4, 3, 2, 1)
ws.set_step_time(0.015)
for i in range(100):
    ws.step()
    time.sleep(0.015)
