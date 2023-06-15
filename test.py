import sys
sys.path.append("build/")
from mtsp_drones_gym import Payload, Workspace

Payload(1, 3, 1, 2, 2)
ws = Workspace(True)
ws.add_drone(x=1, y=3, radius=1, capacity=1)
ws.add_payload(1, 2, 3, 4, 5)
