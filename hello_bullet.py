import pybullet as p
import time
import pybullet_data

HALF_PI = 1.57079632679
G = 9.81
FPS = 120.

physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally
p.setGravity(0, 0, -G)
planeId = p.loadURDF("plane.urdf")

forkbotStartPos = [0, 0, 0]
# cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
forkbotStartOrientation = p.getQuaternionFromEuler([HALF_PI, 0, 0])
forkbotId = p.loadURDF("forkbug.urdf", forkbotStartPos, forkbotStartOrientation, 
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   flags=p.URDF_USE_INERTIA_FROM_FILE)

for i in range (1000):
    p.stepSimulation()
    time.sleep(1./FPS)

forkbotPos, forkbotOrn = p.getBasePositionAndOrientation(forkbotId)

print(forkbotPos[0], type(forkbotPos))
print(forkbotOrn)
p.disconnect()

