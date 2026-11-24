"""Situation recognition (판단): what lies on the line ahead of the robot.

features.py  runtime feature vector from the node's own line map, traced path and IMU pitch
model.py     the small trained classifier (config/situation_model.yaml) -> class probabilities

Only robot data from this run is used (camera line map, odometry, IMU). Labels and training
live outside the package in experiments/situation/ (they use the simulator ground truth).
"""

# Order is the class index of the trained model. Keep in sync with config/situation_model.yaml.
SITUATIONS = (
    'STRAIGHT',      # nearly straight line ahead
    'ARC',           # smooth curve (radius >= ~0.12 m)
    'RIGHT_ANGLE',   # square corner or very tight arc
    'CROSSING',      # line crossing the route (cross bar, big loop, connected crossing): go straight
    'LOOP_CORNER',   # small circle at a corner (or teardrop loop): straight through, tight 270 deg turn
    'STUB',          # one-sided branch off the route (mid-path T): not the finish
    'GAP',           # break in the painted line
    'HILL',          # on or right before a ramp
    'FINISH_T',      # the finish T bar ahead: stop there
    'START_T',       # the start T bar ahead (driving back towards the start): not the finish
    'OFF',           # robot is not on the route (lost / wandering)
)
INDEX = {name: i for i, name in enumerate(SITUATIONS)}

__all__ = ['SITUATIONS', 'INDEX']
