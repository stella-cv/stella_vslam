import json
import numpy as np
from scipy.spatial.transform import Rotation as R
# from MH_04_difficult/cam0/sensor.yaml
tf = np.matrix([[0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975],
                [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768],
                [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949],
                [0.0, 0.0, 0.0, 1.0]])
r = R.from_matrix(tf[:3, :3])
q = r.as_quat()
tf_dict = {"x": tf[0, 3], "y": tf[1, 3], "z": tf[2, 3], "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3]}
print(json.dumps(tf_dict))
