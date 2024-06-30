"""
Resources:
    - Docs:
        - PX4 Docs: https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html
        - PX4 Models PDF: https://github.com/PX4/PX4-ECL/blob/master/EKF/documentation/Process%20and%20Observation%20Models.pdf
        - AHRS: https://ahrs.readthedocs.io/en/latest/filters/ekf.html

Register:
    - Fusion of multiple irregular frames, with different sampling rates
        - Velocity has large oscillations
    - Observation: Kalman Gain & H Jacobian can be "stacked" with sub matrices for particular fusions
"""

import numpy as np
import csv
import matplotlib.pyplot as plt
from . import ekf, derivation

# TODO: Move to numpy

DT = 0.0025
meas = []
ctrls = []
n_m = 0

with open("./ekf/data/flightlog.csv") as file:
    reader = csv.reader(file, delimiter=",")

    base_baro_height = 0
    base_gps_height = 0

    for x in reader:
        press = float(x[19])
        baro_height = 44330.76923 * (1 - ((press / 101325) ** 0.1902632))
        alt = float(x[23])

        if base_baro_height == 0:
            base_baro_height = baro_height
        if base_gps_height == 0:
            base_gps_height = alt

        baro_height -= base_baro_height
        alt -= base_gps_height

        meas.append([baro_height, alt])
        ctrls.append([])
        n_m += 1


handles = derivation.run_derivation(False)
filt = ekf.ExtendedKalmanFilter(
    np.array([[0]]).T,
    np.eye(1) * 1000,
    n_m,
    np.array([0.5]),
)

for i in range(n_m):
    filt.predict(ctrls[i], handles["f"], handles["F"], handles["Q"])
    filt.correct(meas[i], handles["h"], handles["H"], handles["R"](0.5, 5))

t = np.arange(0, n_m * DT, DT)

# plt.plot(t, filt.X_est[0])
plt.plot(t, list(map(lambda x: x[1], meas)))
plt.ylim(-5, 50)
plt.show()
