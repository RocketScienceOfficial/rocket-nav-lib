import csv
import numpy as np
import matplotlib.pyplot as plt
from geo import geo
from . import ekf, derivation, voting, quaternion

# ========== CONSTANTS ==========


g = -9.80665
dt = 0.0025


# ========== DATA ==========


data = []

with open("./ekf/data/flightlog.csv") as file:
    reader = csv.reader(file, delimiter=",")

    base_params = []

    for x in reader:
        acc1_x = float(x[1])
        acc1_y = float(x[2])
        acc1_z = float(x[3])
        acc2_x = float(x[5])
        acc2_y = -float(x[4])
        acc2_z = float(x[6])
        acc3_x = -float(x[8])
        acc3_y = float(x[7])
        acc3_z = float(x[9]) - g
        gyro1_x = float(x[10])
        gyro1_y = float(x[11])
        gyro1_z = float(x[12])
        gyro2_x = float(x[14])
        gyro2_y = -float(x[13])
        gyro2_z = float(x[12])
        mag_x = float(x[16])
        mag_y = float(x[17])
        mag_z = float(x[18])
        press = float(x[19])
        lat = float(x[21])
        lon = float(x[22])
        alt = float(x[23])
        state = float(x[24])

        if len(base_params) == 0:
            base_params = [geo.baro_formula(press), lat, lon, alt]

        currentHeight = geo.baro_formula(press) - base_params[0]

        pos = geo.geo_to_ned(lat, lon, alt, base_params[1], base_params[2], base_params[3])

        data.append(
            [
                pos[0],
                pos[1],
                pos[2],
                currentHeight,
                mag_x,
                mag_y,
                mag_z,
                acc1_x,
                acc1_y,
                acc1_z,
                acc2_x,
                acc2_y,
                acc2_z,
                acc3_x,
                acc3_y,
                acc3_z,
                gyro1_x,
                gyro1_y,
                gyro1_z,
                gyro2_x,
                gyro2_y,
                gyro2_z,
                state,
            ]
        )


# ========== FILTER ==========


start_state = [0]
start_covariance_value = 1
variance_acc_1 = 0.5
variance_acc_2 = 0.5
variance_acc_3 = 2
variance_gyro_1 = 0.3
variance_gyro_2 = 0.3
variance_mag = 0.9
variance_gps = 1.6
variance_baro_height = 0.9

handles = derivation.run_derivation(False)
filt = ekf.ExtendedKalmanFilter(
    start_state,
    start_covariance_value,
    dt,
    g,
)

acc_voter = voting.SensorVoting([6, 32, 100], [variance_acc_1, variance_acc_2, variance_acc_3])
gyro_voter = voting.SensorVoting([500, 2000], [variance_gyro_1, variance_gyro_2])

filter_data = []

for i in range(len(data)):
    filt.predict([], handles["f"], handles["F"], handles["Q"], [1])
    filt.correct([data[i][2], data[i][3]], handles["h"], handles["H"], handles["R"], [variance_gps, variance_baro_height])

    filter_data.append(filt.x[:, 0])


# ========== ANALYSIS ==========


figure, axis = plt.subplots(1, 3)

figure.set_figwidth(30)
figure.set_figheight(10)

t = np.arange(0, len(filter_data) * dt, dt)

axis[0].plot(t, list(map(lambda x: x[0], filter_data)))
axis[1].plot(t, list(map(lambda x: x[3], data)))
axis[2].plot(t, list(map(lambda x: x[2], data)))

plt.show()
