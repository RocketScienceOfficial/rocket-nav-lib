import csv
import numpy as np
import matplotlib.pyplot as plt
from geo import geo
from . import ekf, derivation, voting, quaternion


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

        baro_height = geo.baro_formula(press) - base_params[0]
        pos = geo.geo_to_ned(lat, lon, alt, base_params[1], base_params[2], base_params[3])

        data.append([pos[0], pos[1], pos[2], baro_height, mag_x, mag_y, mag_z, acc1_x, acc1_y, acc1_z, acc2_x, acc2_y, acc2_z, gyro1_x, gyro1_y, gyro1_z, gyro2_x, gyro2_y, gyro2_z, state])


# ========== FILTER ==========


dt = 0.0025
g = -9.80665
start_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
start_covariance_value = 1
variance_mag = 0.9
variance_gps = 2.1
variance_baro_height = 0.9

start_state[0:4] = quaternion.quat_from_vecs([data[0][7], data[0][8], data[0][9]], [0, 0, abs(g)])

handles = derivation.run_derivation(False)
filt = ekf.ExtendedKalmanFilter(
    start_state,
    start_covariance_value,
    dt,
    g,
)

acc_voter = voting.SensorVoting([6, 32], [0.5, 0.5])
gyro_voter = voting.SensorVoting([500, 2000], [0.3, 0.3])

filter_data = []

for i in range(len(data)):
    [acc_x, acc_x_var] = acc_voter.vote([data[i][7], data[i][10]])
    [acc_y, acc_y_var] = acc_voter.vote([data[i][8], data[i][11]])
    [acc_z, acc_z_var] = acc_voter.vote([data[i][9], data[i][12]])
    [gyro_x, gyro_x_var] = gyro_voter.vote([data[i][13], data[i][16]])
    [gyro_y, gyro_y_var] = gyro_voter.vote([data[i][14], data[i][17]])
    [gyro_z, gyro_z_var] = gyro_voter.vote([data[i][15], data[i][18]])

    filt.predict([acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z], handles["f"], handles["F"], handles["Q"], [(acc_x_var + acc_y_var + acc_z_var) / 3, (gyro_x_var + gyro_y_var + gyro_z_var) / 3])
    filt.correct(data[i][0:7], handles["h"], handles["H"], handles["R"], [variance_gps, variance_baro_height, variance_mag])

    filter_data.append(filt.x[:, 0])


# ========== ANALYSIS ==========


figure, axis = plt.subplots(1, 3)

figure.set_figwidth(30)
figure.set_figheight(10)

t = np.arange(0, len(filter_data) * dt, dt)
axis[0].plot(t, list(map(lambda x: x[9], filter_data)))
axis[0].plot(t, list(map(lambda x: x[3], data)))

axis[1].plot(list(map(lambda x: x[8], filter_data)), list(map(lambda x: x[7], filter_data)))
axis[1].plot(list(map(lambda x: x[1], data)), list(map(lambda x: x[0], data)))
axis[1].axis("scaled")

# axis[2].plot(t, list(map(lambda x: (x[4] ** 2 + x[5] ** 2 + x[6] ** 2) ** 0.5, filter_data)))
tmp = []
for i in range(len(data)):
    tmp.append(quaternion.quat_rotate_vec(filter_data[i][0:4], [data[i][7], data[i][8], data[i][9]])[0])
    # tmp.append(data[i][9])
axis[2].plot(t, tmp)

plt.show()
