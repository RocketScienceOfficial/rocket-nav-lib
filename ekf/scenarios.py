import numpy as np
import csv
import os


def __add_noise(X, sigma):
    #np.random.seed(1)
    noise = np.random.normal(0, sigma**0.5, X.shape[0])

    return X + noise


def __writeCSV(name, data):
    path = os.path.dirname(os.path.abspath(__file__))
    file_dir = os.path.join(path, "generated")

    if not os.path.exists(file_dir):
        os.makedirs(file_dir)

    file_path = os.path.join(file_dir, f"scenarios__{name}.csv")

    with open(file_path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)

        writer.writerow(
            [
                "acc_x",
                "acc_y",
                "acc_z",
                "gyro_x",
                "gyro_y",
                "gyro_z",
                "lat",
                "lon",
                "mag_x",
                "mag_y",
                "mag_z",
                "temp",
                "press",
            ]
        )

        for i in range(len(data[0][0])):
            writer.writerow(
                [
                    data[2][0][i],
                    data[2][1][i],
                    data[2][2][i],
                    data[2][3][i],
                    data[2][4][i],
                    data[2][5][i],
                    data[0][0][i],
                    data[0][1][i],
                    data[0][4][i],
                    data[0][5][i],
                    data[0][6][i],
                    0,
                    0,
                ]
            )


def generate_scenario_free_fall_parachute(n, dt, g, start_height, variances):
    N = n
    parachute_factor = 0.5
    mag_strength = 0.5

    t = np.arange(0, dt * N, dt)

    a_x = np.tile(0, N)
    a_y = np.tile(0, N)
    a_z = np.tile(g * parachute_factor, N)
    w_x = np.tile(0, N)
    w_y = np.tile(0, N)
    w_z = np.tile(0, N)
    gps_x = np.tile(0, N)
    gps_y = np.tile(0, N)
    gps_z = start_height + 0.5 * a_z * t**2
    baro_height = start_height + 0.5 * a_z * t**2
    mag_x = np.tile(1, N) * mag_strength
    mag_y = np.tile(0, N) * mag_strength
    mag_z = np.tile(0, N) * mag_strength

    a_z *= -1

    noisy_a_x = __add_noise(a_x, variances["a_x"])
    noisy_a_y = __add_noise(a_y, variances["a_y"])
    noisy_a_z = __add_noise(a_z, variances["a_z"])
    noisy_w_x = __add_noise(w_x, variances["w_x"])
    noisy_w_y = __add_noise(w_y, variances["w_y"])
    noisy_w_z = __add_noise(w_z, variances["w_z"])
    noisy_gps_x = __add_noise(gps_x, variances["gps"])
    noisy_gps_y = __add_noise(gps_y, variances["gps"])
    noisy_gps_z = __add_noise(gps_z, variances["gps"])
    noisy_baro_height = __add_noise(baro_height, variances["baro"])
    noisy_mag_x = __add_noise(mag_x, variances["mag"])
    noisy_mag_y = __add_noise(mag_y, variances["mag"])
    noisy_mag_z = __add_noise(mag_z, variances["mag"])

    true_measurements = np.vstack(
        (gps_x, gps_y, gps_z, baro_height, mag_x, mag_y, mag_z)
    )

    noisy_measurements = np.vstack(
        (
            noisy_gps_x,
            noisy_gps_y,
            noisy_gps_z,
            noisy_baro_height,
            noisy_mag_x,
            noisy_mag_y,
            noisy_mag_z,
        )
    )

    true_controls = np.vstack((a_x, a_y, a_z, w_x, w_y, w_z))

    noisy_controls = np.vstack(
        (noisy_a_x, noisy_a_y, noisy_a_z, noisy_w_x, noisy_w_y, noisy_w_z)
    )

    false_flag = np.tile(False, N)
    true_flag = np.tile(True, N)

    measurements_flags = np.vstack(
        (true_flag, true_flag, true_flag, true_flag, true_flag, true_flag, true_flag)
    )

    return noisy_measurements, measurements_flags, noisy_controls


if __name__ == "__main__":
    __writeCSV(
        "free_fall_parachute",
        generate_scenario_free_fall_parachute(
            1000,
            0.01,
            -9.81,
            1000,
            {
                "a_x": 0.1,
                "a_y": 0.1,
                "a_z": 0.1,
                "w_x": 0.1,
                "w_y": 0.1,
                "w_z": 0.1,
                "gps": 1,
                "baro": 0.5,
                "mag": 0.3,
            },
        ),
    )
