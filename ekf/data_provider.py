import numpy as np
import csv
import os
from geo import geo


def __parse_csv(filename, delim=";"):
    path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(path, "data", filename)
    is_header = True
    data = []
    df_len = -1

    with open(file_path) as file:
        reader = csv.reader(file, delimiter=delim)

        for row in reader:
            if not is_header:
                row.pop()

                if len(row) == df_len:
                    for i in range(len(row)):
                        try:
                            row[i] = float(row[i])
                        except:
                            row[i] = 0

                    data.append(row)
            else:
                is_header = False
                df_len = len(row)

    return data


def get_SAC_data():
    data = __parse_csv("data_0.txt")
    data_len = len(data)

    a_x = np.array(data)[:, 16]
    a_y = np.array(data)[:, 17]
    a_z = np.array(data)[:, 18]
    w_x = np.array(data)[:, 19]
    w_y = np.array(data)[:, 20]
    w_z = np.array(data)[:, 21]
    gps_x = np.array(data)[:, 12]
    gps_y = np.array(data)[:, 13]
    gps_z = np.array(data)[:, 14]
    baro_height = np.array(data)[:, 27]
    mag_x = np.array(data)[:, 22] * 100000
    mag_y = np.array(data)[:, 23] * 100000
    mag_z = np.array(data)[:, 24] * 100000

    gps_flag = np.tile(1, data_len)

    for i in range(len(gps_x)):
        if gps_x[i] == 0:
            gps_flag[i] = 0
        else:
            [gps_x[i], gps_y[i], gps_z[i]] = geo.geo_to_ned(
                32.951805, -106.915802, 137.073196, gps_x[i], gps_y[i], gps_z[i]
            )

        gps_flag[i] = False

    true_flag = np.tile(True, data_len)

    measurements = np.vstack((gps_x, gps_y, gps_z, baro_height, mag_x, mag_y, mag_z))

    measurements_flags = np.vstack(
        (gps_flag, gps_flag, gps_flag, true_flag, true_flag, true_flag, true_flag)
    )

    controls = np.vstack((a_x, a_y, a_z, w_x, w_y, w_z))

    return measurements, measurements_flags, controls
