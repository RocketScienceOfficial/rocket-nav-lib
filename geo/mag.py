'''
Resources:
    - https://github.com/PX4/PX4-ECL/blob/master/geo_lookup/fetch_noaa_table.py
    - https://www.ngdc.noaa.gov/geomag/CalcSurveyFin.shtml
'''

import math
import json
import os
import urllib.request

BASE_URL = "https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfgrid"
KEY = "gFE5W"

SAMPLING_RES = 10
SAMPLING_MIN_LAT = -90
SAMPLING_MAX_LAT = 90
SAMPLING_MIN_LON = -180
SAMPLING_MAX_LON = 180

LAT_DIM = int((SAMPLING_MAX_LAT - SAMPLING_MIN_LAT) / SAMPLING_RES) + 1
LON_DIM = int((SAMPLING_MAX_LON - SAMPLING_MIN_LON) / SAMPLING_RES) + 1


def __get_data(component, key):
    table = []
    index = 0

    for latitude in range(SAMPLING_MIN_LAT, SAMPLING_MAX_LAT + 1, SAMPLING_RES):
        params = urllib.parse.urlencode({'key': KEY, 'lat1': latitude, 'lat2': latitude, 'lon1': SAMPLING_MIN_LON, 'lon2': SAMPLING_MAX_LON,
                                        'latStepSize': 1, 'lonStepSize': SAMPLING_RES, 'magneticComponent': component, 'resultFormat': 'json'})

        f = urllib.request.urlopen(BASE_URL + "?%s" % params)
        data = json.loads(f.read())

        table.append([])

        for p in data['result']:
            table[index].append(p[key])

        index += 1

    return table


def __get_declination():
    return __get_data('d', 'declination')


def __get_inclination():
    return __get_data('i', 'inclination')


def __get_strength():
    return __get_data('f', 'totalintensity')


def __clamp(x, min, max):
    if x > max:
        return max
    elif x < min:
        return min
    else:
        return x


def get_value_from_table(lat, lon, table):
    lat = __clamp(lat, -90, 90 - SAMPLING_RES);
    lat = __clamp(lat, -180, 180 - SAMPLING_RES);

    min_lat = math.floor(lat / SAMPLING_RES) * SAMPLING_RES
    min_lon = math.floor(lon / SAMPLING_RES) * SAMPLING_RES

    min_lat_index = int((min_lat - SAMPLING_MIN_LAT) / SAMPLING_RES)
    min_lon_index = int((min_lon - SAMPLING_MIN_LON) / SAMPLING_RES)

    data_ne = table[min_lat_index + 1][min_lon_index + 1]
    data_se = table[min_lat_index][min_lon_index + 1]
    data_sw = table[min_lat_index][min_lon_index]
    data_nw = table[min_lat_index + 1][min_lon_index]

    data_min = (data_se - data_sw) / SAMPLING_RES * (lon - min_lon) + data_sw
    data_max = (data_ne - data_nw) / SAMPLING_RES * (lon - min_lon) + data_nw
    data = (data_max - data_min) / SAMPLING_RES * (lat - min_lat) + data_min

    return data


def __generate_table_code(table, name, unit):
    code = ""
    code += f"// Magnetic {name} in {unit}\n"
    code += f"static const float {str(name).upper()}_TABLE[LAT_DIM][LON_DIM] = {{\n"

    for i in range(len(table)):
        code += "\t{"

        for j in range(len(table[i])):
            code += ("%.5f" % table[i][j]) + f"f, "

        code += "},\n"

    code += "};\n"

    return code


def generate_code():
    path = os.path.dirname(os.path.abspath(__file__))
    file_dir = os.path.join(path, "generated")

    if not os.path.exists(file_dir):
        os.makedirs(file_dir)

    file_path = os.path.join(file_dir, 'mag_tables.h')

    with open(file_path, 'w') as file:
        file.write(f"#ifndef _MAG_TABLES_H\n#define _MAG_TABLES_H\n\n")
        file.write(f"#define SAMPLING_RES {SAMPLING_RES}\n")
        file.write(f"#define SAMPLING_MIN_LAT {SAMPLING_MIN_LAT}\n")
        file.write(f"#define SAMPLING_MAX_LAT {SAMPLING_MAX_LAT}\n")
        file.write(f"#define SAMPLING_MIN_LON {SAMPLING_MIN_LON}\n")
        file.write(f"#define SAMPLING_MAX_LON {SAMPLING_MAX_LON}\n")
        file.write(f"#define LAT_DIM {LAT_DIM}\n")
        file.write(f"#define LON_DIM {LON_DIM}\n")
        file.write("\n")
        file.write(f"{__generate_table_code(__get_declination(), 'declination', 'degrees')}\n")
        file.write(f"{__generate_table_code(__get_inclination(), 'inclination', 'degrees')}\n")
        file.write(f"{__generate_table_code(__get_strength(), 'strength', 'nT')}\n")
        file.write(f"#endif")


if __name__ == "__main__":
    generate_code()
