from sympy import *


def _quat_to_rot(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    return Matrix(
        [
            [1 - 2 * (q2**2 + q3**2), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1**2 + q3**2), 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1**2 + q2**2)],
        ]
    )


def _quat_mult(p, q):
    r = Matrix(
        [
            p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
            p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
            p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
            p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0],
        ]
    )

    return r


def _print_matrix(m: Matrix):
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            print(m[i, j], end=", ")

        print("")


def run_derivation(generate_eqs):
    print("Starting derivation...")

    dt, g = symbols("dt, g")

    print("Setting State Vector...")

    q_w, q_x, q_y, q_z = symbols("q_w, q_x, q_y, q_z")
    q = Matrix([q_w, q_x, q_y, q_z])
    rot_to_earth = _quat_to_rot(q)
    rot_to_body = rot_to_earth.T

    vel_n, vel_e, vel_d = symbols("vel_n, vel_e, vel_d")
    vel = Matrix([vel_n, vel_e, vel_d])

    pos_n, pos_e, pos_d = symbols("pos_n, pos_e, pos_d")
    pos = Matrix([pos_n, pos_e, pos_d])

    mag_n, mag_e, mag_d = symbols("mag_n, mag_e, mag_d")
    mag = Matrix([mag_n, mag_e, mag_d])

    state_vector = Matrix(
        [
            q_w,
            q_x,
            q_y,
            q_z,
            vel_n,
            vel_e,
            vel_d,
            pos_n,
            pos_e,
            pos_d,
            mag_n,
            mag_e,
            mag_d,
        ]
    )

    print("Setting Control Vector...")

    gyro_x, gyro_y, gyro_z = symbols("gyro_x, gyro_y, gyro_z")
    gyro = Matrix([gyro_x, gyro_y, gyro_z])

    accel_x, accel_y, accel_z = symbols("accel_x, accel_y, accel_z")
    accel = Matrix([accel_x, accel_y, accel_z])

    control_vector = Matrix(
        [
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
        ]
    )

    print("Setting Variance Vector...")

    var_acc, var_gyr = symbols("sigma_acc, sigma_gyr")

    process_variance_vector = Matrix(
        [
            var_acc,
            var_acc,
            var_acc,
            var_gyr,
            var_gyr,
            var_gyr,
        ]
    )

    print("Computing Transition Function...")

    corrected_acc = rot_to_earth * accel + Matrix([0, 0, g])
    q_new = _quat_mult(q, Matrix([1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2]))
    vel_new = vel + corrected_acc * dt
    pos_new = pos + vel * dt
    mag_new = mag

    f = Matrix(
        [
            q_new,
            vel_new,
            pos_new,
            mag_new,
        ]
    )

    print("Computing observation function...")

    h = Matrix(
        [
            pos,
            pos_d,
            # rot_to_body * mag,
            mag,
        ]
    )

    print("Computing Jacobians...")

    F = f.jacobian(state_vector)
    G = f.jacobian(control_vector)
    Q = G * Matrix.diag(*process_variance_vector) * G.T
    H = h.jacobian(state_vector)

    print("Computing Measurement Covariance...")

    var_gps, var_baro, var_mag = symbols("R_GPS, R_BARO, R_MAG")

    meas_variance_vector = Matrix([var_gps, var_gps, var_gps, var_baro, var_mag, var_mag, var_mag])

    R = Matrix.diag(*meas_variance_vector)

    print("Lambdifing functions...")

    functions_handles = {}
    functions_handles["f"] = lambdify([*state_vector, *control_vector, g, dt], f)
    functions_handles["F"] = lambdify([*state_vector, *control_vector, g, dt], F)
    functions_handles["Q"] = lambdify([*state_vector, *control_vector, var_acc, var_gyr, g, dt], Q)
    functions_handles["h"] = lambdify([*state_vector], h)
    functions_handles["H"] = lambdify([*state_vector], H)
    functions_handles["R"] = lambdify([var_gps, var_baro, var_mag], R)

    print("Done!")

    return functions_handles


if __name__ == "__main__":
    run_derivation(True)
