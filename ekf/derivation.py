from sympy import *
from . import code_gen


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


def _create_cov_matrix_entry(i, j):
    return Symbol("P[" + str(i) + "][" + str(j) + "]", real=True)


def _create_cov_matrix(n):
    return Matrix(n, n, _create_cov_matrix_entry)


def generate_observation_equations(P, H, R):
    K = P * H.T * (H * P * H.T + R).inv()

    return H, K


def generate_cov_prediction(P, F, Q):
    P_new = F * P * F.T + Q

    return P_new


def run_derivation(generate_eqs):
    print("Starting derivation...")

    dt, g = symbols("dt, g")

    print("Setting State Vector...")

    z = symbols("z")

    state_vector = Matrix(
        [
            z,
        ]
    )

    print("Computing Transition Function...")

    f = Matrix(
        [
            z,
        ]
    )

    print("Setting Variance Vector...")

    var_h = symbols("sigma_h")

    process_variance_vector = Matrix(
        [
            var_h,
        ]
    )

    print("Computing observation function...")

    h = Matrix(
        [
            z,
            z,
        ]
    )

    print("Computing Jacobians...")

    F = f.jacobian(state_vector)
    Q = F * Matrix.diag(*process_variance_vector) * F.T
    H = h.jacobian(state_vector)

    print("Computing Measurement Covariance...")

    var_gps, var_baro = symbols("R_GPS, R_BARO")

    meas_variance_vector = Matrix([var_gps, var_baro])

    R = Matrix.diag(*meas_variance_vector)

    if not generate_eqs:
        print("Lambdifing functions...")

        functions_handles = {}
        functions_handles["f"] = lambdify([*state_vector, g, dt], f)
        functions_handles["F"] = lambdify([*state_vector, g, dt], F)
        functions_handles["Q"] = lambdify([*state_vector, var_h, g, dt], Q)
        functions_handles["h"] = lambdify([*state_vector], h)
        functions_handles["H"] = lambdify([*state_vector], H)
        functions_handles["R"] = lambdify([var_gps, var_baro], R)

        print("Done!")

        return functions_handles
    else:
        print("Generating equations...")

        P = _create_cov_matrix(state_vector.shape[0])

        code_gen.write_cov_matrix("cov", generate_cov_prediction(P, F, Q))
        code_gen.write_obs_eqs("fusion", generate_observation_equations(P, H, R))

        print("Done!")


if __name__ == "__main__":
    run_derivation(True)
