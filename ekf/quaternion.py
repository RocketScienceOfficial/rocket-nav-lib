def quat_normalize(q):
    d = (q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2) ** 0.5

    return [
        q[0] / d,
        q[1] / d,
        q[2] / d,
        q[3] / d,
    ]


def quat_from_vecs(a, b):
    cross = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
    dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    a_mag = (a[0] ** 2 + a[1] ** 2 + a[2] ** 2) ** 0.5
    b_mag = (b[0] ** 2 + b[1] ** 2 + b[2] ** 2) ** 0.5

    q = [0, 0, 0, 0]

    q[0] = a_mag * b_mag + dot
    q[1] = cross[0]
    q[2] = cross[1]
    q[3] = cross[2]

    return quat_normalize(q)


def quat_rotate_vec(q, v):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    vx = v[0]
    vy = v[1]
    vz = v[2]

    return [
        +vx * (1 - 2 * (q2**2 + q3**2)) + vy * (2 * (q1 * q2 - q0 * q3)) + vz * (2 * (q1 * q3 + q0 * q2)),
        +vx * (2 * (q1 * q2 + q0 * q3)) + vy * (1 - 2 * (q1**2 + q3**2)) + vz * (2 * (q2 * q3 - q0 * q1)),
        +vx * (2 * (q1 * q3 - q0 * q2)) + vy * (2 * (q2 * q3 + q0 * q1)) + vz * (1 - 2 * (q1**2 + q2**2)),
    ]
