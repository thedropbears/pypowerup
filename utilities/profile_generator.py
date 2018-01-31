import math


def sign(x):
    if x >= 0:
        return 1
    else:
        return -1


def cubic_generator(keypoints):
    """Return a function that returns the distance and speed at a given time.
    Args:
        keypoints: a list of (time, distance, speed) tuples.
    """

    # Approach taken from Introduction to Robotics: Mechanics, Planning and
    # Control. Craig, 2005.
    coefficients = []
    for idx in range(len(keypoints)-1):
        start = keypoints[idx]
        finish = keypoints[idx+1]
        tf = finish[0]-start[0]
        d0, v0 = start[1:3]
        df, vf = finish[1:3]
        coefficients.append((start[0], finish[0],
                            (d0, v0,
                             3/tf**2*(df-d0)-2*v0/tf-vf/tf,
                             -2/tf**3*(df-d0)+(vf+v0)/tf**2)))

    def trajectory(t):
        for coeff in coefficients:
            if coeff[0] <= t <= coeff[1]:
                t_rel = t-coeff[0]
                c = coeff[2]
                d = c[0]+c[1]*t_rel+c[2]*t_rel**2+c[3]*t_rel**3
                v = c[1]+2*c[2]*t_rel+3*c[3]*t_rel**2
                a = 2*c[2]+6*c[3]*t_rel
                return (d, v, a)
    return trajectory


def generate_interpolation_trajectory(x_start, x_final, traj_to_match):
    """Generate a 1d interpolation profile, where the velocity is constant
    over the duration of the trajectory.
    Args:
        x_start: Start position of the trajectory.
        x_final: End position of the trajectory.
        traj_to_match: Motion profile to interpolate along the length of.
    Returns:
        A list of (pos, vel acc) tuples.
    """
    x = x_final - x_start

    vel = 50*x/len(traj_to_match)

    num_segments = len(traj_to_match)
    segments = [(x_start+x*i/num_segments, vel, 0) for i in range(0, num_segments)]
    return segments


def generate_interpolation_function(x_start, x_final, interpolation_distance):
    """Generate a function to interpolate in 1 dimension, where the velocity
    is constant over the duration of the trajectory.

    Args:
        x_start: starting position on the axis we are controlling
        x_final: finishing position on the axis we are controlling
        interpolation_distance: distance we are travelling along the axis we
            are *not* controlling. Function returned will match position and
            velocity of the axis we are controlling to distance and speed on
            the axis we are not controlling.
    """
    # distance to cover on the axis we are controlling
    x = x_final - x_start

    avg_vel = x / interpolation_distance  # units / m

    def get_profile_point(distance, speed):
        distance_proportion = distance / interpolation_distance
        # convert from units/m to units / sec
        vel = avg_vel * speed
        return (x_start+distance_proportion*x, vel, 0)
    return get_profile_point


def generate_trapezoidal_function(
        x_start, v_start, x_final, v_final, v_max, a_pos, a_neg):
    direction = sign(x_final-x_start)

    # area under the velocity-time trapezoid
    x = x_final - x_start

    v_max = abs(v_max)*direction
    a_pos = abs(a_pos)*direction
    a_neg = -abs(a_neg)*direction

    if x == 0:
        return [(x_start, v_start, 0.0)]

    # find the max reachable velocity if we spend all our time accelerating
    # and decelerating. Used as max velocity in cases where we don't hit the
    # robot's top speed
    triangular_max = direction * math.sqrt(
            (2*x*a_pos*a_neg+a_neg*v_start**2-a_pos*v_final**2)/(a_neg-a_pos))
    v_max = direction * min(abs(v_max), abs(triangular_max))

    # time (since the start of the trajectory) that we hit v_max
    t_cruise = (v_max - v_start)/a_pos
    # distance we have travelled once we hit v_max
    x_cruise = t_cruise*(v_start + v_max)/2
    # time it takes to slow down to v_final
    t_slow = (v_final - v_max)/a_neg
    # time at which we start decelerating
    t_decel = (x-t_cruise*(v_start + v_max)/2
               - t_slow*(v_final + v_max)/2)/v_max + t_cruise
    # how long we are cruising at v_max for (flat part of the trapezoid)
    t_constant = t_decel - t_cruise
    # how far we have travelled since the start when we start decelerating
    x_decel = x_cruise + v_max*t_constant
    # time at which we finish the trajetory

    decel_dist = x_final - x_decel
    decel_mag = v_max - v_final

    def get_speed(distance):
        if distance < x_cruise:
            accel_proportion = (distance / x_cruise)
            target_v = (v_max * accel_proportion
                        # factor that decays as we accelerate. Used to jump start
                        # acceleration from 0 speed.
                        + (1-accel_proportion) * direction * 0.4)
            # print("Accelerating. accel_proportion %s, target_v %s" % (accel_proportion, target_v))
            return target_v
        elif distance < x_decel:
            target_v = v_max
            # print("Cruising at %s" % target_v)
            return target_v
        elif distance < x_final:
            if decel_dist == 0:
                print("Warning: decel_dist is 0. Returning max_v")
                return v_max
            decel_proportion = 1 - ((x_final - distance) / decel_dist)
            target_v = (v_max
                        - decel_mag * decel_proportion
                        + (1 - decel_proportion) * -direction * 0.2)
            # print("Decelerating. decel_proportion %s, target_v %s, final_v %s" % (decel_proportion, target_v, v_final))
            return target_v
        else:
            print("WARNING: calling speed profile function when after final distance")
            return v_final

    return get_speed


def generate_trapezoidal_trajectory(
        x_start, v_start, x_final, v_final, v_max, a_pos, a_neg, frequency):
    """Generate a 1d trapezoidal profile.

    Returns:
        A list of (pos, vel acc) tuples.
    """
    direction = sign(x_final-x_start)

    # area under the velocity-time trapezoid
    x = x_final - x_start

    v_max = abs(v_max)*direction
    a_pos = abs(a_pos)*direction
    a_neg = -abs(a_neg)*direction

    if x == 0:
        return [(x_start, v_start, 0.0)]

    # find the max reachable velocity if we spend all our time accelerating
    # and decelerating. Used as max velocity in cases where we don't hit the
    # robot's top speed
    triangular_max = direction * math.sqrt(
            (2*x*a_pos*a_neg+a_neg*v_start**2-a_pos*v_final**2)/(a_neg-a_pos))
    v_max = direction * min(abs(v_max), abs(triangular_max))

    # time (since the start of the trajectory) that we hit v_max
    t_cruise = (v_max - v_start)/a_pos
    # distance we have travelled once we hit v_max
    x_cruise = t_cruise*(v_start + v_max)/2
    # time it takes to slow down to v_final
    t_slow = (v_final - v_max)/a_neg
    # time at which we start decelerating
    t_decel = (x-t_cruise*(v_start + v_max)/2
               - t_slow*(v_final + v_max)/2)/v_max + t_cruise
    # how long we are cruising at v_max for (flat part of the trapezoid)
    t_constant = t_decel - t_cruise
    # how far we have travelled since the start when we start decelerating
    x_decel = x_cruise + v_max*t_constant
    # time at which we finish the trajetory
    t_f = t_decel + t_slow

    # interpolate the first (acceleration) portion of the path
    # number of discrete segments we pass through
    num_segments = int(t_cruise * frequency)
    segments = []
    if num_segments > 0:
        for i in range(0, num_segments+1):
            # velocity in the current timestep
            v = (v_max-v_start)*i/num_segments+v_start
            segments.append((
                    x_start+((v+v_start)/2)*t_cruise*i/num_segments,
                    v, a_pos))

    # interpolate along the cruise section of the path
    # do it as a list comprehension so that it runs faster
    num_segments = int(t_decel*frequency - num_segments)
    segments += [(
        (x_start+x_cruise + v_max * (t_decel-t_cruise) * i / num_segments),
        v_max, 0) for i in range(1, num_segments+1)]

    # interpolate along the deceleration portion of the path
    num_segments = int((t_f-t_decel)*frequency)
    for i in range(1, num_segments+1):
        v = v_max - (v_max-v_final) * i/num_segments
        segments.append((
            x_start + x_decel + (v+v_max)/2 * (t_f-t_decel) * i/num_segments,
            v, a_neg))

    return segments
