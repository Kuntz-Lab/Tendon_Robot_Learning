#!/usr/bin/env python3

import argparse
import csv
import datetime
import itertools
import sys

import numpy as np
from numpy import pi, sin, cos

from cpptendon.tendon import TendonRobot
from cpptendon.controller import levenberg_marquardt, Bounds

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Generate a grid of configurations, specifically for a robot with only
        tendons, three straight tendons, and one helical tendon.  The output
        file has the following columns:

        - i: config number
        - len_{1..n_tendons}: tendon length commands
        - servo_{1..n_tendons}: associated servo commands (from lengths)
        - tau_{1..n_tendons}: associated tensions (may be empty)
        - fktip_{x,y,z}: predicted tip positions from FK (may be empty)

        Will generate configurations on three criteria:

        (1) only pulling on helical (length-based, 1D),
        (2) only pulling on straight tendons (length-based, 2D), and
        (3) full search in tension space (tension-based, 4D).

        Discretization will occur equally for each.
        '''
    parser.add_argument('robot_toml')
    parser.add_argument('-N', '--number', type=int, default=10,
                        help='''
                            Number of subdivisions in each dimension
                            (default is 10)
                            ''')
    parser.add_argument('-o', '--output', default='grid-lengths.csv',
                        help='output csv file (default is grid-lengths.csv)')

    return parser

def verify_assumptions(robot):
    '''
    Checks that the robot

    - has 4 tendons
    - the first three are straight-routed
    - the straight-routed tendons have equal and constant distance from the
      backbone
    - the last one is helical
    - rotation disabled
    - retraction disabled

    Raises a ValueError if robot is not valid.
    '''
    # assert rotation disabled
    if robot.enable_rotation:
        raise ValueError('robot rotation is not disabled')

    # assert retraction disabled
    if robot.enable_retraction:
        raise ValueError('robot retraction is not disabled')

    # assert: robot has 4 tendons
    Nt = len(robot.tendons)
    if Nt != 4:
        raise ValueError(f'# tendons != 4 (is {Nt} instead)')

    straight_tendons = robot.tendons[:3]
    helical_tendon = robot.tendons[-1]

    # assert: first three tendons are straight
    for i, t in enumerate(straight_tendons):
        if not t.is_straight():
            raise ValueError(f'tendon #{i+1} is not straight')

    # assert: first three tendons have identical and constant distance from the
    # backbone
    Dval = straight_tendons[0].D[0]
    eps = 1e-5
    for i, t in enumerate(straight_tendons):
        if abs(t.D[0] - Dval) > eps:
            raise ValueError(
                    f'tendon #{i+1} is not the same distance as #1 from the '
                    f'backbone (t[{i}].D[0] != t[0].D[0] : {t.D[0]} != {Dval})')
        if any(abs(d) > eps for d in t.D[1:]):
            raise ValueError(f'tendon #{i+1} backbone distance is not constant'
                             f' (t[{i}].D: {t.D})')

    # assert last tendon is helical
    if not helical_tendon.is_helix():
        raise ValueError('tendon #4 is not helical')

def helical_len_range(robot, N):
    '''
    Goes through commanding the helical tendon by itself by length.

    Returns two values (lengths, None)
    - the first is the commanded lengths as a numpy array.
    - the second is the value None, which means we don't have a prediction of
      the tip position.
    '''
    Nt = len(robot.tendons)
    slack_lengths = np.array([t.min_length for t in robot.tendons])
    ht = robot.tendons[-1]
    len_range = np.linspace(max(ht.min_length, 0), ht.max_length, N)
    taus = np.array([0.] * robot.state_size())
    home_shape = robot.home_shape(taus)
    bounds = Bounds()
    bounds.lower = [0]
    bounds.upper = [ht.max_tension]
    taus[Nt-1] = ht.max_tension / 2
    print(taus)
    print(bounds.upper)
    print(bounds.lower)

    for h_length in len_range:
        if h_length <= 0:
            continue # skip
        lengths = slack_lengths.copy()
        lengths[-1] = h_length

        # approximate taus
        def f(x):
            nonlocal taus
            nonlocal Nt
            taus[Nt-1] = x
            _, solved_lengths = robot.shape_and_lengths(taus, home_shape)
            return solved_lengths[-1:]

        result = levenberg_marquardt(
                f=f,
                bounds=bounds,
                initial_state=np.array([taus[Nt-1]]),
                des=np.array([lengths[-1]]),
                max_iters=1000,
                stop_threshold_err=1e-10,
                )
        print()
        print('LevmarResult: (helical_length_range)')
        print('  state:               ', result.state)
        print('  fout:                ', result.fout)
        print('  err_init:            ', result.err_init)
        print('  err:                 ', result.err)
        print('  JT_err:              ', result.JT_err)
        print('  Dp:                  ', result.Dp)
        print('  mu_over_JTJ:         ', result.mu_over_JTJ)
        print('  iters:               ', result.iters)
        print('  term_condition:      ', result.term_condition)
        print('  num_fk_calls:        ', result.num_fk_calls)
        print('  num_jacobian_calls:  ', result.num_jacobian_calls)
        print('  num_linear_solves:   ', result.num_linear_solves)
        print('  term_reason:         ', result.term_reason)
        taus[Nt-1] = result.state

        assert abs(result.fout - h_length) < 1e-6, (result.fout, h_length)

        # calculate tip position
        tip = robot.shape(taus).p[-1]
        print('  tip position:        ', tip)
        print()

        yield (lengths, taus, tip)

def angle_fix(theta):
    '''fixes theta to be between 0 and 2*pi in angle space'''
    return np.mod(theta, 2*pi)

def straight_len_range(robot, N):
    '''
    Goes through commanding the straight tendons by themselves by length.

    Returns two values (lengths, predicted_tip)
    - lengths: the commanded lengths as a numpy array.
    - predicted_tip: the predict tip position for this command, based on the
      assumption of a constant-curvature circular arc.
    '''
    slack_lengths = np.array([t.min_length for t in robot.tendons])

    theta_range = np.linspace(0, 2*pi, N+1)[:-1]
    max_of_minlen = max(t.min_length for t in robot.tendons[:3])
    min_of_maxlen = min(t.max_length for t in robot.tendons[:3])
    arclen_diff_range = np.linspace(max(max_of_minlen, 0), min_of_maxlen, N)

    L = robot.specs.L
    r = robot.tendons[0].D[0]
    home_shape = robot.home_shape()
    all_bounds = Bounds.from_robot(robot)
    all_bounds.upper = [x*2 for x in all_bounds.upper]

    subsample = lambda arr, idxs: [arr[i] for i in idxs]

    for theta, arcdiff in itertools.product(theta_range, arclen_diff_range):
        if arcdiff <= 0:
            continue # skip
        lengths = slack_lengths.copy()

        # for arcdiff (of a tendon aligned with theta), and curving angle theta,
        # calculate the lengths for the tendons we pull on.
        idxs = []
        for i, t in enumerate(robot.tendons[:3]):
            dtheta = angle_fix(theta - t.C[0])
            if pi/2 < dtheta < 3*pi/2:
                continue # skip, this tendon needs slack
            lengths[i] = arcdiff * cos(dtheta)
            idxs.append(i)

        # approximate taus using levenberg marquardt
        def f(x):
            nonlocal idxs
            taus = np.array([0.0] * robot.state_size())
            for i, xval in zip(idxs, x):
                taus[i] = xval
            _, solved_lengths = robot.shape_and_lengths(taus, home_shape)
            return np.array(subsample(solved_lengths, idxs))

        bounds = Bounds()
        bounds.upper = subsample(all_bounds.upper, idxs)
        bounds.lower = subsample(all_bounds.lower, idxs)

        taus = np.array([0.0] * robot.state_size())
        sublengths = subsample(lengths, idxs)
        print(f'sublengths = {sublengths}')

        result = levenberg_marquardt(
                f=f,
                bounds=bounds,
                initial_state=[0.0] * len(idxs),
                des=sublengths,
                max_iters=1000,
                stop_threshold_err=1e-10,
                )

        for num, i in enumerate(idxs):
            taus[i] = result.state[num]

        print()
        print('LevmarResult: (helical_length_range)')
        print('  state:               ', result.state)
        print('  fout:                ', result.fout)
        print('  err_init:            ', result.err_init)
        print('  err:                 ', result.err)
        print('  JT_err:              ', result.JT_err)
        print('  Dp:                  ', result.Dp)
        print('  mu_over_JTJ:         ', result.mu_over_JTJ)
        print('  iters:               ', result.iters)
        print('  term_condition:      ', result.term_condition)
        print('  num_fk_calls:        ', result.num_fk_calls)
        print('  num_jacobian_calls:  ', result.num_jacobian_calls)
        print('  num_linear_solves:   ', result.num_linear_solves)
        print('  term_reason:         ', result.term_reason)
        print()

        assert np.linalg.norm(result.fout - sublengths) < 1e-6, \
                (result.fout, sublengths)

        # estimate tip position as constant curvature with no squishing
        phi = arcdiff / r # total angle of curvature of the backbone
        rho = L / phi
        w = rho * (1 - cos(phi))
        x = w * cos(theta)
        y = w * sin(theta)
        z = rho * sin(phi)
        tip = np.array([x, y, z])

        #debug = {
        #    'theta': theta,
        #    'arcdiff': arcdiff,
        #    'r': r,
        #    'L': L,
        #    'phi': phi,
        #    'rho': rho,
        #    'w': w,
        #    'tip': tip,
        #}
        #print('DEBUG:')
        #for k, v in debug.items():
        #    print(f'  {k}: {v}')

        yield (lengths, taus, np.array([x, y, z]))

def lens_to_servos(lengths):
    '''
    Converts lengths to servos (hard-coded conversions)

    This is pseudo-code of what Rahul does:

      Nt = len(robot.tendons)
      offsets = np.array([0.04]*Nt) - robot.home_shape().L_i # 4cm - L_i
      x = np.clip(robot.shape(taus).L_i + offsets, 0.0, 0.053)
      control = np.round(1000 * (1 + x / 0.053))

    our lengths are

      shape = home_shape.L_i - shape.L_i

    We have

      x = np.clip(shape.L_i - home_shape.L_i + 0.04, 0.0, 0.053)

    This is equal to

      x = np.clip(0.04 - lengths, 0.0, 0.053)

    Then control is

      control = np.round(1000 * (1 + x / 0.053))

    which maps x to an integer between 1000 and 2000.
    '''
    lengths = np.asarray(lengths)
    x = np.clip(0.04 - lengths, 0.0, 0.053)
    control = np.round(1000 * (1 + x / 0.053))
    return control.astype(int)

def tension_range(robot, N):
    '''
    Goes through commanding all tendons by tension.

    Does not include commanded shapes that are covered by helical_len_range()
    and straight_len_range().

    Does not pull on all three straight tendons at the same time.

    Returns commanded tensions as a tuple.
    '''
    zeros = np.array([0.0]*4) # home position
    yield zeros

    slack_lengths = np.array([t.min_length for t in robot.tendons])

    t_h = robot.tendons[-1] # helical tendon
    ranges = [np.linspace(0, t.max_tension, N) for t in robot.tendons]
    range_h = ranges[-1]
    for i, j in itertools.combinations(range(3), 2):
        t_i = robot.tendons[i] # ith straight tendon
        t_j = robot.tendons[j] # jth straight tendon
        range_i = ranges[i]
        range_j = ranges[j]
        for tau_i, tau_j, tau_h in itertools.product(range_i, range_j, range_h):
            # only keep tensions that pull on the helical and at least one other
            if tau_h == 0 or tau_i == tau_j == 0:
                continue # skip
            taus = zeros.copy()
            taus[i] = tau_i
            taus[j] = tau_j
            taus[-1] = tau_h
            yield taus

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    robot = TendonRobot.from_toml(args.robot_toml)
    N = args.number
    Nt = len(robot.tendons)

    verify_assumptions(robot)

    print(f'writing {args.output}')
    with open(args.output, 'w') as fout:
        writer = csv.writer(fout, lineterminator='\n')

        header = ['i']
        header.extend(f'len_{j+1}' for j in range(Nt))
        header.extend(f'servo_{j+1}' for j in range(Nt))
        header.extend(f'tau_{j+1}' for j in range(Nt))
        header.extend(['fktip_x', 'fktip_y', 'fktip_z'])
        writer.writerow(header)

        i = 1
        for lengths, taus, fktip in itertools.chain(
                helical_len_range(robot, N), straight_len_range(robot, N)):
            row = [i]
            row.extend(np.round(lengths, decimals=4))
            row.extend(lens_to_servos(lengths))

            if taus is None:
                row.extend([''] * Nt) # empty taus
            else:
                row.extend(np.round(taus, decimals=4))

            if fktip is None:
                row.extend([''] * 3)  # empty fktip
            else:
                row.extend(np.round(fktip, decimals=4))

            writer.writerow(row)
            i += 1

        for taus in tension_range(robot, N):
            home_shape = robot.home_shape()
            shape = robot.shape(taus)
            lengths = robot.calc_dl(home_shape.L_i, shape.L_i)
            fktip = shape.p[-1]

            row = [i]
            row.extend(np.round(lengths, decimals=4))
            row.extend(lens_to_servos(lengths))
            row.extend(np.round(taus, decimals=4))
            row.extend(np.round(fktip, decimals=4))
            writer.writerow(row)
            i += 1

    estimated_secs = (i-1) * 2 * 5 # go home between each.  each command = 5 sec
    estimated_duration = datetime.timedelta(seconds=estimated_secs)
    print(f'  {i-1} rows, at 10 s each = {estimated_duration}')

if __name__ == '__main__':
    main(sys.argv[1:])
