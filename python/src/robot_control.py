#!/usr/bin/env python3
import argparse
import sys
import numpy as np
import time
import csv
import contextlib
import collections

import registration as reg
import cpptendon as t
import gridgen
from measure_tip_poses import Ndi, RobotController


NdiPosition = collections.namedtuple('NdiPosition', ['ndi_x', 'ndi_y', 'ndi_z'])
NdiQuaternion = collections.namedtuple('NdiQuaternion',
        ['ndi_qw', 'ndi_qx', 'ndi_qy', 'ndi_qz'])

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    
    parser.add_argument('robot_toml')
    parser.add_argument('-r', '--read', default='sphere.csv',
                        help='read csv file (default is sphere.csv)')
    parser.add_argument('-o', '--output', default='sphere_task.csv',
                        help='output csv file (default is sphere_task.csv)')

    parser.add_argument('-t', '--table', default='lookup_table.csv',
                        help='lookup table csv file (default is lookup_table.csv)')
    parser.add_argument('-np', '--ndi_port', default="/dev/ttyUSB0",
                        help='ndi port (default is /dev/ttyUSB0)')
    parser.add_argument('-ap', '--arduino_port', default='/dev/ttyACM0',
                        help='arduino port (default is /dev/ttyACM0)')
    parser.add_argument('-nb', '--ndi_baud', default=921600,
                        help='ndi_baud (default is 921600)')
    parser.add_argument('-ab', '--arduino_baud', default=9600,
                        help='arduino_baud (default is 9600') 
    return parser

def from_txt_to_csv():
    with open('sphere.txt') as infile, open('sphere.csv', 'w') as outfile:
        for line in infile:
            outfile.write(" ".join(line.split()).replace(' ', ','))
            outfile.write(",")
            outfile.write("\n")

def read_csv(file_name):
    
    rows = []
    with open(file_name, 'r') as fin:
        csvreader = csv.reader(fin)
        for row in csvreader:
           rows.append(row)
    return rows

def write_csv(file_name, contents):
    fields = ["len_1", "len_2", "len_3", "len_4", "servo_1", "servo_2", "servo_3", "servo_4", "tau_1","tau_2","tau_3","tau_4","x", "y", "z", "x_d", "y_d", "z_d"]
    with open(file_name, 'w') as fout:
        writer = csv.writer(fout)
        writer.writerow(fields)
        writer.writerows(contents)

def pretty_sleep(secs):
    print('  - delay:     ', end='')
    remainder = np.mod(secs, 1)
    if remainder > 0:
        print(secs, end='  ')
        sys.stdout.flush()
        time.sleep(remainder)
    secs = int(secs)
    for i in range(secs):
        print(secs - i, end='  ')
        sys.stdout.flush()
        time.sleep(1)
    print()


@contextlib.contextmanager
def open_connector(cls, port, baud):
    connector = None
    try:
        connector = cls(port, baud)
        yield connector
    finally:
        if connector is not None:
            connector.close()

def open_ndi(port, baud):
    return open_connector(Ndi, port, baud)

def open_arduino(port, baud):
    return open_connector(RobotController, port, baud)

def local_controller(robot, controller, lengths, taus, tip, desired_tip, ndi, arduino, T):
    dt = 0.1 # seconds
    max_speed = 0.01 # 1 cm / sec

    tolerance = 0.01
    tip = np.array(tip, dtype=np.float64)
    desired_tip = np.array(desired_tip, dtype=np.float64)
    taus_old = taus
    _, lengths_tilde_old = robot.shape_and_lengths(taus_old)
    lengths_tilde_old = np.asarray(lengths_tilde_old)
    count = 1
    max_iter = 100
    while np.linalg.norm(tip - desired_tip) > tolerance:
        if count > max_iter :
            break
        start = time.time()
        v_times_dt = t.controller.clamped_v_times_dt(tip, desired_tip, max_speed * dt)
        print("v_times_dt:    ", v_times_dt)
        taus_new = controller.damped_resolved_rate_update(taus_old, v_times_dt)
        print("1")
        _, lengths_tilde_new = robot.shape_and_lengths(taus_new)
        print("2")
        lengths_tilde_new = np.asarray(lengths_tilde_new)
        print("3")
        d_lengths = lengths_tilde_new - lengths_tilde_old
        print("4")
        lengths = lengths + d_lengths
        print("5")
        servos = gridgen.lens_to_servos(lengths)
        print("6")
        arduino.move_robot(np.array(servos))
        print("7")
        tip, quat = ndi.measure()
        print("8")
        tip = reg.convert_to_robot_base(T, list(tip)) 
        tip = np.array(tip, dtype=np.float64)
        print()
        print("Current measured tip position vs Desired tip position\n")
        print('current: ' , tip)
        print("desired: ", desired_tip) 
        print("error: ", np.linalg.norm(tip - desired_tip)) 
        
        time_taken = time.time() - start
        sleep_time = dt - time_taken 
        #if sleep_time > 0:
        time.sleep(sleep_time)
        taus_old = taus_new
        lengths_tilde_old = lengths_tilde_new
        count += 1
    return servos, tip

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    T = reg.registration()    
    desired_tip = read_csv(args.read)
    #lookup = read_csv(args.table)[1:]
    with open(args.table, 'r') as fin:
        reader = csv.DictReader(fin)
        lookup = [
            {
                'lengths': np.array([float(row[f'len_{i+1}']) for i in range(4)]),
                'servos': np.array([float(row[f'servo_{i+1}']) for i in range(4)]),
                'taus': np.array([float(row[f'tau_{i+1}']) for i in range(4)]),
                'tip': np.array([float(row[c]) for c in ('x', 'y', 'z')]),
            } for row in reader
        ]
    # Populate nn 
    nn = t.motion_planning.NearestNeighborL2()
    for row in lookup:
        nn.add(row['tip'], row)

    with open_ndi(args.ndi_port, args.ndi_baud) as ndi, \
            open_arduino(args.arduino_port, args.arduino_baud) as arduino:

        # Home pose
        arduino.move_robot(np.array([2000]*4))
        pretty_sleep(5)
        robot = t.tendon.TendonRobot.from_toml(args.robot_toml)
        controller = t.controller.Controller(robot)
        result = []
        #for row in desired_tip:
        tip_d = np.array([float(x) for x in desired_tip[3][:-1]])
        key, nearest = nn.nearest(tip_d)

        #move the robot to the nearest point from the look-up table
        arduino.move_robot(nearest['servos'])
        pretty_sleep(6)

        tip,quat = ndi.measure()
        tip = reg.convert_to_robot_base(T, list(tip)) #tip - list
        configs, tip = local_controller(
                robot, controller,
                nearest['lengths'], nearest['taus'], tip, tip_d,
                ndi, arduino, T)
        print(tip, tip_d)

if __name__=='__main__':
    main(sys.argv[1:])


