#!/usr/bin/env python3
'''
Measure physical robot tip positions after commanding to specific positions.
'''

import abc
import argparse
import collections
import contextlib
import csv
import itertools
import os
import sys
import time

import click  # command-line parser
import dotenv # for loading env vars from a .* file
import numpy as np
import serial # serial port

import cpptendon

# load env vars from .measure_tip_poses file
# performed here to load env vars before click's decorators
__config = dotenv.find_dotenv(filename='.mtp', usecwd=True)
if __config:
    dotenv.load_dotenv(dotenv_path=__config)

@click.command(
    context_settings={
        'auto_envvar_prefix': 'MTP',
        'help_option_names': ['-h', '--help'],
    },
)
@click.argument('cmd_csv', envvar='MTP_CMD_CSV',
                type=click.Path(exists=True, dir_okay=False))
@click.argument('arduino_port', envvar='MTP_ARDUINO_PORT',
                type=click.Path(exists=True, dir_okay=False))
@click.argument('ndi_port', envvar='MTP_NDI_PORT',
                type=click.Path(exists=True, dir_okay=False))
@click.option(
    '-o', '--output', default='ndi-tip-poses.csv',
    type=click.Path(dir_okay=False, writable=True),
    help='''
        Output file with measurements.  Will continue previous run if already
        exists.''',
)
# TODO: separate delay to commanded position and auto-homing between
@click.option('-d', '--delay', type=float, default=5.0,
              help='delay between movements')
@click.option('--arduino-baud', type=int, default=9600,
              help='Baud rate for the Arduino serial connection.')
# TODO: bring in options from ndi-terminal app
@click.option('--ndi-baud', type=int, default=921600,
              help='Baud rate for the NDI serial connection.')
def measure_tip_poses(
        cmd_csv,
        arduino_port,
        ndi_port,
        output,
        delay,
        arduino_baud,
        ndi_baud):
    '''
    Moves the physical robot from specific commands and records tip positions
    from the NDI tracker into an output CSV file.  The input CSV file is
    expected to have columns 'i', 'servo_{j}' for j in {1, ..., # tendons}.

    The output CSV file will add columns 'step', 'type', and
    'ndi_{x,y,z,qx,qy,qz,qw}'.  The ndi columns are the x, y, z positions of
    the tracker (which should be glued to the robot tip), and qx, qy, qz, and
    qw which are the quaternion components of the tracker orientation.

    In-between each given command, this script commands the robot to the home
    position by fully slacking all of the tendons.

    Each visited position is given a static amount of time (with --delay) to
    reach the commanded position and allow the robot to settle (quasi-static
    control).

    The 'step' column is an incremental number starting from 1 to each visited
    positions, including the intermediary home positions.

    The 'type' column will have the value 'home' for intermediate home visits,
    and value 'command' for a commanded position from the input CSV file.

    If the output file exists, then this script will attempt to continue where
    it left off.

    IMPORTANT: if the robot is misbehaving and you want to abort, use CTRL-C to
    stop.  This script will flush the output of the current row to the CSV
    file, immediately cancel the current robot position, and slacken all of the
    tendons before exiting.

    == ENVIRONMENT VARIABLES ==
    You may use environment variables for each of the options, prefixed with
    'MTP_' (which stands for Measured Tip Position), such as MTP_ARDUINO_PORT,
    MTP_ARDUINO_BAUD, MTP_NDI_PORT, MTP_NDI_BAUD, MTP_CMD_CSV, MTP_DELAY, etc.

    These environment variables may alternatively be placed in a '.mtp' file
    in the current directory as KEY=VALUE pairs, one on each line.
    '''
    with open(cmd_csv, 'r') as fin:
        reader = csv.DictReader(fin)
        header = reader.fieldnames
        commands = list(reader)
        assert all(x in header for x in
                   ('i', 'servo_1', 'servo_2', 'servo_3', 'servo_4'))

    if 'type' not in header:
        header.insert(0, 'type')
    if 'step' not in header:
        header.insert(0, 'step')
    for suffix in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'):
        col = f'ndi_{suffix}'
        if col not in header:
            header.append(col)

    # load previous progress
    completed_commands = None
    if os.path.exists(output):
        with open(output, 'r') as fin:
            out_reader = csv.DictReader(fin)
            completed_commands = list(out_reader)
            if completed_commands and completed_commands[-1]['type'] == 'home':
                # remove this last one to go to home again
                completed_commands.pop()
            if completed_commands:
                row = completed_commands[-1]
                i = int(row['i']) + 1
                step = int(row['step']) + 1
                print(f'Starting where you left off on command {i} (step {step})')

    # start collecting data and write to the output incrementally
    with open_ndi(ndi_port, ndi_baud) as ndi, \
         open_arduino(arduino_port, arduino_baud) as controller, \
         open(output, 'w') as fout:

        writer = csv.DictWriter(fout, fieldnames=header, lineterminator='\n',
                                extrasaction='ignore')
        writer.writeheader()

        # rewrite the current completed commands
        stepnum = 1
        if completed_commands:
            writer.writerows(completed_commands)
            stepnum = int(completed_commands[-1]['step']) + 1

        # run remaining commands
        steps = gen_steps(commands)
        steps = itertools.dropwhile(lambda x: x['step'] < stepnum, steps)
        for step in steps:
            servo_cmd = step['servo_cmd']
            print()
            print('Step', step['step'])
            print('  - type:     ', step['type'])
            print('  - i:        ', step['i'])
            print('  - cmd:      ', servo_cmd)
            controller.move_robot(servo_cmd)
            print(type(servo_cmd))
            for line in controller.readlines():
                print('    ARDUINO:  ', line)

            pretty_sleep(delay)
            pos, quat = ndi.measure()
            print(f'  - pos:      ', list(pos))
            print(f'  - quat:     ', list(quat))
            step.update(pos._asdict())
            step.update(quat._asdict())

            writer.writerow(step)

    print('DONE')

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

def gen_steps(commands):
    '''
    Generator for steps from the list of commands.

    Each given command is a dictionary containing 'i', and 'servo_{1,2,3,4}'.

    Populates the following fields: 'step', 'i', 'type', 'servo_{1,2,3,4}', and
    'servo_cmd'.  The 'servo_cmd' field contains a numpy array for convenience.
    '''
    num = 1
    servo_max = 2000
    servo_labels = [f'servo_{i+1}' for i in range(4)]
    servo_home_cmd = np.array([servo_max]*4)
    for cmd in commands:
        # auto-home step
        step = {
            'step': num,
            'i': int(cmd['i']),
            'type': 'home',
            'servo_cmd': servo_home_cmd,
        }
        step.update(zip(servo_labels, servo_home_cmd))
        yield step
        num += 1

        # command step
        step = dict(cmd)
        servo_cmd = np.array([int(cmd[label]) for label in servo_labels])
        step.update({
            'step': num,
            'type': 'command',
            'servo_cmd': servo_cmd
        })
        step.update(zip(servo_labels, servo_cmd))
        yield step
        num += 1

class DummySerial:
    'Dummy class to fake Ndi and RobotController'
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
    def measure(self):
        return (NdiPosition(1, 2, 3), NdiQuaternion(4, 5, 6, 7))
    def move_robot(self, raw_controls): pass
    def close(self): pass

class Ndi:
    def __init__(self, port, baud=921600):
        self._manager = cpptendon.ndi.NdiManager()
        self._manager.init(port, baud)
        self._manager.start_tracking()

    def measure(self):
        reply = self._manager.send_raw(b'bx')
        message = self._manager.parse_bx(reply)
        first_tool = message[0]
        p = message[0].position
        q = message[0].orientation
        pos = NdiPosition(p[0], p[1], p[2])
        quat = NdiQuaternion(q[0], q[1], q[2], q[3])
        return pos, quat

    def close(self):
        self._manager.close()

class RobotController:
    def __init__(self, port, baud=9600):
        self._connection = serial.Serial(port, baud, timeout=0.05)
        self._received = b''

    def move_robot(self, raw_controls):
        msg = '<' + ','.join(str(val) for val in raw_controls) + '>\n'
        print(f'    robot: sending {repr(msg)}')
        self._connection.write(msg.encode())

    def readlines(self):
        self._received += self._connection.read(100)
        lines = self._received.splitlines()
        if lines:
            self._received = lines[-1]
            return lines[:-1]
        else:
            self._received = b''
            return []

    def close(self):
        self._connection.close()

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
    #return open_connector(DummySerial, port, baud)

def open_arduino(port, baud):
    return open_connector(RobotController, port, baud)
    #return open_connector(DummySerial, port, baud)

NdiPosition = collections.namedtuple('NdiPosition', ['ndi_x', 'ndi_y', 'ndi_z'])
NdiQuaternion = collections.namedtuple('NdiQuaternion',
        ['ndi_qw', 'ndi_qx', 'ndi_qy', 'ndi_qz'])

def main(arguments):
    measure_tip_poses(args=arguments, standalone_mode=False)

if __name__ == '__main__':
    measure_tip_poses()
