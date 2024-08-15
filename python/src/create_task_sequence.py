#!/usr/bin/env python3
'''
Creates a sequence of tasks by daisy-chaining start and goal configurations
from a csv file.  Will create a separate problem toml file for each task and
number them.
'''

import argparse
import copy
import csv
import os
import subprocess as subp
import sys
import toml

def populate_parser(parser=None):
    if not parser:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Creates a sequence of tasks by daisy-chaining start and goal
        configurations from a csv file.  Will create a separate problem toml
        file for each task and number them.
        '''
    parser.add_argument('problem',
                        help='''
                            Problem toml file.  This will be the basis for the
                            created task toml files.
                            ''')
    parser.add_argument('configs_csv',
                        help='''
                            CSV file containing configurations in order of
                            visiting.  The first configuration will be the
                            first start position, the second will be the first
                            goal position and the second start position, etc.
                            The expected columns are "tau_i" for i in range(1,
                            num_tendons + 1), "theta" (if
                            robot.enable_rotation) and "s_start" (if
                            robot.enable_retraction).
                            ''')
    parser.add_argument('-C', '--directory', default='tasks',
                        help='''
                            Output to this directory for the created tasks.
                            The files will start with the name task-0001.toml
                            and numerically increase from there.  Files will be
                            overwrittin if they exist in this output directory.
                            (default is './tasks')
                            ''')
    return parser

def read_config(row):
    'Convert a single CSV dict into a config'
    config = {'tau': [], 'theta': 0.0, 's_start': 0.0};
    for i in range(1, len(row)+1):
        name = 'tau_' + str(i)
        if name in row:
            config['tau'].append(float(row[name]))
        else:
            break

    if 'theta' in row:
        config['theta'] = float(row['theta'])

    if 's_start' in row:
        config['s_start'] = float(row['s_start'])

    return config

def read_configs(csvfile):
    '''
    Read and return a list of config dictionaries, with keys 'tau', 'theta',
    and 's_start'
    '''
    with open(csvfile, 'r') as fin:
        reader = csv.DictReader(fin)
        return [read_config(row) for row in reader]

def mkdir_p(directory):
    'Create the directory if it does not already exist'
    subp.check_call(['mkdir', '-p', directory])

def create_task(fname, problem_base, start, goal):
    '''
    Write the task toml file replacing the start and goal configurations of the
    problem_base.

    @param fname (string): filename to write to
    @param problem_base (toml dict): problem configuration to copy
    @param start (config): start config to put into the problem_base
    @param goal (config): goal config to put into the problem base
    '''
    problem = copy.deepcopy(problem_base)
    problem['problem'].update({
        'start': start['tau'],
        'start_rotation': start['theta'],
        'start_retraction': start['s_start'],
        'goal': goal['tau'],
        'goal_rotation': goal['theta'],
        'goal_retraction': goal['s_start'],
        })
    with open(fname, 'w') as fout:
        toml.dump(problem, fout)

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    problem = toml.load(args.problem)
    configs = read_configs(args.configs_csv)
    mkdir_p(args.directory)

    for i, configs in enumerate(zip(configs[:-1], configs[1:])):
        start, goal = configs
        fname = os.path.join(args.directory, 'task-{:04d}.toml'.format(i+1))
        print('Creating', fname)
        create_task(fname, problem, start, goal)

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
