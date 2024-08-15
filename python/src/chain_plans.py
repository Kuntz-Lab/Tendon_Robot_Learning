#!/usr/bin/env python3
'''
Joins csv files for multiple tendon robot plans.  It will account for the
rotation boundary condition on the theta if it is there.
'''

import argparse
import csv
import math
import os
import sys

from csv_combine import all_have_same_header_rows

def populate_parser(parser=None):
    if not parser:
        parser = argparse.ArgumentParser(
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
            )
    parser.description = '''
        Joins csv files for multiple tendon robot plans into one cohesive plan.
        The plans should be specified in order so that it makes sense to put
        them all together.  For duplicate rows (like when the next plan starts
        from the goal of the previous), they will try to be removed.
        '''
    parser.add_argument('csvfiles', metavar='csv', nargs='+',
                        help='''
                            List of csv files specifying the plans to join
                            together.  They will be joined in the order
                            specified here.
                            ''')
    parser.add_argument('-o', '--output', default='chained.csv',
                        help='output file')
    return parser

def fix_theta(prev_theta, theta):
    '''
    Fix theta to not wrap around.  Basically returns an equivalent value for
    theta that is less than or equal to pi distance away from prev_theta.

    >>> fix_theta(1.5, -3.1)
    -1.4584073464102065
    '''
    dx = theta - prev_theta
    while abs(dx) > math.pi:
        sign = -1 if dx < 0 else 1
        theta -= sign * 2 * math.pi
        dx = theta - prev_theta
    return theta

def is_duplicate(row_a, row_b):
    'Checks if these two rows are equivalent'
    for key in row_a:
        if abs(float(row_a[key]) - float(row_b[key])) > 1e-5:
            return False
    return True

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    if not all_have_same_header_rows(args.csvfiles):
        print('All given CSV files must have the same header row',
              file=sys.stderr)
        return 1

    # get header row
    with open(args.csvfiles[0], 'r') as fin:
        reader = csv.DictReader(fin)
        header = reader.fieldnames

    with open(args.output, 'w') as fout:
        writer = csv.DictWriter(fout, header)
        writer.writeheader()

        prev_row = {x: 0.0 for x in header}
        i = 1

        for csvfile in args.csvfiles:
            print(f'{csvfile} >> {args.output}')
            with open(csvfile, 'r') as fin:
                reader = csv.DictReader(fin)
                for row in reader:

                    # fix theta
                    # if 'theta' in row:
                    #     row['theta'] = fix_theta(
                    #         float(prev_row['theta']),
                    #         float(row['theta']))

                    # filter duplicate rows
                    row['i'] = prev_row['i']
                    if is_duplicate(prev_row, row):
                        continue

                    # renumber i
                    if 'i' in row:
                        row['i'] = i
                    i += 1

                    writer.writerow(row)
                    prev_row = row

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
