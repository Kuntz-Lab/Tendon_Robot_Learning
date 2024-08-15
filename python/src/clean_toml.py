#!/usr/bin/env python3
'Loads and outputs the toml file, formatted'

import argparse
import sys
import toml

def populate_parser(parser=None):
    'Populate or create an argument parser'
    if not parser:
        parser = argparse.ArgumentParser()
    parser.description = 'Loads a toml file and outputs it formatted'
    parser.add_argument('toml', help='Toml file to parse')
    parser.add_argument('-i', '--inplace', action='store_true',
                        help='''
                            Modify the file in-place.  The default is to output
                            to stdout.
                            ''')
    return parser

def main(arguments):
    'Main logic here'
    parser = populate_parser()
    args = parser.parse_args(arguments)
    contents = toml.load(args.toml)
    if args.inplace:
        toml.dump(contents, args.toml)
    else:
        print(toml.dumps(contents))
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
