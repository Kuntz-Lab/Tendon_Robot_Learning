#!/usr/bin/env python3
import argparse
import sys
import numpy as np
import csv
import cpptendon as t

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    
    parser.add_argument('-r', '--read', default='sphere.csv',
                        help='read csv file (default is sphere.csv)')
    parser.add_argument('-o', '--output', default='nearest_sphere.csv',
                        help='output csv file (default is nearest_sphere.csv)')

    parser.add_argument('-t', '--table', default='lookup_table.csv',
                        help='output csv file (default is lookup_table.csv)')
    return parser

def from_txt_to_csv():
    with open('diamond.txt') as infile, open('diamond.csv', 'w') as outfile:
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


def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    
    desired_tip = read_csv(args.read)
    lookup = read_csv(args.table)[1:]
    # Populate nn 
    nn = t.motion_planning.NearestNeighborL2()
    for row in lookup:
        nn.add(list(map(float,row[-3:])), list(map(float,row[:-3])))        

    result = []
    for row in desired_tip:
        mergedList = []
        key, value = nn.nearest(list(map(float, row[:-1])))
        
        mergedList.extend(value)
        mergedList.extend(key.tolist())
        mergedList.extend(list(map(float, row[:-1])))
        result.append(mergedList)

    write_csv(args.output, result)

if __name__=='__main__':
    main(sys.argv[1:])

