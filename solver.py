#!/usr/bin/python
# -*- coding: utf-8 -*-

from naivesolver import NaiveSolver
from collections import namedtuple

Customer = namedtuple("Customer", ['index', 'demand', 'x', 'y'])

def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    parts = lines[0].split()
    customer_count = int(parts[0])
    vehicle_count = int(parts[1])
    vehicle_capacity = int(parts[2])

    customers = []
    for i in range(1, customer_count+1):
        line = lines[i]
        parts = line.split()
        customers.append(Customer(i-1, int(parts[0]), float(parts[1]), float(parts[2])))

    solver = NaiveSolver(customers, vehicle_count, vehicle_capacity)
    total_distance, routes = solver.solve()

    output = "{0} 0\n".format(total_distance)
    for route in routes:
        for node in route:
            output += str(node) + " "
        output += "\n"

    return output

if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1:
        file_location = sys.argv[1].strip()
        with open(file_location, 'r') as input_data_file:
            input_data = input_data_file.read()
        print(solve_it(input_data))
    else:

        print(
            'This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/vrp_5_4_1)')
