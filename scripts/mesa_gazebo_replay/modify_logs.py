#!/usr/bin/env python
# coding: utf-8

# Auteur : vdufour
# Date   : 18/05/2021

""" Brief
Script to modify logs file from MESA if necessary.

Main use: split log .json file in several json file to be used with "run_mesa_gazebo_sim.sh"

Options:
    - into_integer = convert data from string format to integer
    - interchange_x_y = convert x to y (and reversed) and invert axes
    - invert_axes = simply invert axes (x=-x and y=-y ...)
    - plot = display a graph to check log data coherence
"""

import numpy as np
import math
import json
import argparse
import matplotlib.pyplot as plt



def main():
    parser = argparse.ArgumentParser(description='Inverse axis in log json file')
    parser.add_argument('--file',type=str)
    parser.add_argument('--option',type=str,help="into_integer=convert coord from string to integer \
                        / interchange_x_y=replace x and y \ invert_axes=change axes senses",default="nothing")
    parser.add_argument('--split',type=bool,default=False)
    #parser.add_argument('--output',type=str,default="compressed_output")
    args = parser.parse_args()

    with open(args.file) as f:
        data = json.load(f)

    if args.split:
        for i in range(0,len(data)):
            splitted_data = data[str(i)]
            with open('intelaero_laserscan_{:d}.json'.format(i),'w') as outfile:
                json.dump(splitted_data,outfile,indent=4)
                print("File number: {:d} created".format(i))
    elif args.option != "nothing":
        for i in range(0,len(data)):
            for j in range(0,len(data[str(i)])):
                x = data[str(i)][j]['x']
                y = data[str(i)][j]['y']
                z = data[str(i)][j]['z']
                if args.option == "into_integer":
                    data[str(i)][j]['x'] = int(x)
                    data[str(i)][j]['y'] = int(y)
                    data[str(i)][j]['z'] = int(z)
                    with open('to_int.json','w') as outfile:
                        json.dump(data,outfile,indent=4)
                elif args.option == "interchange_x_y":
                    data[str(i)][j]['x'] = -int(y)
                    data[str(i)][j]['y'] = -int(x)
                    with open('interchange_x_y.json','w') as outfile:
                        json.dump(data,outfile,indent=4)
                elif args.option == "invert_axes":
                    data[str(i)][j]['x'] = -int(x)
                    data[str(i)][j]['y'] = -int(y)
                    with open('invert_axes.json','w') as outfile:
                        json.dump(data,outfile,indent=4)
                else:
                    break

    else:
        print("Nothing asked")

    if args.option == "plot":
        color = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
        fig = plt.figure(figsize=(15,15))
        plt.axis([0,-120,0,12])
        for i in range(0,len(data)):
            for j in range(0,len(data[str(i)])):
                plt.scatter(data[str(i)][j]["x"],data[str(i)][j]["z"],color=color[i],data=data)
        plt.show()
        print("Displayed")


    print("Job done")


if __name__ == "__main__":
    main()
