import numpy as np
import math
import json
import argparse
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser(description='Inverse axis in log json file')
    parser.add_argument('--file',type=str)
    args = parser.parse_args()

    with open(args.file) as f:
        data = json.load(f)

    size = 0
    for j in range(0,len(data)):
        index = []
        for i in range(1,len(data[str(j)])-1):
            if ((data[str(j)][i]["y"] == data[str(j)][i+1]["y"]) & (data[str(j)][i]["z"] == data[str(j)][i+1]["z"])) | ((data[str(j)][i]["y"] == data[str(j)][i+1]["y"]) & (data[str(j)][i]["x"] == data[str(j)][i+1]["x"])):
                index.append(i)
        index.sort(reverse=True)
        for k in index:
            del data[str(j)][k]
        size = max(size,len(data[str(j)]))

    for j in range(0,len(data)):
        if len(data[str(j)]) < size:
            for i in range(len(data[str(j)]),size):
                data[str(j)].append(data[str(j)][-1])


    with open('compressed_coords.json','w') as outfile:
        json.dump(data,outfile,indent=4)

    print("Job done")



if __name__ == "__main__":
    main()
