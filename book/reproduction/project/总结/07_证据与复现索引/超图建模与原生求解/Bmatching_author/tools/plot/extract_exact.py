from app.app_io_pb2 import (
    VisualisationsFile,
    Visualisation,
    ExperimentConfig,
    ExperimentResultPart,
)
import google.protobuf.text_format as text_format
import sys
import dataloader
import optperfprofpy
import matplotlib
import matplotlib.pyplot as plt
import os
import numpy as np
import pandas as pd


def extract_exact(data: pd.DataFrame):
    part = data[data["isExact"] == 1] if "isExact" in data else data
    names = part["hypergraph_name"].unique()
    print("Unique exact:", len(names))
    print(names)
    return names


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("stats.py <experiment folder1> <experiment folder2> ....")
        print("Please supply at least arguments")
    for i in range(1, len(sys.argv)):
        # data_table = #dataloader.load_files_to_pandas([sys.argv[i]])
        data = dataloader.load_files([sys.argv[i]])
        # names = extract_exact(data_table)
        for capacity in [0]:  # data_table["runConfig_capacity"].unique():
            pass
        task = ExperimentConfig()
        exakts = {}
        exakts[-1] = ExperimentResultPart()
        exakts[1] = ExperimentResultPart()
        exakts[3] = ExperimentResultPart()
        exakts[5] = ExperimentResultPart()
        for d in data:
            for r in d.results:
                print("BEGIN")
                print(r)
                print("END")
                if r.is_exact and r.run_information.algo_duration.ToSeconds() > 70:
                    exakts[r.run_config.capacity].results.append(r)
            #         print(r.run_information.algo_duration.ToSeconds())
            #         print(r)
            #         task.hypergraphs.append(r.hypergraph)
            #        # exit(0)
        # print(task)
        print(exakts)
        for c in [-1, 1, 5, 3]:
            print(f"LEN {c}:", len(exakts[c].results))
