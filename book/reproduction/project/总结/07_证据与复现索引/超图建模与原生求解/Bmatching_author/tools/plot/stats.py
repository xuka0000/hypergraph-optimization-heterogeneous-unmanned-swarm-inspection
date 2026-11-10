from app.app_io_pb2 import VisualisationsFile, Visualisation
import google.protobuf.text_format as text_format
import sys

print(sys.version)
import dataloader
import optperfprofpy
import matplotlib
import matplotlib.pyplot as plt
import os
import numpy as np
import pandas as pd
import tikzplotlib
from scipy.stats import gmean
import json


def print_s(part: pd.DataFrame):
    print("    # experiments:   ", len(part))
    print("    AVG weight:      ", np.average(part["weight"]))

    print("    GMEAN weight:     ", gmean(part["weight"]))
    print(
        "    AVG runTime:     ",
        np.average(part["runInformation_algoDuration"]),
    )
    print(
        "    GMEAN runTime:    ",
        gmean(part["runInformation_algoDuration"]),
    )
    if "isExact" in part:
        print("    # exact:         ", np.sum(part["isExact"] == 1))
        print(
            "    # exact AVG TIME:",
            np.average(part[part["isExact"] == 1]["runInformation_algoDuration"]),
        )
        print(
            "    # exact GMEAN TIME:",
            gmean(part[part["isExact"] == 1]["runInformation_algoDuration"]),
        )
        print(
            "    # exact MAX TIME:",
            np.max(part[part["isExact"] == 1]["runInformation_algoDuration"]),
        )
    else:
        print("    # exact:        0")
        print(
            "    # exact:         ",
            np.sum(part["runInformation_algoDuration"] < 1780),
        )
        print(
            "    # exact AVG TIME:",
            np.average(
                part[part["runInformation_algoDuration"] < 1780][
                    "runInformation_algoDuration"
                ]
            ),
        )
        print(
            "    # exact GMEAN TIME:",
            gmean(
                part[part["runInformation_algoDuration"] < 1780][
                    "runInformation_algoDuration"
                ]
            ),
        )
        print(
            "    # exact MAX TIME:",
            np.max(
                part[part["runInformation_algoDuration"] < 1780][
                    "runInformation_algoDuration"
                ]
            ),
        )


def stats_print(data: pd.DataFrame):
    print(data.keys())
    print("General Information:")
    print("Hostname:", data["runInformation_machine_hostName"].unique())
    print("Git describe:", data["runInformation_gitDescribe"].unique())
    print("Configs: ", data["runConfig_shortName"].unique())
    print("Capacities: ", data["runConfig_capacity"].unique())
    for config in np.sort(data["runConfig_shortName"].unique()):
        for capacity in np.sort(data["runConfig_capacity"].unique()):
            print("Config: ", config, " with b(v)=", capacity, "all")
            part = data[
                (data["runConfig_capacity"] == capacity)
                & (data["runConfig_shortName"] == config)
            ]
            print_s(part)
            for sort in np.sort(data["hypergraph_sort"].unique()):
                print("Config: ", config, " with b(v)=", capacity, "sort=", sort)
                part = data[
                    (data["runConfig_capacity"] == capacity)
                    & (data["runConfig_shortName"] == config)
                    & (data["hypergraph_sort"] == sort)
                ]
            # print_s(part)


previous_res = {}


def load_and_check(data):
    for index, line in data.iterrows():
        if line["isExact"] == 1:
            name = line["hypergraph_name"] + str(line["runConfig_capacity"])
            if name in previous_res:
                print("found")
                if line["weight"] != previous_res[name][0]:
                    print("ALERM")
                    print(
                        name,
                        line["runInformation_algoDuration"],
                        previous_res[name][0],
                        line["weight"],
                    )
            else:
                previous_res[name] = (
                    line["weight"],
                    line["runInformation_algoDuration"],
                    line["algorithmRunInformations"],
                )


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("stats.py <experiment folder1> <experiment folder2> ....")
        print("Please supply at least arguments")
    for i in range(1, len(sys.argv)):
        data = dataloader.load_files_to_pandas([sys.argv[i]])
        stats_print(data)
