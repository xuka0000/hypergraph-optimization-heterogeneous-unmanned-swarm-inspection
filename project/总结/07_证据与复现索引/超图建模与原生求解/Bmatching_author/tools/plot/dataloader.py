import pandas as pd
import numpy as np
from app.app_io_pb2 import Result, ExperimentResultPart
from collections.abc import MutableMapping
import glob
from google.protobuf import json_format


def flatten(d, parent_key="", sep="_"):
    items = []
    for k, v in d.items():
        new_key = parent_key + sep + k if parent_key else k
        if "uration" in new_key:
            v = float(v.replace("s", ""))
        if isinstance(v, MutableMapping):
            items.extend(flatten(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def load_files(dirs):
    results_all = []
    try:
        for dir in dirs:
            results_part = []
            for res in glob.glob(dir + "/result-*-*.binary_proto"):
                part = ExperimentResultPart()
                f = open(res, "rb")
                part.ParseFromString(f.read())
                f.close()
                results_part += [part]
            if len(results_part) == 0:
                return pd.DataFrame()
            # validate thread count
            ids = np.sort(np.array([p.this_process for p in results_part]))
            if not np.array_equal(ids, np.unique(ids)) or not np.array_equal(
                ids, np.array(range(results_part[0].concurrent_processes))
            ):
                print(ids, np.unique(ids))
                print(dirs, "not all contained")
                # exit(1)
            results_all += results_part
        return results_all
    except IOError:
        print(sys.argv[1] + ": Could not open file.")
        exit(1)


def prefix(p, prefix_names, dir: str):
    if prefix_names:
        p.run_config.short_name = dir.split("/")[-1] + "/" + p.run_config.short_name
    return p


def load_files_to_pandas(dirs, prefix_names=False):
    results_all = []
    try:
        for dir in dirs:
            results_part = []
            for res in glob.glob(dir + "/result-*-*.binary_proto"):
                part = ExperimentResultPart()
                f = open(res, "rb")
                part.ParseFromString(f.read())
                f.close()
                results_part += [part]
            if len(results_part) == 0:
                return pd.DataFrame()
            # validate thread count
            ids = np.sort(np.array([p.this_process for p in results_part]))
            if not np.array_equal(ids, np.unique(ids)) or not np.array_equal(
                ids, np.array(range(results_part[0].concurrent_processes))
            ):
                print(ids, np.unique(ids))
                print(dirs, "not all contained")
                # exit(1)
            results_all += [(p, dir) for p in results_part]
    except IOError:
        print(sys.argv[1] + ": Could not open file.")
        exit(1)
    results = [
        flatten(json_format.MessageToDict(prefix(r, prefix_names, dir)))
        for res, dir in results_all
        for r in res.results
    ]
    res2 = pd.DataFrame(results)
    for col in res2.columns:
        res2[col] = pd.to_numeric(res2[col], errors="ignore")
    res2 = res2.fillna(0)
    for col in res2.columns:
        # res2[col] = [str(r) for r in res2[col]]
        res2[col] = pd.to_numeric(res2[col], errors="ignore")
    return res2
