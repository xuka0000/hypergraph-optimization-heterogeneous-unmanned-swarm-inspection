import optperfprofpy
from google.protobuf import json_format
import json
import pandas as pd
import numpy as np
import sys
from app.app_io_pb2 import Result, ExperimentResultPart
from collections.abc import MutableMapping
import os
import matplotlib.pyplot as plt
import dataloader


# import debugpy
# debugpy.listen(5678)
# debugpy.wait_for_client()

res2 = dataloader.load_files_to_pandas(sys.argv[1:])

print(res2.head())
cwd = os.getcwd()
print(cwd)
prob_def = ['hypergraph_name']
perf_meas = ['weight']
solver_char = ['runConfig_shortName']
for capacity in [1, 3, 5]:
    extended_df = res2.copy()
    extended_df = extended_df[extended_df["runConfig_capacity"]
                              == capacity].copy()
    print("len:", len(extended_df))
    extended_df["feas"] = True
    print(extended_df["weight"])
    print(extended_df["hypergraph_name"])
    print(capacity)

    taus, solver_vals, solvers, transformed_data = optperfprofpy.calc_perprof(
        extended_df, prob_def, perf_meas, solver_char, inv_perf_meas=False)  # ,tau_val=np.linspace(1,0,20*len(extended_df)))

    optperfprofpy.draw_simple_pp(taus, solver_vals, solvers)
    plt.title(
        f"PP on walshaw hypergraphs (e-weight random(100), capacity {capacity})")

    plt.savefig(sys.argv[1]+f"/plot{capacity}.png")
    print("saved")
