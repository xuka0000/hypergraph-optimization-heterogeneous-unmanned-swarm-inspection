from app.app_io_pb2 import VisualisationsFile, Visualisation
import google.protobuf.text_format as text_format
import sys
import dataloader
import optperfprofpy
import matplotlib
import matplotlib.pyplot as plt
import os
import numpy as np
import pandas as pd
import tikzplotlib
from scipy import stats
import json
import concurrent.futures
from multiprocessing import Pool
import threading
import hashlib
import inspect
import glob
import copy
import matplotlib.colors as mcolors
import matplotlib.transforms as transforms

os.register_at_fork(after_in_child=lambda: _get_font.cache_clear())

plot_lock = threading.Lock()
# import debugpy
# debugpy.listen(5678)
# debugpy.wait_for_client()
# matplotlib.use("pgf")
plt.style.use("seaborn")

matplotlib.rcParams.update(
    {
        "pgf.texsystem": "xelatex",
        "font.family": "serif",  # use serif/main font for text elements
        "text.usetex": True,
        "pgf.rcfonts": False,
    }
)
SMALL_SIZE = 8
MEDIUM_SIZE = 10
BIGGER_SIZE = 12


plt.rc("font", size=BIGGER_SIZE)  # controls default text sizes
plt.rc("axes", titlesize=SMALL_SIZE)  # fontsize of the axes title
plt.rc("axes", labelsize=SMALL_SIZE)  # fontsize of the x and y labels
plt.rc("xtick", labelsize=SMALL_SIZE)  # fontsize of the tick labels
plt.rc("ytick", labelsize=SMALL_SIZE)  # fontsize of the tick labels
plt.rc("legend", fontsize=SMALL_SIZE)  # legend fontsize
plt.rc("figure", titlesize=BIGGER_SIZE)  # fontsize of the figure title
plt.rcParams["legend.title_fontsize"] = SMALL_SIZE
plt.rcParams["figure.dpi"] = 300


def escape_(str: str):
    return str.replace("_", "\_")


def prepare_title_tex(vis, data, capacity):
    count = len(data["hypergraph_name"].unique())
    title = vis.title
    title = title.replace("$num", f"{count}")
    capacity_s = f"rnd"
    if capacity != -1:
        capacity_s = f"{capacity}"
    title = title.replace("$capacity", f"{capacity_s}")
    title = title.replace("$performance_characteristic", vis.performance_characteristic)
    return title


def prepare_title_png(vis, data, capacity):
    count = len(data["hypergraph_name"].unique())
    title = vis.title
    title = title.replace("$num", f"{count}")
    capacity_s = f"rnd"
    if capacity != -1:
        capacity_s = f"{capacity}"
    title = title.replace("$capacity", f"{capacity_s}")
    title = title.replace("$performance_characteristic", vis.performance_characteristic)
    return title.replace("\\\\", "\n")


def custom_sum(series):
    return int(series[series == 1].sum())


def solved_instances_(visualisation, data: pd.DataFrame):
    all2 = data.copy()
    groupbysort = visualisation.bool_params["groupby_sort"]
    data = [("all", all2)]
    if groupbysort:
        print("keyxs", all2.keys())
        data2 = {
            sort: all2[all2["hypergraph_sort"] == sort].copy()
            for sort in all2["hypergraph_sort"].unique()
        }
        for sort, all_data in data2.items():
            data = [(sort, all_data)] + data
    res = pd.DataFrame(columns=["runConfig_shortName", "isExact", "capacity", "sort"])
    for sort, all_data in data:
        for capacity in visualisation.capacity:
            with_capacity = all_data[all_data["runConfig_capacity"] == capacity].copy()
            for heuristic_name1 in with_capacity["runConfig_shortName"].unique():
                with_capacity = with_capacity[
                    with_capacity["hypergraph_name"].isin(
                        with_capacity[
                            with_capacity["runConfig_shortName"] == heuristic_name1
                        ]["hypergraph_name"]
                    )
                ]
            if len(with_capacity) == 0:
                print("WARN: empty capacity")
                return
            with_capacity = with_capacity.groupby(
                ["hypergraph_name", "runConfig_shortName"], as_index=False
            ).mean()

            val = (
                with_capacity.groupby("runConfig_shortName")["isExact"]
                .apply(custom_sum)
                .reset_index()
            )
            val["capacity"] = capacity if capacity != -1 else "rnd"
            val["sort"] = sort
            res = res.append(val, ignore_index=True)
            print(val)
    print(res)
    dir = sys.argv[1] + f"/vis/{visualisation.folder_name}/"
    print(dir)
    if not os.path.exists(dir):
        os.makedirs(dir)
    results = res.groupby(["sort", "capacity", "runConfig_shortName"]).mean()
    results.index.rename(["Type", "$b(v)$", "Configuration"], inplace=True)
    if visualisation.bool_params["pivotise"]:
        results = results.reset_index()
        results = pd.pivot_table(
            results,
            values="isExact",
            index=["Type", "$b(v)$"],
            columns=["Configuration"],
            aggfunc=np.mean,
        )
        print(results.columns)
        results.columns = pd.MultiIndex.from_product([["\# Exact"], results.columns])
    return results


def solved_instances(visualisation, data):
    dir = sys.argv[1] + f"/vis/{visualisation.folder_name}/"
    results = solved_instances_(visualisation, data)
    results.style.format("{:.0f}", escape="latex").set_table_styles(
        [
            {"selector": "toprule", "props": ":hline;"},
            {"selector": "midrule", "props": ":hline;"},
            {"selector": "bottomrule", "props": ":hline;"},
        ],
        overwrite=False,
    ).to_latex(
        f"{dir}/{visualisation.file_prefix}_solved_instances.tex",
        multicol_align="c",
        multirow_align="t",
    )


def geo_metric_mean_(visualisation, data: pd.DataFrame):
    all2 = data.copy()
    groupbysort = visualisation.bool_params["groupby_sort"]
    data = [("all", all2)]
    if groupbysort:
        print("keyxs", all2.keys())
        data2 = {
            sort: all2[all2["hypergraph_sort"] == sort].copy()
            for sort in all2["hypergraph_sort"].unique()
        }
        for sort, all_data in data2.items():
            data = [(sort, all_data)] + data
    res = pd.DataFrame(
        columns=[
            "runConfig_shortName",
            "runInformation_algoDuration",
            "capacity",
            "sort",
        ]
    )
    for sort, all_data in data:
        for capacity in visualisation.capacity:
            with_capacity = all_data[all_data["runConfig_capacity"] == capacity].copy()
            if visualisation.string_params["exact_only"] == "true":
                with_capacity = with_capacity[
                    with_capacity["hypergraph_name"].isin(
                        with_capacity[with_capacity["isExact"] == 1][
                            "hypergraph_name"
                        ].unique()
                    )
                ]
            for heuristic_name1 in with_capacity["runConfig_shortName"].unique():
                with_capacity = with_capacity[
                    with_capacity["hypergraph_name"].isin(
                        with_capacity[
                            with_capacity["runConfig_shortName"] == heuristic_name1
                        ]["hypergraph_name"]
                    )
                ]
            if len(with_capacity) == 0:
                continue
            with_capacity = with_capacity.groupby(
                ["hypergraph_name", "runConfig_shortName"], as_index=False
            ).mean()

            val = (
                with_capacity.groupby("runConfig_shortName")[
                    "runInformation_algoDuration"
                ]
                .apply(stats.gmean)
                .reset_index()
            )
            val["capacity"] = capacity if capacity != -1 else "rnd"
            val["sort"] = sort
            res = res.append(val, ignore_index=True)
            print(val)
    print(res)
    dir = sys.argv[1] + f"/vis/{visualisation.folder_name}/"
    print(dir)
    if not os.path.exists(dir):
        os.makedirs(dir)
    results = res.groupby(["sort", "capacity", "runConfig_shortName"]).mean()
    results.index.rename(["Type", "$b(v)$", "Configuration"], inplace=True)
    if visualisation.bool_params["pivotise"]:
        results = results.reset_index()
        results = pd.pivot_table(
            results,
            values="runInformation_algoDuration",
            index=["Type", "$b(v)$"],
            columns=["Configuration"],
            aggfunc=np.mean,
        )
        print(results.columns)
        results.columns = pd.MultiIndex.from_product(
            [["Geometric Mean [s]"], results.columns]
        )
    return results


def geo_metric_mean(visualisation, data: pd.DataFrame):
    dir = sys.argv[1] + f"/vis/{visualisation.folder_name}/"
    results = geo_metric_mean_(visualisation, data)
    results.style.format("{:.2f}").format_index(escape="latex").set_table_styles(
        [
            {"selector": "toprule", "props": ":hline;"},
            {"selector": "midrule", "props": ":hline;"},
            {"selector": "bottomrule", "props": ":hline;"},
        ],
        overwrite=False,
    ).to_latex(
        f"{dir}/{visualisation.file_prefix}_geo_instances.tex",
        column_format="lrlr" if not visualisation.bool_params["pivotise"] else "lrrr",
        multicol_align="c",
        multirow_align="t",
    )


def combined_instances(visualisation, data):
    dir = sys.argv[1] + f"/vis/{visualisation.folder_name}/"
    r1 = geo_metric_mean_(visualisation, data)
    r2 = solved_instances_(visualisation, data)
    print(pd.concat([r1, r2], axis=1).columns)
    pd.concat([r1, r2], axis=1).style.format(
        "{:.2f}", subset=["Geometric Mean [s]"]
    ).format("{:.0f}", subset=["\# Exact"]).format_index(
        escape="latex"
    ).set_table_styles(
        [
            {"selector": "toprule", "props": ":hline;"},
            {"selector": "midrule", "props": ":hline;"},
            {"selector": "bottomrule", "props": ":hline;"},
        ],
        overwrite=False,
    ).to_latex(
        f"{dir}/{visualisation.file_prefix}_combined_instances.tex",
        column_format="lrlr" if not visualisation.bool_params["pivotise"] else "lrrrrr",
        multicol_align="c",
        multirow_align="t",
    )


def performance_profile(visualisation, data):
    prob_def = ["hypergraph_name"]
    max_is_good = True
    if visualisation.bool_params["minimization_problem"]:
        max_is_good = False
        print("Minimization Problem detected")
    perf_meas = [visualisation.performance_characteristic]
    solver_char = ["runConfig_shortName"]
    all2 = data.copy()
    print(
        all2[
            [
                "hypergraph_name",
                "weight",
                "runConfig_shortName",
                "runInformation_algoDuration",
            ]
        ]
    )
    all2 = all2[
        all2[visualisation.performance_characteristic] != 0
    ]  # exclude zero performance
    if visualisation.double_params["max_val"] != 0:
        all2 = all2[
            all2[visualisation.performance_characteristic]
            < visualisation.double_params["max_val"]
        ]  # exclude zero performance
    groupbysort = visualisation.bool_params["groupby_sort"]
    data = {"all": all2}
    if groupbysort:
        print("keyxs", all2.keys())
        data2 = {
            sort: all2[all2["hypergraph_sort"] == sort].copy()
            for sort in all2["hypergraph_sort"].unique()
        }
        for sort, all_data in data2.items():
            data[sort] = all_data
    for sort, all_data in data.items():
        for capacity in visualisation.capacity:
            with_capacity = all_data[all_data["runConfig_capacity"] == capacity].copy()
            if visualisation.bool_params["exact_filter"]:
                with_capacity = with_capacity[
                    with_capacity["hypergraph_name"].isin(
                        with_capacity[with_capacity["isExact"] == 1][
                            "hypergraph_name"
                        ].unique()
                    )
                ]
            if visualisation.string_params["exact_only"] == "true":
                with_capacity = with_capacity[with_capacity["isExact"] == 1]
            for heuristic_name1 in with_capacity["runConfig_shortName"].unique():
                with_capacity = with_capacity[
                    with_capacity["hypergraph_name"].isin(
                        with_capacity[
                            with_capacity["runConfig_shortName"] == heuristic_name1
                        ]["hypergraph_name"]
                    )
                ]
            if len(with_capacity) == 0:
                print("WARN: empty capacity")
                continue
            else:
                print(len(with_capacity), "instance*exp combinations.")
            with_capacity = with_capacity.groupby(
                ["hypergraph_name", "runConfig_shortName"], as_index=False
            ).mean()
            with_capacity["feas"] = True
            tau_vals = None
            if visualisation.double_params["min_tau"] != 0.0:
                tau_vals = np.linspace(
                    visualisation.double_params["min_tau"], 1.0, num=400
                )
            if visualisation.double_params["max_tau"] != 0.0:
                tau_vals = np.linspace(
                    1.0, visualisation.double_params["max_tau"], num=400
                )
            # if visualisation.double_params["max_tau"] != 0.0:
            #    tau_vals = np.linspace(1.0, visualisation.double_params["max_tau"], num=30)
            taus, solver_vals, solvers, transformed_data = optperfprofpy.calc_perprof(
                with_capacity,
                prob_def,
                perf_meas,
                solver_char,
                inv_perf_meas=False,
                maxvalue_good=max_is_good,
                tau_val=tau_vals,
            )  # ,tau_val=np.linspace(1,0,20*len(extended_df)))
            print(taus)
            legend_pos = None
            if "legend_pos" in visualisation.string_params:
                legend_pos = visualisation.string_params["legend_pos"]
            fi = None
            if "fig_height_inch" in visualisation.double_params:
                fi = (
                    visualisation.double_params["fig_width_inch"]
                    if visualisation.double_params["fig_width_inch"] != 0.0
                    else visualisation.double_params["fig_height_inch"]
                    / ((5**0.5 - 1) / 2),
                    visualisation.double_params["fig_height_inch"],
                )

            dir = sys.argv[1] + f"/vis/{visualisation.folder_name}/"
            print(dir)
            if not os.path.exists(dir):
                os.makedirs(dir)
            with plot_lock:
                optperfprofpy.draw_simple_pp(
                    taus,
                    solver_vals,
                    solvers,
                    legend_pos=legend_pos,
                    max_is_good=max_is_good,
                    log_scale=visualisation.bool_params["x_log_scale"],
                    fig_size=fi,
                )
                # plt.xlim(1,  taus.max())
                plt.title(prepare_title_png(visualisation, with_capacity, capacity))
                if not visualisation.bool_params["legend_background_disabled"]:
                    print("LEGEND True")
                plt.legend(frameon=True, loc=4)
                plt.savefig(
                    f"{dir}/{visualisation.file_prefix}_performance_profile_{capacity}_{sort}.png"
                )
                plt.savefig(
                    f"{dir}/{visualisation.file_prefix}_performance_profile_{capacity}_{sort}.pdf",
                    bbox_inches=transforms.Bbox(
                        [
                            [0, -0.05],
                            [3.2, 2.05] if fi is None else [fi[0], fi[1] + 0.05],
                        ]
                    ),
                )
                plt.savefig(
                    f"{dir}/{visualisation.file_prefix}_performance_profile_{capacity}_{sort}.pgf"
                )
                plt.savefig(
                    f"{dir}/{visualisation.file_prefix}_performance_profile_{capacity}_{sort}.raster.pgf",
                    rasterized=True,
                )
                plt.title(prepare_title_tex(visualisation, with_capacity, capacity))
                tikzplotlib.save(
                    f"{dir}/{visualisation.file_prefix}_performance_profile_{capacity}_{sort}.tex",
                    extra_axis_parameters=[
                        "width=\\plotzwidth",
                        "height=\\plotzheight",
                        "legend style={ font = \\tiny }",
                        "align =center",
                    ],
                )
                plt.close()


def geo_mean_overflow(iterable):
    return np.exp(np.log(iterable).mean())


def numprintify(str):
    return "\\numprint{\stripzero{" + f"{str:06d}" + "}}"


def num_printer(v):
    return "\\numprint{" + f"{v:0.2f}" + "}"


def num_print(ser):
    return [num_printer(v) for v in ser]


def speeds(by_heuristic1, groups, rename_groups):
    _groups = groups
    if len(groups) == 0:
        by_heuristic1["Type"] = "all"
        _groups = ["Type"]
    result1 = by_heuristic1.groupby(_groups + ["runConfig_capacity"])
    bef_nodes = result1["hypergraph_nodeCount"].mean().apply(num_printer)
    bef_edges = result1["hypergraph_edgeCount"].mean().apply(num_printer)
    res_edges = result1["edges_0"].mean().apply(num_printer)  # .apply(stats.gmean)
    res_count = result1.count()  # .apply(stats.gmean)
    res_nodes = result1["nodes_0"].mean().apply(num_printer)  # .apply(stats.gmean)
    res_speedup = result1["speedup"].mean().apply(num_printer)
    res_geospeedup = result1["speedup"].apply(stats.gmean).apply(num_printer)
    res_geospeedup.name = "Geom. Speedup"
    res = pd.concat(
        [
            res_count["solution_0"],
            bef_edges,
            res_edges,
            bef_nodes,
            res_nodes,
            res_speedup,
            res_geospeedup,
        ],
        axis=1,
    ).rename(
        {
            "hypergraph_nodeCount": "\# Vertices (b.)",
            "hypergraph_edgeCount": "\# Edges (b.)",
            "runConfig_capacity": "Capacity",
            "edges_0": "\# Edges",
            "nodes_0": "\# Vertices",
            "solution_0": "Instances",
            "speedup": "Avg Speedup",
        },
        axis=1,
    )
    print(res)
    res.index.rename(
        (rename_groups if len(groups) > 0 else ["Type"]) + ["$b(v)$"],
        inplace=True,
    )
    return res


def removal_effectivness(visualisation: Visualisation, data: pd.DataFrame):
    data["runConfig_capacity"] = [
        "rnd" if c == -1 else str(c) for c in data["runConfig_capacity"]
    ]
    groups = ["hypergraph_nodeCount", "hypergraph_edgeCount"]
    rename_groups = ["\# Vertices", "\# Edges"]
    if visualisation.bool_params["groupby_sort"]:
        groups = ["hypergraph_sort"]
        rename_groups = ["Type"]
    dir = sys.argv[1] + f"/vis/{visualisation.folder_name}/"
    if not os.path.exists(dir):
        os.makedirs(dir)
    res = []
    data["edges_0"] = pd.to_numeric(
        [i[0]["edgeCount"] for i in data["algorithmRunInformations"]]
    )
    data["solution_0"] = pd.to_numeric(
        [
            i[0]["size"] if "size" in i[0] else 0
            for i in data["algorithmRunInformations"]
        ]
    )
    data["edges_0"] = data["edges_0"] - data["solution_0"]
    data["nodes_0"] = pd.to_numeric(
        [
            i[0]["nodeCount"] if "nodeCount" in i[0] else 0
            for i in data["algorithmRunInformations"]
        ]
    )
    with_capacity = data.copy()
    heuristic_name1 = visualisation.string_params["heuristic_name1"]
    heuristic_name2 = visualisation.string_params["heuristic_name2"]
    if heuristic_name1 == "":
        print("Please specify 'heuristic_name1'")
        exit(1)
    if visualisation.bool_params["exact_filter"]:
        with_capacity2 = (
            with_capacity.groupby(["hypergraph_name", "runConfig_capacity"])["isExact"]
            .sum()
            .reset_index()
        )
        exact_solved = [
            (r["hypergraph_name"], r["runConfig_capacity"])
            for _, r in with_capacity2[with_capacity2["isExact"] > 0].iterrows()
        ]
        with_capacity = with_capacity[
            with_capacity[["hypergraph_name", "runConfig_capacity"]]
            .apply(tuple, axis=1)
            .isin(exact_solved)
        ]
    if visualisation.string_params["exact_only"] == "true":
        with_capacity = with_capacity[with_capacity["isExact"] == 1]
    by_heuristic1 = (
        with_capacity[with_capacity["runConfig_shortName"] == heuristic_name1]
        .groupby(["hypergraph_sort", "hypergraph_name", "runConfig_capacity"])
        .mean()
    )
    by_heuristic2 = (
        with_capacity[with_capacity["runConfig_shortName"] == heuristic_name2]
        .groupby(["hypergraph_sort", "hypergraph_name", "runConfig_capacity"])
        .mean()
    )
    by_heuristic1["speedup"] = (
        by_heuristic2["runInformation_algoDuration"]
        / by_heuristic1["runInformation_algoDuration"]
    )

    print(by_heuristic1.columns)
    res = pd.concat(
        [
            speeds(by_heuristic1, groups, rename_groups),
            speeds(by_heuristic1, [] if len(groups) == 1 else rename_groups, "all"),
        ]
    )

    res.style.set_table_styles(
        [
            {"selector": "toprule", "props": ":hline;"},
            {"selector": "midrule", "props": ":hline;"},
            {"selector": "bottomrule", "props": ":hline;"},
        ],
        overwrite=False,
    ).to_latex(
        f"{dir}/{visualisation.file_prefix}_effectivness_reduction.tex",
        column_format="rrrrrrrrr",
        multirow_align="t",
    )


def process_text(r: str, rules):
    if len(rules) == 0:
        return r
    for k in rules:
        r = r.replace(k, rules[k])
    return r


def removal_plot(visualisation: Visualisation, data: pd.DataFrame):
    heuristic_name1 = visualisation.string_params["heuristic_name1"]
    print(heuristic_name1)
    data = data[data["runConfig_shortName"] == heuristic_name1]
    if len(data) == 0:
        print(
            "WARNING: probably you should pass another heuristic_name1, because this one is empty."
        )
    print(data.columns)
    print(data["hypergraph_sort"])
    dir = sys.argv[1] + f"/vis/{visualisation.folder_name}/"
    if not os.path.exists(dir):
        os.makedirs(dir)
    res = []
    data["edges_0"] = pd.to_numeric(
        [
            i[0]["freeEdges"] if "freeEdges" in i[0] else 0
            for i in data["algorithmRunInformations"]
        ]
    )
    data["solution_0"] = pd.to_numeric(
        [
            i[0]["size"] if "size" in i[0] else 0
            for i in data["algorithmRunInformations"]
        ]
    )
    # data["edges_0"] = data["edges_0"] - data["solution_0"]
    data["nodes_0"] = pd.to_numeric(
        [
            i[0]["nodeCount"] if "nodeCount" in i[0] else 0
            for i in data["algorithmRunInformations"]
        ]
    )
    with_capacity = data
    sort_crit = "hypergraph_sort"
    if visualisation.string_params["sort_name"] != "":
        sort_crit = visualisation.string_params["sort_name"]
    if heuristic_name1 == "":
        print("Please specify 'heuristic_name1'")
        exit(1)
    if visualisation.string_params["exact_only"] == "true":
        with_capacity = with_capacity[with_capacity["isExact"] == 1]
    by_heuristic1 = (
        with_capacity[with_capacity["runConfig_shortName"] == heuristic_name1]
        .groupby(
            list(set([sort_crit, "hypergraph_name", "runConfig_capacity"])),
            as_index=False,
        )
        .mean()
        .copy()
    )
    # by_heuristic1 = by_heuristic1.reset_index()
    nodes_rel = by_heuristic1["nodes_0"] / by_heuristic1["hypergraph_nodeCount"]
    edges_rel = by_heuristic1["edges_0"] / by_heuristic1["hypergraph_edgeCount"]
    cols = {}
    marks = [
        ".",
        "o",
        "v",
        "^",
        "<",
        ">",
        "8",
        "s",
        "p",
        "*",
        "h",
        "H",
        "D",
        "d",
        "P",
        "X",
    ]
    i = 0
    sort = {}
    i = 0
    for so in by_heuristic1[sort_crit].unique():
        sort[so] = list(mcolors.TABLEAU_COLORS.values())[i]
        i += 1
    result = []
    fi = None
    if "fig_height_inch" in visualisation.double_params:
        fi = (
            visualisation.double_params["fig_width_inch"]
            if visualisation.double_params["fig_width_inch"] != 0.0
            else visualisation.double_params["fig_height_inch"] / ((5**0.5 - 1) / 2),
            visualisation.double_params["fig_height_inch"],
        )
    fig = plt.figure(figsize=fi)
    ax = fig.add_axes([0.125, 0.175, 0.80, 0.75])
    im = 0
    by_cap = []
    for c in by_heuristic1["runConfig_capacity"].unique():
        result = []
        for co in by_heuristic1[sort_crit].unique():
            result += [
                ax.scatter(
                    nodes_rel[
                        (by_heuristic1["runConfig_capacity"] == c)
                        & (by_heuristic1[sort_crit] == co)
                    ],
                    edges_rel[
                        (by_heuristic1["runConfig_capacity"] == c)
                        & (by_heuristic1[sort_crit] == co)
                    ],
                    marker=marks[im],
                    c=sort[co],
                    label=str(c),
                    alpha=0.5,
                    s=(
                        4
                        + 20
                        * np.log(
                            (
                                by_heuristic1[
                                    (by_heuristic1["runConfig_capacity"] == c)
                                    & (by_heuristic1[sort_crit] == co)
                                ]["hypergraph_edgeCount"]
                            )
                        )
                        / np.max(
                            np.log(
                                (
                                    by_heuristic1[
                                        (by_heuristic1["runConfig_capacity"] == c)
                                        & (by_heuristic1[sort_crit] == co)
                                    ]["hypergraph_edgeCount"]
                                )
                            )
                        )
                    )
                    if "marker_size" not in visualisation.double_params
                    else visualisation.double_params["marker_size"],
                )
            ]
        by_cap += [result]
        im += 1
    legend_pos = None
    if "legend_pos" in visualisation.string_params:
        legend_pos = visualisation.string_params["legend_pos"]
    # Create a legend for marker shapes
    marker_legend = ax.legend(
        [l for l in by_cap[0]],
        [
            process_text(r, visualisation.rename_labels)
            for r in by_heuristic1[sort_crit].unique()
        ],
        loc="lower right"
        if "legend_sort_pos" not in visualisation.string_params
        else visualisation.string_params["legend_sort_pos"],
        ncol=len(by_heuristic1[sort_crit].unique()) // 2
        if "ncol" not in visualisation.int64_params
        else visualisation.int64_params["ncol"],
        frameon=True,
    )

    # Create a legend for colors
    color_legend = ax.legend(
        [l[0] for l in by_cap],
        by_heuristic1["runConfig_capacity"].unique(),
        loc="center left",
        title="Capacity",
        frameon=True,
    )

    # Add the marker legend back to the axis
    ax.add_artist(marker_legend)
    plt.xlabel("Nodes")
    plt.ylabel("Edges")
    plt.title(prepare_title_png(visualisation, with_capacity, 0))
    plt.savefig(f"{dir}/{visualisation.file_prefix}_removal_plot.png")
    plt.savefig(
        f"{dir}/{visualisation.file_prefix}_removal_plot.pdf",
        bbox_inches=transforms.Bbox(
            [[0, -0.05], [3.2, 2.05] if fi is None else [fi[0], fi[1] + 0.05]]
        ),
    )
    plt.savefig(f"{dir}/{visualisation.file_prefix}_removal_plot.pgf")


def time_v_weight(visualisation: Visualisation, data: pd.DataFrame):
    dir = sys.argv[1] + f"/vis/{visualisation.folder_name}/"
    weight_param = "weight"
    if visualisation.string_params["weight_param"] != "":
        weight_param = visualisation.string_params["weight_param"]
    if not os.path.exists(dir):
        os.makedirs(dir)
    for capacity in visualisation.capacity:
        with_capacity = data[(data["runConfig_capacity"] == capacity)].copy()
        print(with_capacity.sort_values("hypergraph_name"))
        with_capacity.sort_values("hypergraph_name").to_csv(
            sys.argv[1] + f"/vis/{visualisation.folder_name}/data{capacity}.csv"
        )
        heuristic_name1 = visualisation.string_params["heuristic_name1"]
        heuristic_name2 = visualisation.string_params["heuristic_name2"]
        if heuristic_name1 == "" or heuristic_name2 == "":
            print("Please specify 'heuristic_name1' or 'heuristic_name2'")
            exit(1)
        if visualisation.string_params["exact_only"] == "true":
            with_capacity = with_capacity[with_capacity["isExact"] == 1]
            with_capacity = with_capacity[
                with_capacity["hypergraph_name"].isin(
                    with_capacity[
                        with_capacity["runConfig_shortName"] == heuristic_name1
                    ]["hypergraph_name"]
                )
            ]
            with_capacity = with_capacity[
                with_capacity["hypergraph_name"].isin(
                    with_capacity[
                        with_capacity["runConfig_shortName"] == heuristic_name2
                    ]["hypergraph_name"]
                )
            ]
        by_heuristic1 = (
            with_capacity[with_capacity["runConfig_shortName"] == heuristic_name1]
            .groupby("hypergraph_name")
            .mean()
        )
        by_heuristic2 = (
            with_capacity[with_capacity["runConfig_shortName"] == heuristic_name2]
            .groupby("hypergraph_name")
            .mean()
        )
        by_heuristic1["weight_2"] = by_heuristic2[weight_param]
        print(capacity)
        print(
            [
                n
                for n in by_heuristic1[
                    by_heuristic1[weight_param] != by_heuristic1["weight_2"]
                ].index
            ]
        )
        fig = plt.figure()
        ax = plt.gca()
        ax.set_xlabel(escape_(f"{heuristic_name1}/{heuristic_name2} time"))
        ax.set_ylabel(escape_(f"{heuristic_name1}/{heuristic_name2} weight"))

        ax.scatter(
            by_heuristic1["runInformation_algoDuration"]
            / by_heuristic2["runInformation_algoDuration"],
            # c=["r" if i == 0 else "b" for i in by_heuristic1["isExact"]],
            by_heuristic1[weight_param] / by_heuristic2[weight_param],
            label="instances",
        )
        geo_mean1_weight = geo_mean_overflow(by_heuristic1[weight_param])
        geo_mean1_time = geo_mean_overflow(by_heuristic1["runInformation_algoDuration"])

        geo_mean2_weight = geo_mean_overflow(by_heuristic2[weight_param])
        geo_mean2_time = geo_mean_overflow(by_heuristic2["runInformation_algoDuration"])
        print(geo_mean1_time, geo_mean2_time, geo_mean1_weight, geo_mean2_weight)
        print(
            by_heuristic1[
                by_heuristic1["runInformation_algoDuration"]
                < 0.5 * by_heuristic2["runInformation_algoDuration"]
            ].to_csv(f"{dir}/{visualisation.file_prefix}_times_{capacity}.csv")
        )
        ax.scatter(
            geo_mean1_time / geo_mean2_time,
            geo_mean1_weight / geo_mean2_weight,
            c=["orange"],
            label="geo\_mean",
        )
        plt.title(prepare_title_png(vis, with_capacity, capacity))
        plt.grid(True)
        plt.legend()
        with plot_lock:
            plt.savefig(
                f"{dir}/{visualisation.file_prefix}_time_v_weight_{capacity}.png"
            )
            plt.savefig(
                f"{dir}/{visualisation.file_prefix}_time_v_weight_{capacity}.pdf",
                bbox_inches=transforms.Bbox([[0, -0.05], [3.2, 2.05]]),
            )
            plt.savefig(
                f"{dir}/{visualisation.file_prefix}_time_v_weight_{capacity}.pgf",
            )
            plt.savefig(
                f"{dir}/{visualisation.file_prefix}_time_v_weight_{capacity}.raster.pgf",
                rasterized=True,
            )
            plt.title(prepare_title_tex(vis, with_capacity, capacity))
            tikzplotlib.save(
                f"{dir}/{visualisation.file_prefix}_time_v_weight_{capacity}.tex",
                extra_axis_parameters=[
                    "width=\\plotzwidth",
                    "height=\\plotzheight",
                    "legend style={ font = \\tiny }",
                    "align =center",
                ],
            )
            plt.close()


sys_i = ""


def generate_folders(f_path):
    with open(f_path, "r") as f:
        visualisations_file = text_format.Parse(f.read(), VisualisationsFile())
        for visualisation in visualisations_file.visualisations:
            dir = sys.argv[1] + f"/vis/{visualisation.folder_name}/"
            print(dir)
            if not os.path.exists(dir):
                os.makedirs(dir)


def hash_data(vis):
    readable_hash = []
    for dir in vis.experiment_paths:  # order is given by hash
        for filename in np.sort(
            glob.glob(sys.argv[1] + dir + "/result-*-*.binary_proto")
        ):
            with open(filename, "rb") as f:
                bytes = f.read()  # read entire file as bytes
                readable_hash += [hashlib.sha256(bytes).hexdigest()]
    return hashlib.sha256(
        b"".join(str(field).encode() for field in readable_hash)
    ).hexdigest()


def hash_sha(vis):
    fields_to_hash = [
        value for field_descriptor, value in vis.ListFields() if value != 0
    ]
    func = None
    if vis.type == "performance_profile":
        if vis.performance_characteristic == "":
            vis.performance_characteristic = "weight"
        func = performance_profile
    if vis.type == "time_v_weight":
        func = time_v_weight
    if vis.type == "removal_effectivness":
        func = removal_effectivness
    if vis.type == "solved_instances":
        func = solved_instances
    if vis.type == "geometric_mean":
        func = geo_metric_mean
    if vis.type == "removal_plot":
        func = removal_plot
    if vis.type == "combined_instances":
        func = combined_instances
    if func is None:
        print("WARN", vis.type, " is not known!")
        data = b"".join(str(field).encode() for field in fields_to_hash)
        # Calculate the hash value using SHA-256
        return hashlib.sha256(data).hexdigest()
    fields_to_hash += [inspect.getsource(func), hash_data(vis)]
    # Convert the field values to bytes and concatenate them
    data = b"".join(str(field).encode() for field in fields_to_hash)
    # Calculate the hash value using SHA-256
    return hashlib.sha256(data).hexdigest()


def write_hash(vis, hash):
    dir = sys.argv[1] + f"/vis/{vis.folder_name}/"
    if not os.path.exists(dir):
        os.makedirs(dir)
    path = f"{dir}/{vis.file_prefix}.sha"
    with open(path, "w") as f:
        print(hash, " written")
        f.write(hash)


def hash_exists(vis, hash):
    dir = sys.argv[1] + f"/vis/{vis.folder_name}/"
    path = f"{dir}/{vis.file_prefix}.sha"
    if not os.path.exists(path):
        return False
    with open(path, "r") as f:
        content = f.read()
        if content != hash:
            print(content, " does not match ", hash_sha(vis))
            return False
    return True


def process_visfile(f_path):
    print("Now plotting", f_path, "sys", sys_i, "argv:", sys.argv[1])
    with open(f_path, "r") as f:
        visualisations_file = text_format.Parse(f.read(), VisualisationsFile())
        for vis in visualisations_file.visualisations:
            hash_vis = hash_sha(vis)
            cop = copy.deepcopy(vis)
            if hash_exists(vis, hash_vis):
                print("Skipping")
                continue
            data = dataloader.load_files_to_pandas(
                [sys.argv[1] + f"/{exp_path}" for exp_path in vis.experiment_paths],
                vis.bool_params["prefix_names"],
            )
            if len(data) == 0:
                print("Skipping empty data")
                continue
            for i in range(10):
                if vis.string_params[f"ignore_substr{i}"] != "":
                    data = data[
                        data["runConfig_shortName"].str.contains(
                            vis.string_params[f"ignore_substr{i}"]
                        )
                        == False
                    ]
            if len(vis.rename_labels) > 0:
                data["runConfig_shortName"] = data["runConfig_shortName"].replace(
                    vis.rename_labels
                )
            # Check if is valid data set.
            if not "runConfig_capacity" in data.columns:
                print(f"[WARN] Skipping empty/invalid data in {vis.title}")
                continue
            if vis.type == "performance_profile":
                if vis.performance_characteristic == "":
                    vis.performance_characteristic = "weight"
                performance_profile(vis, data)
            if vis.type == "time_v_weight":
                time_v_weight(vis, data)
            if vis.type == "removal_effectivness":
                removal_effectivness(vis, data)
            if vis.type == "solved_instances":
                solved_instances(vis, data)
            if vis.type == "geometric_mean":
                geo_metric_mean(vis, data)
            if vis.type == "combined_instances":
                combined_instances(vis, data)
            if vis.type == "removal_plot":
                print("pass", data.columns)
                print(data["hypergraph_sort"])
                removal_plot(vis, data)
            print(vis)
            print("before")
            print(cop)
            print(hash_sha(vis))
            print(hash_vis)

            write_hash(vis, hash_vis)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("plot.py <path to experiment git> <visualisation files>")
        print("Please supply at least arguments")
    sys_i = sys.argv[1]
    for i in range(2, len(sys.argv)):
        generate_folders(sys.argv[i])
        process_visfile(sys.argv[i])
    # with concurrent.futures.ThreadPoolExecutor() as executor:
    #     executor.map(process_visfile, [sys.argv[i] for i in range(2, len(sys.argv))])
