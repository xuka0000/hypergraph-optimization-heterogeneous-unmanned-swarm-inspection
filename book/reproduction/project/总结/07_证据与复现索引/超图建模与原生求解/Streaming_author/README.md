# heihgm

heihgm is a program to solve matching  in hypergraphs  in (semi-)streaming fashion.

## Compiling & Running one-off computations

We use Bazel as our build system. We recommend using [Bazelisk](https://github.com/bazelbuild/bazelisk) as a wrapper to automatically download and install the correct Bazel version.

Compile the application:
```
bazel build -c opt //app:cli
```

### Running with the CLI

We provide a  CLI interface for running the hypergraph matching algorithms:

```
./bazel-bin/app/cli --algorithm=greedy --mode=in_memory --hypergraph_file=/path/to/hgr
```

**Available Flags:**
- `--algorithm`: The algorithm to run (e.g., `greedy`, `greedy_set`, `best_evict`, `lenient`).
- `--mode`: `in_memory` (default) or `from_disk`.
- `--hypergraph_file` (or `--file`): Path to the hypergraph file.
- `--seed`: Seed for randomization.
- `--eps_alpha`: Sets the `eps` and `alpha` parameters for algorithms that require them (e.g., `greedy`, `greedy_set`, `lenient`).

Run `./bazel-bin/app/cli --help` to see all available algorithms for the selected mode dynamically.


## Running experiments

### Compile

```
bazel build -c opt runner:fork_runner 
```

### Running

```
./bazel-bin/runner/fork_runner --experiment_path=/path/to/folder/containing/experiment  --random_order 1 --max_alloc_memory <max_mem_to_consume>  --max_alloc_memory_per_process <max_memory_for_one_process>  --cycles_to_queue_new 1 --seed 1234 --root_path=/path/to/folder/containing/hypergraphs
```
Please choose sensible values for <max_mem_to_consume> and <max_memory_for_one_process> in MB. So something like 50000 and 100000.

### Plotting
Please install python3.12 via spack (install via spack.io):
```
cd code
spack env activate .
spack install
```

```
spack env activate .
bazel run -c opt tools/plot:plot_cc /absolute/path/to/results/folder /absolute/path/to/visualisation.textproto 
```

## Citation

If you use this software in your research, please cite our paper [Semi-Streaming Algorithms for Hypergraph Matching](https://drops.dagstuhl.de/entities/document/10.4230/LIPIcs.ESA.2025.79):

```bibtex
@InProceedings{reinstadtler_et_al:LIPIcs.ESA.2025.79,
  author =	{Reinst\"{a}dtler, Henrik and Ferdous, S M and Pothen, Alex and U\c{c}ar, Bora and Schulz, Christian},
  title =	{{Semi-Streaming Algorithms for Hypergraph Matching}},
  booktitle =	{33rd Annual European Symposium on Algorithms (ESA 2025)},
  pages =	{79:1--79:19},
  series =	{Leibniz International Proceedings in Informatics (LIPIcs)},
  ISBN =	{978-3-95977-395-9},
  ISSN =	{1868-8969},
  year =	{2025},
  volume =	{351},
  editor =	{Benoit, Anne and Kaplan, Haim and Wild, Sebastian and Herman, Grzegorz},
  publisher =	{Schloss Dagstuhl -- Leibniz-Zentrum f{\"u}r Informatik},
  address =	{Dagstuhl, Germany},
  URL =		{https://drops.dagstuhl.de/entities/document/10.4230/LIPIcs.ESA.2025.79},
  URN =		{urn:nbn:de:0030-drops-245478},
  doi =		{10.4230/LIPIcs.ESA.2025.79},
  annote =	{Keywords: hypergraph, matching, semi-streaming}
}
```
