# HeiHGM::BMatching

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![CI](https://github.com/HeiHGM/Bmatching/actions/workflows/ci.yml/badge.svg)](https://github.com/HeiHGM/Bmatching/actions/workflows/ci.yml)
[![GitHub Release](https://img.shields.io/github/v/release/HeiHGM/Bmatching)](https://github.com/HeiHGM/Bmatching/releases/latest)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-orange.svg)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/CMake-%3E%3D%203.20-blue.svg)](https://cmake.org/)
[![Linux](https://img.shields.io/badge/platform-Linux-blue)](https://github.com/HeiHGM/Bmatching)
[![macOS](https://img.shields.io/badge/platform-macOS-blue)](https://github.com/HeiHGM/Bmatching)
[![Homebrew](https://img.shields.io/badge/homebrew-available-orange)](https://github.com/HeiHGM/homebrew-bmatching)
[![GitHub Stars](https://img.shields.io/github/stars/HeiHGM/Bmatching)](https://github.com/HeiHGM/Bmatching/stargazers)
[![GitHub Issues](https://img.shields.io/github/issues/HeiHGM/Bmatching)](https://github.com/HeiHGM/Bmatching/issues)
[![GitHub Last Commit](https://img.shields.io/github/last-commit/HeiHGM/Bmatching)](https://github.com/HeiHGM/Bmatching/commits)
[![DOI](https://img.shields.io/badge/DOI-10.7155%2Fjgaa.v30i1.3166-green.svg)](https://doi.org/10.7155/jgaa.v30i1.3166)
[![arXiv](https://img.shields.io/badge/arXiv-2408.06924-b31b1b)](https://arxiv.org/abs/2408.06924)
[![Zenodo](https://img.shields.io/badge/Zenodo-data-blue)](https://doi.org/10.5281/zenodo.18225669)
[![Agent-Ready](https://img.shields.io/badge/agent--ready-yes-brightgreen)](https://github.com/HeiHGM/Bmatching)
[![Heidelberg University](https://img.shields.io/badge/Heidelberg-University-red)](https://www.uni-heidelberg.de)

**Fast, flexible and scalable b-matching in hypergraphs.**

<p align="center">
  <img src="https://raw.githubusercontent.com/HeiHGM/logo/master/heihgm-logo.png" alt="HeiHGM Logo" width="256"/>
</p>

A solver for b-matching problems in hypergraphs, combining graph reductions, integer linear programming, and local search into a flexible algorithm pipeline.

Experiment data is available on [Zenodo](https://doi.org/10.5281/zenodo.18225669). For reproducing paper results, see [reproducibility/reproducibility.md](reproducibility/reproducibility.md).

> **Python Interface:** An easy-to-use Python interface for this software is available in [CHSZLabLib](https://github.com/CHSZLab/CHSZLabLib).

---

## Quick Start

```sh
# Clone & build
git clone https://github.com/HeiHGM/Bmatching.git
cd Bmatching
./compile.sh

# Try it right away on bundled examples
./build/app/bmatching_cli --graph examples/small.hgr --algorithms greedy --capacity 1

# Weighted hypergraph with reductions
./build/app/bmatching_cli --graph examples/weighted.hgr --algorithms reductions,greedy,unfold --capacity 2

# Compact output (weight, size, time only)
./build/app/bmatching_cli --graph examples/weighted.hgr --algorithms reductions,greedy,unfold --quiet
```

---

## Install via Homebrew

```sh
brew install --HEAD HeiHGM/bmatching/bmatching
bmatching --graph examples/weighted.hgr --algorithms greedy --capacity 2 --quiet
```

---

## Building from Source

**Prerequisites:** CMake >= 3.20, a C++17 compiler, and ncurses dev headers.

```sh
./compile.sh            # Release build (default)
./compile.sh Debug      # Debug build
```

Or manually:

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

All external dependencies (Abseil, Protobuf, GoogleTest, Easylogging++, wide-integer) are fetched automatically via CMake's FetchContent.

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `BMATCHING_USE_GUROBI` | OFF | Enable Gurobi ILP solver (requires local install, set `GUROBI_HOME`) |
| `BMATCHING_USE_SCIP` | OFF | Enable SCIP solver |
| `BMATCHING_USE_BSUITOR` | OFF | Enable bSuitor algorithm (fetched via git) |
| `BMATCHING_USE_KARP_SIPSER` | OFF | Enable Karp-Sipser algorithm (fetched via git) |
| `BMATCHING_USE_HASHING` | OFF | Enable edge hashing for Weighted Domination reduction |
| `BMATCHING_USE_TCMALLOC` | OFF | Use tcmalloc from gperftools |
| `BMATCHING_ENABLE_LOGGING` | OFF | Enable easylogging++ log output |
| `BMATCHING_FREE_MEMORY_CHECK` | OFF | Enable free memory checking in runner |
| `BUILD_TESTING` | ON | Build unit tests |

Example with Gurobi and bSuitor enabled:

```sh
cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBMATCHING_USE_GUROBI=ON \
  -DBMATCHING_USE_BSUITOR=ON
cmake --build build -j$(nproc)
```

### Built Binary

After building, the CLI is available at:

```
build/app/bmatching_cli
```

---

## Command-Line Interface

```sh
bmatching_cli --graph <file> --algorithms <algo1,algo2,...> [options]
```

### Required Flags

| Flag | Description |
|------|-------------|
| `--graph <path>` | Path to the hypergraph file |
| `--algorithms <list>` | Comma-separated algorithm pipeline (e.g. `reductions,greedy,unfold`) |

### Common Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--capacity <int>` | 1 | Node capacity (`-1` = use node weights from file) |
| `--format <str>` | `hgr` | Input format: `hgr` or `graph` |
| `--ordering_method <str>` | `bmindegree_dynamic` | Greedy ordering method (see below) |
| `--timeout <float>` | 60.0 | Solver timeout in seconds (ilp_exact, presolved_ilp, scip, local_improvement) |
| `--max_tries <int>` | 1000 | Max iterations for ILS / local search |
| `--iters <int>` | 10 | Local improvement iterations |
| `--distance <int>` | 5 | Edges per local improvement neighborhood |
| `--backend <str>` | `gurobi` | Solver backend for local_improvement: `gurobi` or `scip` |
| `--disable_hint` | false | Don't mark reductions solution as exact |
| `--max_runs <int>` | 10 | Maximum reduction rounds |
| `--reps <int>` | 1 | Number of reduction repetitions |
| `--inplace` | false | Use newer ILS interface internally |

### Output Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--quiet` | false | Only print weight, size, exactness, and time |
| `--output <path>` | stdout | Write result to file |
| `--output_format <str>` | `text` | Output format: `text`, `json`, or `binary` |

---

## Algorithms

Algorithms are chained via `--algorithms` as a comma-separated pipeline. Each algorithm in the pipeline runs in sequence on the same hypergraph and matching state.

### `greedy`

Greedily adds edges to a b-matching using a chosen ordering strategy.

Available `--ordering_method` values:

| Ordering Method | Description |
|-----------------|-------------|
| `default_order` | Iterates over edges in index order, adds all possible edges |
| `bmaximize` | Uses a three-compartment data structure, greedily adds the first free edge |
| `bweight` | Sorts edges by weight, adds in descending order |
| `bratio_static` | Scales edge weight by vertex capacities, sorts descending |
| `bmult_static` | Scales edge by minimal vertex capacity, sorts descending |
| `bratio_dynamic` | Scales by residual capacity, recalculates after each addition |
| `bmindegree_dynamic` | Scales by residual capacity / vertex degree, recalculates dynamically |
| `bmindegree1_dynamic` | Uses product of 1/degree as ordering |

```sh
# Greedy with weight-based ordering
bmatching_cli --graph input.hgr --algorithms greedy --ordering_method bweight --capacity 3

# Greedy with dynamic scaling (default ordering)
bmatching_cli --graph input.hgr --algorithms greedy --capacity 5
```

### `reductions` & `unfold`

Reduces the graph size using b-matching reductions. Always pair with `unfold` afterwards to recover the full solution.

```sh
# Reductions + greedy on the remainder
bmatching_cli --graph input.hgr --algorithms reductions,greedy,unfold --capacity 5

# With extra reduction rounds
bmatching_cli --graph input.hgr --algorithms reductions,greedy,unfold \
  --max_runs 20 --reps 3 --capacity 1
```

### `ils`

Iterated local search: improves an existing solution by searching for beneficial edge swaps, then perturbs to escape local optima. Requires an a priori solution (run greedy first).

```sh
# Greedy + ILS refinement
bmatching_cli --graph input.hgr --algorithms greedy,ils --max_tries 5000 --capacity 1

# Full pipeline: reductions + greedy + ILS + unfold
bmatching_cli --graph input.hgr --algorithms reductions,greedy,ils,unfold \
  --max_tries 10000 --capacity 3

# ILS in-place mode
bmatching_cli --graph input.hgr --algorithms greedy,ils \
  --max_tries 5000 --inplace --capacity 1
```

### `ilp_exact`

Exactly solves the b-matching using Gurobi ILP. Requires `BMATCHING_USE_GUROBI=ON`.

```sh
# Exact solve with 5-minute timeout
bmatching_cli --graph input.hgr --algorithms ilp_exact --timeout 300 --capacity 1

# Reductions first, then exact solve on the remainder
bmatching_cli --graph input.hgr --algorithms reductions,ilp_exact,unfold \
  --timeout 300 --capacity 1
```

### `presolved_ilp`

Solves the reduced (presolved) graph exactly using Gurobi ILP. Designed to run after `reductions`. Requires `BMATCHING_USE_GUROBI=ON`.

```sh
bmatching_cli --graph input.hgr --algorithms reductions,presolved_ilp,unfold \
  --timeout 300 --capacity 1
```

### `scip`

Solves the reduced graph exactly using the SCIP solver. Requires `BMATCHING_USE_SCIP=ON`.

```sh
bmatching_cli --graph input.hgr --algorithms reductions,scip,unfold \
  --timeout 120 --capacity 3
```

### `local_improvement`

Iteratively improves solution quality by solving small ILP subproblems around selected edges. Requires an a priori solution and either Gurobi or SCIP.

```sh
# Local improvement with Gurobi backend
bmatching_cli --graph input.hgr --algorithms reductions,greedy,local_improvement,unfold \
  --backend gurobi --iters 20 --distance 10 --timeout 60 --max_tries 1000 --capacity 1

# Local improvement with SCIP backend
bmatching_cli --graph input.hgr --algorithms reductions,greedy,local_improvement,unfold \
  --backend scip --iters 10 --distance 5 --timeout 30 --max_tries 500 --capacity 3
```

### External Algorithms

For comparison, the runner can link external solvers. Enable at build time:

| Algorithm | Build Flag | Authors | Capacity |
|-----------|------------|---------|----------|
| **bSuitor** | `BMATCHING_USE_BSUITOR=ON` | Khan et al. | any |
| **kss** / **ksmd** | `BMATCHING_USE_KARP_SIPSER=ON` | Dufosse et al. | 1 only |

---

## Output Examples

### Default (text)

```sh
bmatching_cli --graph input.hgr --algorithms greedy --capacity 2
```

Prints the full result as text to stdout.

### Quiet mode

```sh
bmatching_cli --graph input.hgr --algorithms reductions,greedy,unfold --quiet
```

```
weight: 42
size: 15
exact: false
time_ms: 1.234
```

### JSON to file

```sh
bmatching_cli --graph input.hgr --algorithms greedy --output_format json --output result.json
```

### Binary to file

```sh
bmatching_cli --graph input.hgr --algorithms greedy --output_format binary --output result.pb
```

---

## Logging

Logging is disabled by default. To enable: set `BMATCHING_ENABLE_LOGGING=ON` at cmake configure time.

When enabled, control verbosity at runtime:

```sh
bmatching_cli --graph input.hgr --algorithms greedy --undefok=v --v=8
```

| Level | Functions |
|-------|-----------|
| 8 | `addToMatching`, `removeFromMatching` |

---

## Usage with Spack

[Spack](https://spack.io) manages system dependencies on compute clusters:

```sh
git clone --depth=100 --branch=releases/v0.20 https://github.com/spack/spack.git
cd spack && . share/spack/setup-env.sh

cd /path/to/Bmatching
spack env activate .
spack install
spack load
```

Then build as usual.

---

## Citation

If you use this software in your research, please cite:

> Ernestine Großmann, Felix Joos, Henrik Reinstädtler, and Christian Schulz.
> **Engineering Hypergraph *b*-Matching Algorithms.**
> *Journal of Graph Algorithms and Applications (JGAA)*, 30(1):1--24, 2026.
> DOI: [10.7155/jgaa.v30i1.3166](https://doi.org/10.7155/jgaa.v30i1.3166)

```bibtex
@article{GrossmannJRS26,
  author    = {Gro{\ss}mann, Ernestine and Joos, Felix and Reinst{\"a}dtler, Henrik and Schulz, Christian},
  title     = {Engineering Hypergraph $b$-Matching Algorithms},
  journal   = {Journal of Graph Algorithms and Applications},
  volume    = {30},
  number    = {1},
  pages     = {1--24},
  year      = {2026},
  doi       = {10.7155/jgaa.v30i1.3166}
}
```
