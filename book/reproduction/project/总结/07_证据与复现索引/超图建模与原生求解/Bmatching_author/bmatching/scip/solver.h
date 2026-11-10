/**
 * @file ilp_ball.h
 * @author Henrik Reinstädtler (reinstaedtler@stud.uni-heidelberg.de)
 * @brief solve local results by ILP
 * @version 0.1
 *
 * @copyright Copyright (c) 2023 Henrik Reinstädtler
 *
 */
#pragma once

#ifdef HEIPHYPBMATCH_USE_SCIP
#include "scip/scip.h"
#include "scip/scipdefplugins.h"
#endif
#include <map>
#include <sstream>
#include <vector>
/** @brief macro to call scip function with exception handling
 *
 * this macro calls a SCIP function and throws an instance of SCIPException if
 * anything went wrong
 *
 */
#define SCIP_CALL_EXC(x)                                                       \
  {                                                                            \
    SCIP_RETCODE retcode;                                                      \
    if ((retcode = (x)) != SCIP_OKAY) {                                        \
      std::cerr << retcode << std::endl;                                       \
      exit(1);                                                                 \
    }                                                                          \
  }
namespace HeiHGM::BMatching {
namespace bmatching {
namespace scip {

template <class BMatching> class SCIPBMatchingSolver {
#ifdef HEIPHYPBMATCH_USE_SCIP
  SCIP *_scip = 0;
  std::map<typename BMatching::EdgeType, SCIP_VAR *> _vars; // our edges

  std::map<typename BMatching::EdgeType, SCIP_CONS *>
      _cons; // our constraints e.g. nodes
  BMatching &bmatching;
  typename BMatching::Graph_t &hypergraph;

public:
  SCIPBMatchingSolver(BMatching &bm, typename BMatching::Graph_t &g)
      : hypergraph(g), bmatching(bm) { // initialize scip
    SCIP_CALL_EXC(SCIPcreate(&_scip));

    // load default plugins linke separators, heuristics, etc.
    SCIP_CALL_EXC(SCIPincludeDefaultPlugins(_scip));

    // disable scip output to stdout
    // SCIPmessagehdlrSetQuiet(SCIPgetMessagehdlr(_scip), TRUE);

    // create an empty problem
    SCIP_CALL_EXC(SCIPcreateProb(_scip, "hbmatching", NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL));
    SCIP_CALL_EXC(SCIPsetObjsense(_scip, SCIP_OBJSENSE_MAXIMIZE));
    for (auto e : bmatching.free_edges()) {
      SCIP_VAR *var;
      std::stringstream namebuf;
      namebuf << "e" << e;
      // create the SCIP_VAR object
      SCIP_CALL_EXC(SCIPcreateVar(_scip, &var, namebuf.str().c_str(), 0.0, 1.0,
                                  (double)hypergraph.edgeWeight(e),
                                  SCIP_VARTYPE_BINARY, TRUE, FALSE, NULL, NULL,
                                  NULL, NULL, NULL));
      SCIP_CALL_EXC(SCIPaddVar(_scip, var));
      _vars[e] = var;
    }
    // collect constraints
    for (auto n : hypergraph.nodes()) {
      // only constraint at vertex
      if (bmatching.residual_capacity(n) < bmatching.matchableAtNode(n)) {
        std::stringstream namebuf;
        namebuf << "n" << n;
        SCIP_CONS *cons;

        // create SCIP_CONS object
        SCIP_CALL_EXC(SCIPcreateConsLinear(
            _scip, &cons, namebuf.str().c_str(), 0, NULL, NULL, 0.0,
            bmatching.residual_capacity(n), TRUE, TRUE, TRUE, TRUE, TRUE, FALSE,
            FALSE, FALSE, FALSE, FALSE));

        // add the vars belonging to field in this row to the constraint
        for (auto incE : hypergraph.incidentEdges(n)) {
          if (_vars.find(incE) != _vars.end()) {
            SCIP_CALL_EXC(SCIPaddCoefLinear(_scip, cons, _vars[incE], 1.0));
          }
        }
        // add the constraint to scip
        SCIP_CALL_EXC(SCIPaddCons(_scip, cons));

        // store the constraint for later on
        _cons[n] = cons;
      }
    }
  }
  ~SCIPBMatchingSolver() {
    try {
      for (auto &[e, var] : _vars) {
        SCIP_CALL_EXC(SCIPreleaseVar(_scip, &var));
      }
      for (auto &[e, cons] : _cons) {
        SCIP_CALL_EXC(SCIPreleaseCons(_scip, &cons));
      }
      SCIP_CALL_EXC(SCIPfree(&_scip));
    } catch (...) // catch other errors
    {
      // do nothing, but abort in debug mode
      abort();
    }
  }
  bool solve(double timeout = 10) {
    SCIP_CALL_EXC(SCIPsetRealParam(_scip, "limits/time", timeout));

    //    SCIPsetEmphasis(_scip, SCIP_PARAMEMPHASIS_OPTIMALITY, true);
    SCIP_CALL_EXC(SCIPsolve(_scip));
    SCIP_SOL *sol = SCIPgetBestSol(_scip);
    for (auto e : hypergraph.edges()) {
      if (_vars.find(e) != _vars.end()) {
        if (SCIPgetSolVal(_scip, sol, _vars[e]) > 0.5) {
          bmatching.addToMatching(e);
        }
      }
    }
    return SCIP_STATUS_OPTIMAL == SCIPgetStatus(_scip);
  }
#else
public:
  SCIPBMatchingSolver(BMatching &bm, typename BMatching::Graph_t &g) {}
  bool solve(double timout = 0) {
    std::cerr << "Not compiled with scip=enabled. Please recompile"
              << std::endl;
    exit(1);
    return false;
  }
#endif
};
#ifdef HEIPHYPBMATCH_USE_SCIP
template <class BMatching>
void improveMatchingByILP(BMatching &bmatching,
                          typename BMatching::EdgeID_t edge,
                          std::vector<int> &visited_edges,
                          std::vector<int> &prior_edges, int curr_round,
                          unsigned int distance = 5000, double timeout = 50) {
  SCIP *_scip = 0;
  auto &hypergraph = *bmatching._g;
  // do Bfs
  std::vector<typename BMatching::EdgeID_t> edges_list = {edge};
  edges_list.reserve(distance);
  std::vector<typename BMatching::NodeID_t> nodes;
  nodes.reserve(distance * 4);

  visited_edges[edge] = curr_round;
  unsigned int index = 0;
  while (index < edges_list.size() && index < distance) {
    auto curr = edges_list[index];
    visited_edges[curr] = curr_round;
    for (auto p : bmatching._g->pins(curr)) {
      if (prior_edges[p] != curr_round) {
        nodes.push_back(p);
        prior_edges[p] = curr_round;
        for (auto e : bmatching._g->incidentEdges(p)) {
          if (visited_edges[e] != curr_round && !bmatching.isExact(e)) {
            edges_list.push_back(e);
          }
        }
      }
    }
    index++;
  }
  // remove unexplored
  edges_list.resize(index);
  // now add all the pins incident to gorobi
  try {
    SCIP_CALL_EXC(SCIPcreate(&_scip));

    // load default plugins linke separators, heuristics, etc.
    SCIP_CALL_EXC(SCIPincludeDefaultPlugins(_scip));

    // disable scip output to stdout
    SCIPmessagehdlrSetQuiet(SCIPgetMessagehdlr(_scip), TRUE);

    // create an empty problem
    SCIP_CALL_EXC(SCIPcreateProb(_scip, "hbmatching", NULL, NULL, NULL, NULL,
                                 NULL, NULL, NULL));
    SCIP_CALL_EXC(SCIPsetObjsense(_scip, SCIP_OBJSENSE_MAXIMIZE));
    SCIP_CALL_EXC(SCIPsetRealParam(_scip, "limits/time", timeout));

    // initialise edges
    std::map<typename BMatching::EdgeType, SCIP_VAR *> _vars; // our edges

    std::map<typename BMatching::EdgeType, SCIP_CONS *>
        _cons; // our constraints e.g. nodes
    for (auto e : edges_list) {
      SCIP_VAR *var;
      std::stringstream namebuf;
      namebuf << "e" << e;
      // create the SCIP_VAR object
      SCIP_CALL_EXC(SCIPcreateVar(_scip, &var, namebuf.str().c_str(), 0.0, 1.0,
                                  (double)hypergraph.edgeWeight(e),
                                  SCIP_VARTYPE_BINARY, TRUE, FALSE, NULL, NULL,
                                  NULL, NULL, NULL));
      SCIP_CALL_EXC(SCIPaddVar(_scip, var));
      _vars[e] = var;
    }
    for (auto n : nodes) {
      // only constraint at vertex

      std::stringstream namebuf;
      namebuf << "n" << n;
      SCIP_CONS *cons;
      int outside_edges = 0;
      for (auto e : hypergraph.incidentEdges(n)) {
        if (visited_edges[e] != curr_round) {
          outside_edges += bmatching.isInMatching(e);
        }
      }
      // create SCIP_CONS object
      // this is an equality since there must be a queen in every row
      SCIP_CALL_EXC(SCIPcreateConsLinear(
          _scip, &cons, namebuf.str().c_str(), 0, NULL, NULL, 0.0,
          std::max(bmatching.capacity(n) - outside_edges, 0), TRUE, TRUE, TRUE,
          TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE));

      // add the vars belonging to field in this row to the constraint
      for (auto e : hypergraph.incidentEdges(n)) {
        if (visited_edges[e] == curr_round) {
          if (_vars.find(e) != _vars.end()) {
            SCIP_CALL_EXC(SCIPaddCoefLinear(_scip, cons, _vars[e], 1.0));
          }
        }
      }
      // add the constraint to scip
      SCIP_CALL_EXC(SCIPaddCons(_scip, cons));

      // store the constraint for later on
      _cons[n] = cons;
    }

    SCIP_CALL_EXC(SCIPsolve(_scip));
    SCIP_SOL *sol = SCIPgetBestSol(_scip);

    if (SCIP_STATUS_OPTIMAL == SCIPgetStatus(_scip)) {
      // add to bmatching
      for (auto e : edges_list) {
        if (SCIPgetSolVal(_scip, sol, _vars[e]) < 0.5 &&
            bmatching.isInMatching(e)) {
          bmatching.removeFromMatching(e);
        }
      }
      for (auto e : edges_list) {
        if (SCIPgetSolVal(_scip, sol, _vars[e]) > 0.5 &&
            !bmatching.isInMatching(e)) {
          bmatching.addToMatching(e);
        }
      }
    } else {
      std::cout << "No optimal solution" << std::endl;
      // add to bmatching
      for (auto e : edges_list) {
        if (SCIPgetSolVal(_scip, sol, _vars[e]) < 0.5 &&
            bmatching.isInMatching(e)) {
          bmatching.removeFromMatching(e);
        }
      }
      for (auto e : edges_list) {
        if (SCIPgetSolVal(_scip, sol, _vars[e]) > 0.5 &&
            !bmatching.isInMatching(e)) {
          bmatching.addToMatching(e);
        }
      }
    }
    bmatching.maximize();
    for (auto &[e, var] : _vars) {
      SCIP_CALL_EXC(SCIPreleaseVar(_scip, &var));
    }
    for (auto &[e, cons] : _cons) {
      SCIP_CALL_EXC(SCIPreleaseCons(_scip, &cons));
    }
    SCIP_CALL_EXC(SCIPfree(&_scip));
  } catch (...) {
    std::cout << "SCIP exception occurred. " << std::endl;
  }
  if (!bmatching.bMatching()) {
    std::cout << "WARN bmatching invalid." << std::endl;
  }
}
#endif
template <class BMatching>
void iterated_local_search_ilp(BMatching &bmatching, unsigned int iters,
                               unsigned int distance, double timeout,
                               const int max_tries) {
#ifdef HEIPHYPBMATCH_USE_SCIP
  // we reuse the memory, store in which round an edge was touched by bfs
  std::vector<int> visited_edges(bmatching._g->realEdgeSize(), -1);
  std::vector<int> prior_edges(bmatching._g->initialNumNodes(), -1);
  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();
  int tries = 0;
  while (tries < max_tries) {
    for (unsigned int i = 0; i < iters; i++) {
      tries++;
      if (tries >= max_tries) {
        break;
      }
      auto prior_weight = bmatching.weight();
      improveMatchingByILP(bmatching, bmatching.randomEdgeNonExact(),
                           visited_edges, prior_edges, i, distance, timeout);
      if (bmatching.weight() > prior_weight) {
        tries = 0;
      }
      if (tries > max_tries) {
        break;
      };
    }
  }
#else
  std::cerr << "Please recompile with --define scip=enabled" << std::endl;
  exit(-1);
#endif
}
} // namespace scip
} // namespace bmatching
} // namespace HeiHGM::BMatching