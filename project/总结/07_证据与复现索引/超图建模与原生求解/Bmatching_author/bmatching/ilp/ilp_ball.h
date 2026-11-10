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
#include <vector>

#ifdef USE_GUROBI_ENABLED
#include "gurobi_c++.h"
#else
typedef int GRBEnv;
typedef int GRBVar;
typedef int GRBModel;
#endif
namespace HeiHGM::BMatching {
namespace bmatching {
namespace ilp {
template <class BMatching>
void improveMatchingByILP(BMatching &bmatching,
                          typename BMatching::EdgeID_t edge, GRBEnv *env,
                          std::vector<int> &visited_edges,
                          std::vector<int> &prior_edges,
                          std::vector<GRBVar> &edges, int curr_round,
                          unsigned int distance = 5000) {
#ifdef USE_GUROBI_ENABLED
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
    GRBModel model = GRBModel(*env);
    // GRBVar* nodes = new GRBVar [graph->initialNumNodes()];
    // std::cout<<"Edges: "<<edges_list.size()<<" Nodes:
    // "<<nodes.size()<<std::endl; std::cout<<"[Before] Size:
    // "<<bmatching.size()<<" Weight: "<<bmatching.weight()<<std::endl;
    model.set(GRB_StringAttr_ModelName, "Matching");
    model.set(GRB_DoubleParam_TimeLimit, 50); // TODO(Henrik): Replace
    model.set(GRB_DoubleParam_MIPGap, 0);
    model.set(GRB_IntParam_Threads, 1);

    // initialise edges
    GRBLinExpr edgeWeightSum = 0;
    for (auto e : edges_list) {
      edges[e] = model.addVar(0.0, 1.0, 0, GRB_BINARY);
      edges[e].set(GRB_DoubleAttr_Start, bmatching.isInMatching(e));
      edgeWeightSum += bmatching._g->edgeWeight(e) * edges[e];
    }
    for (auto v : nodes) {
      GRBLinExpr nodeLimit = 0;
      int outside_edges = 0;
      for (auto e : bmatching._g->incidentEdges(v)) {
        if (visited_edges[e] == curr_round) {
          nodeLimit += edges[e];
        } else {
          outside_edges += bmatching.isInMatching(e);
        }
      }
      model.addConstr(nodeLimit <=
                          std::max(bmatching.capacity(v) - outside_edges, 0),
                      "valid bmatching");
    }
    model.setObjective(edgeWeightSum, GRB_MAXIMIZE);
    //// Optimize model
    model.optimize();
    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL ||
        model.get(GRB_IntAttr_Status) == GRB_TIME_LIMIT) {
      // add to bmatching
      for (auto e : edges_list) {
        auto val = edges[e].get(GRB_DoubleAttr_X);
        if (val < 0.5 && bmatching.isInMatching(e)) {
          bmatching.removeFromMatching(e);
        }
      }
      for (auto e : edges_list) {
        auto val = edges[e].get(GRB_DoubleAttr_X);
        if (val > 0.5 && !bmatching.isInMatching(e)) {
          bmatching.addToMatching(e);
        }
      }
    } else {
      std::cout << "No solution" << std::endl;
      // add to bmatching
      for (auto e : edges_list) {
        auto val = edges[e].get(GRB_DoubleAttr_X);
        if (val < 0.5 && bmatching.isInMatching(e)) {
          bmatching.removeFromMatching(e);
        }
      }
      for (auto e : edges_list) {
        auto val = edges[e].get(GRB_DoubleAttr_X);
        if (val > 0.5 && !bmatching.isInMatching(e)) {
          bmatching.addToMatching(e);
        }
      }
    }
    bmatching.maximize();
    // std::cout<<"[After]  Size: "<<bmatching.size()<<" Weight:
    // "<<bmatching.weight()<<std::endl;

    // delete[] nodes;
    // delete[] edges;
  } catch (GRBException e) {
    std::cout << "Gurobi exception occurred:"
              << " ERROR " << e.getErrorCode() << ". " << std::endl;
    exit(EXIT_FAILURE);
  }
#else
  std::cerr << "Please recompile with gurobi=enabled" << std::endl;
  exit(-1);
#endif
}
template <class BMatching>
void iterated_local_search_ilp(BMatching &bmatching, unsigned int iters,
                               unsigned int distance, double timeout) {
#ifdef USE_GUROBI_ENABLED
  auto env = new GRBEnv("grb.log");
  env->set(GRB_IntParam_OutputFlag, 0);
  // we reuse the memory, store in which round an edge was touched by bfs
  std::vector<int> visited_edges(bmatching._g->realEdgeSize(), -1);
  std::vector<int> prior_edges(bmatching._g->initialNumNodes(), -1);
  std::vector<GRBVar> edges(bmatching._g->realEdgeSize());
  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();
  for (unsigned int i = 0; i < iters; i++) {
    improveMatchingByILP(bmatching, bmatching.randomEdgeNonExact(), env,
                         visited_edges, prior_edges, edges, i, distance);
    std::chrono::high_resolution_clock::time_point t2 =
        std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count() > timeout) {
      break;
    };
  }
  delete env;
#else
  std::cerr << "Please recompile with -DUSE_GUROBI_ENABLED=1" << std::endl;
  exit(-1);
#endif
}
template <class BMatching>
void long_iterated_local_search_ilp(BMatching &bmatching,
                                    unsigned int distance) {
#ifdef USE_GUROBI_ENABLED
  auto env = new GRBEnv("grb.log");
  env->set(GRB_IntParam_OutputFlag, 0);
  // we reuse the memory, store in which round an edge was touched by bfs
  std::vector<int> visited_edges(bmatching._g->realEdgeSize(), -1);
  std::vector<int> prior_edges(bmatching._g->initialNumNodes(), -1);
  GRBVar *edges = new GRBVar[bmatching._g->realEdgeSize()];
  auto oldvalue = bmatching.weight();
  int i = 0;
  do {
    oldvalue = bmatching.weight();
    for (auto e : bmatching._g->edges()) {
      improveMatchingByILP(bmatching, e, env, visited_edges, prior_edges, edges,
                           i++, distance);
    }
    std::cout << oldvalue << " < " << bmatching.weight() << std::endl;
  } while (oldvalue != bmatching.weight());
  delete env;
#else
  std::cerr << "Please recompile with -DUSE_GUROBI_ENABLED=1" << std::endl;
  exit(-1);
#endif
}
/**
 * @brief Computes a matching in  the rest of the graph
 *
 * @tparam BMatching
 * @param graph
 * @param bmatching
 * @param optimal
 * @param timeout
 * @return BMatching
 */
template <class BMatching>
void computePresolvedIlp(typename BMatching::Graph_t *graph,
                         BMatching &bmatching, bool &optimal,
                         double timeout = 1000.0) {
#ifdef USE_GUROBI_ENABLED
  try {
    GRBEnv *env = new GRBEnv();
    env->set(GRB_IntParam_OutputFlag, 0);
    GRBModel model = GRBModel(*env);
    // GRBVar* nodes = new GRBVar [graph->initialNumNodes()];
    GRBVar *edges = new GRBVar[graph->realEdgeSize()];
    model.set(GRB_StringAttr_ModelName, "Matching");
    model.set(GRB_DoubleParam_TimeLimit, timeout);
    model.set(GRB_DoubleParam_MIPGap, 0);
    model.set(GRB_IntParam_Threads, 1);

    // initialise edges
    GRBLinExpr edgeWeightSum = 0;
    for (auto e : bmatching.free_edges()) {
      if (bmatching.isMatchable(e) && graph->edgeIsEnabled(e)) {
        edges[e] = model.addVar(0.0, 1.0, 0, GRB_BINARY);
        edges[e].set(GRB_DoubleAttr_Start, 0);
        edgeWeightSum += graph->edgeWeight(e) * edges[e];
      }
    }
    for (auto v : graph->nodes()) {
      GRBLinExpr nodeLimit = 0;
      for (auto e : graph->incidentEdges(v)) {
        if (bmatching.isMatchable(e) && graph->edgeIsEnabled(e)) {
          nodeLimit += edges[e];
        }
      }
      model.addConstr(nodeLimit <=
                          (bmatching.capacity(v) - bmatching.matchesAtNode(v)),
                      "valid bmatching");
    }
    model.setObjective(edgeWeightSum, GRB_MAXIMIZE);
    //// Optimize model
    model.optimize();
    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
      optimal = true;
      // add to bmatching
      for (auto e : graph->edges()) {
        if (bmatching.isMatchable(e) && graph->edgeIsEnabled(e)) {
          auto val = edges[e].get(GRB_DoubleAttr_X);
          if (val > 0.5)
            bmatching.addToMatching(e);
        }
      }
    } else {
      optimal = false;
      std::cout << "No solution" << std::endl;
      for (auto e : graph->edges()) {
        if (bmatching.isMatchable(e) && graph->edgeIsEnabled(e)) {
          auto val = edges[e].get(GRB_DoubleAttr_X);
          if (val > 0.5)
            bmatching.addToMatching(e);
        }
      }
    }
    // delete[] nodes;
    delete[] edges;
    delete env;
  } catch (GRBException e) {
    std::cout << "Gurobi exception occurred:"
              << " ERROR " << e.getErrorCode() << ". " << std::endl;
    exit(EXIT_FAILURE);
  }
#else
  std::cerr << "Please recompile with -DUSE_GUROBI_ENABLED=1" << std::endl;
  exit(-1);
#endif
}
} // namespace ilp
} // namespace bmatching
} // namespace HeiHGM::BMatching