/**
 * @file ilp_exact.h
 * @author Henrik Reinstädtler (reinstaedtler@stud.uni-heidelberg.de)
 * @brief ILP to solve for Hypergraph b-matching
 * @version 0.1
 *
 * @copyright Copyright (c) 2023 Henrik Reinstädtler
 *
 */
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
void computeIlp(typename BMatching::Graph_t *graph, BMatching &bmatching,
                bool &optimal, double timeout = 7200.0) {
#ifdef USE_GUROBI_ENABLED

  try {
    GRBEnv *env = new GRBEnv();
    env->set(GRB_IntParam_OutputFlag, 0);
    GRBModel model = GRBModel(*env);

    std::cout << "Allocating " << graph->realEdgeSize() << std::endl;
    GRBVar *edges = new GRBVar[graph->realEdgeSize()];
    model.set(GRB_StringAttr_ModelName, "Matching");
    model.set(GRB_DoubleParam_TimeLimit, timeout);
    model.set(GRB_DoubleParam_MIPGap, 0);
    model.set(GRB_IntParam_Threads, 1);

    GRBLinExpr edgeWeightSum = 0;
    for (auto e : graph->edges()) {
      edges[e] = model.addVar(0.0, 1.0, 0, GRB_BINARY);
      edges[e].set(GRB_DoubleAttr_Start, bmatching.isInMatching(e));
      edgeWeightSum += graph->edgeWeight(e) * edges[e];
    }
    for (auto v : graph->nodes()) {
      GRBLinExpr nodeLimit = 0;
      for (auto e : graph->incidentEdges(v)) {
        nodeLimit += edges[e];
      }
      model.addConstr(nodeLimit <= bmatching.capacity(v), "valid bmatching");
    }
    model.setObjective(edgeWeightSum, GRB_MAXIMIZE);
    //// Optimize model
    model.optimize();
    bmatching.reset();

    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
      optimal = true;
      // add to bmatching
      for (auto e : graph->edges()) {
        auto val = edges[e].get(GRB_DoubleAttr_X);
        if (val > 0.5)
          bmatching.addToMatching(e);
      }
    } else {
      optimal = false;
      std::cout << "No solution" << std::endl;
      for (auto e : graph->edges()) {
        auto val = edges[e].get(GRB_DoubleAttr_X);
        if (val > 0.5)
          bmatching.addToMatching(e);
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
  std::cerr << "Please recompile with gurobi=enabled" << std::endl;
  exit(-1);
#endif
}
} // namespace ilp
} // namespace bmatching
} // namespace HeiHGM::BMatching