/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#ifndef COLPA_DATASTRUCTURES_GRAPH_HPP_
#define COLPA_DATASTRUCTURES_GRAPH_HPP_

// STL headers
#include <set>
#include <vector>

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

// colpa headers
//#include "colpa/datastructures/PathClass.hpp"
#include "colpa/datastructures/State.hpp"
#include "colpa/datastructures/Types.hpp"

// TODO (avk): state and length are made public to accomodate the
// roadmapmanager which seems stupid. Change it if possible.

namespace colpa {
namespace datastructures {

enum EvaluationStatus { NotEvaluated, Evaluated };

class VertexProperties {
public:
  // Set state wrapper around underlying OMPL state.
  void setState(StatePtr state);

  // Get state wrapper around underlying OMPL state.
  StatePtr getState();

  // Set G class
  void setGclass(PathClassType g_class);

  // get G class
  PathClassType getGclass() ;

  // set RHS class
  void setRHSclass(PathClassType rhs_class) ;

  // Get RHS class
  PathClassType getRHSclass() ;

  // Set H class
  void  setHclass(PathClassType h_class) ;

  // Get H class
  PathClassType getHclass();


  // Set cost-to-come.
  void setCostToCome(double cost);

  // Get cost-to-come.
  double getCostToCome();

  // Set cost-to-come rhs-value.
  void setRHS(double cost);

  // Get cost-to-come rhs-value.
  double getRHS();

  // Set heuristic.
  void setHeuristic(double heuristic);

  // Get heuristic.
  double getHeuristic();

  // Get estimated total cost.
  double getEstimatedTotalCost();

  // Set the vertex parent.
  void setParent(Vertex parent);

  // Get the vertex parent.
  Vertex getParent();

  // Get true if parent exists
  bool hasParent();

  // Removes parent, change hasParent to False
  void removeParent();

  // Set the color status of the vertex.
  void setColor(int color);

  // Get the color status of the vertex.
  int getColor();

  /// Underlying state.
  /// TODO (avk): why is this public?
  StatePtr mState;

private:
  /// Cost-to-Come g-value.
  double mCostToCome{std::numeric_limits<double>::infinity()};// Depreciated

  /// Cost-to-Come rhs-value.
  double mRHS{std::numeric_limits<double>::infinity()};// Depreciated

  PathClassType mRHSclass{std::numeric_limits<double>::infinity()};

  PathClassType mGclass{std::numeric_limits<double>::infinity()};

  PathClassType mHclass{std::numeric_limits<double>::infinity()};


  /// Heuristic value.
  double mHeuristic{std::numeric_limits<double>::infinity()};// Depreciated

  /// Parent.
  Vertex mParent{boost::graph_traits<BasicGraph>::null_vertex()};

  /// Color status.
  int mColor{-1};
};

class EdgeProperties {
public:
  // Sets the length of the edge. As Heuristic Distance
  void setLength(double length);

  // Get the length of the edge.
  double getLength();

  /// Sets the value of the edge.
  void setValue(double value);

  /// Get the value of the edge.
  double getValue();

  /// Get the value of the edge class.
  PathClassType getWclass();

  // Sets the evaluation status.
  void setEvaluationStatus(EvaluationStatus evaluationStatus);

  // Get the evaluation status.
  EvaluationStatus getEvaluationStatus();

  // Sets the color status.
  void setColor(int color);

  // Get the color status.
  int getColor();

  /// The length of the edge using the space distance metric.
  /// TODO (avk): Why is this public?
  double mLength;

private:
  /// Edge Value.
  double mValue;

  bool mValueEvaluated{false};

  /// Evaluation status.
  EvaluationStatus mEvaluationStatus{EvaluationStatus::NotEvaluated};

  /// Color status.
  int mColor{-1};

};

/// Undirected Boost graph using the properties just defined.
typedef boost::
    adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties>
        Graph;

/// Shared pointer to Graph.
typedef std::shared_ptr<Graph> GraphPtr;

/// Shared pointer to const Graph.
typedef std::shared_ptr<const Graph> ConstGraphPtr;

/// Boost vertex iterator
typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;

/// Boost edge iterator
typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;

/// Boost graph neighbor iterator
typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

/// Map each vertex to the underlying state [read from the graphml file]
typedef boost::property_map<Graph, colpa::datastructures::StatePtr VertexProperties::*>::type
    VPStateMap;

/// Map each edge to its length
typedef boost::property_map<Graph, double EdgeProperties::*>::type EPLengthMap;

} // namespace datastructures
} // namespace colpa

#endif // COLPA_DATASTRUCTURES_GRAPH_HPP_
