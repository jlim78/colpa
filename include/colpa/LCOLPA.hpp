/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#ifndef LCOLPA_LCOLPA_HPP_
#define LCOLPA_LCOLPA_HPP_

// STL headers
#include <exception>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>
#include <chrono> // For high resolution clock

// Timed Callback
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>

// OMPL headers
#include <ompl/base/Planner.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/geometric/PathGeometric.h>
// For geometric equations like unitNBallMeasure
#include <ompl/util/GeometricEquations.h>
// For halton sequence real vector bounds
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>

// LCOLPA headers. Include all the headers.
#include "colpa/datastructures.hpp"
#include "colpa/event.hpp"
#include "colpa/io.hpp"
// For Halton sequence sample in n-dimension
#include "colpa/sampler/HaltonSequence.hpp"

namespace colpa {

enum PlannerStatus { Solved, NotSolved };

/// The OMPL Planner class that implements the algorithm.
class LCOLPA : public ompl::base::Planner {
public:
  /// Constructor.
  /// \param[in] si The OMPL space information manager.
  explicit LCOLPA(const ompl::base::SpaceInformationPtr& si);

  /// Destructor.
  ~LCOLPA(void);

  /// Callback for visualization
  void setDebugCallback(std::function<void(colpa::datastructures::Graph g)> callback);
  std::function<void(colpa::datastructures::Graph g)> mCallback;

  /// Setup the planner.
  void setup() override;

  /// Set the problem definition and define the start, goal.
  /// \param[in] pdef OMPL Problem Definition.
  void setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) override;

  /// Solve the planning problem.
  /// \param[in] ptc OMPL Planning Termination Condition.
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition& ptc);

  /// Clear the planner setup.
  void clear() override;

  /// Set the event to be used by LCOLPA.
  /// \param[in] event Event that defines the trigger condition.
  void setEvent(colpa::event::EventPtr event);

  /// Returns the event used by the algorithm.
  colpa::event::ConstEventPtr getEvent() const;

  /// Set the connection radius of the graph.
  void setConnectionRadius(double radius);

  /// Get the connection radius of the graph.
  double getConnectionRadius();

  /// Set the collision checking resolution along the edge.
  void setCollisionCheckResolution(double resolution);

  /// Get the connection radius of the graph.
  double getCollisionCheckResolution();

  /// Set the roadmap. Loads the graph.
  void setRoadmap(std::string filename);

  /// Setup kNearestNeighbor
  void setupKNN();

  /// Set the best path cost.
  void setBestPathCost(colpa::datastructures::PathClassType cost);

  /// Get the best path cost.
  colpa::datastructures::PathClassType getBestPathCost();

  /// Set status of the planner.
  void setPlannerStatus(PlannerStatus);

  /// Get status of the planner.
  PlannerStatus getPlannerStatus();

  /// Get the number of edges evaluated.
  double getNumberOfEdgeEvaluations();

  /// Get the number of vertex expansion.
  double getNumberOfVertexExpansions();

  /// Evaluate all vertices to identify the differences next time
  void perceiveNewWorld();

  /// Identify all changed vertices and update their edges, then updateVertex
  bool perceiveChanges();

  /// Sample a rectangle between start and goal using Halton sampling
  void generateNewSamples(int batchSize, bool updateVertices);

  /// Generate a halton sample at the given index.
  std::vector<double> haltonSample(std::size_t index) const;

  /// For visuaization of graph
  colpa::datastructures::Graph getGraph(){return mGraph;};

  std::vector<colpa::datastructures::Edge> getPerceivedChangedEdges(){return mPerceivedChangedEdges;};

  void setStateClassifier(std::function<int(const ompl::base::State*)> classifier);

private:
  /// Adds source and target vertices, and relevant edges to \c mGraph.
  /// Sets up the event and the selector.
  void setupPreliminaries();

  /// Returns edge between source and target vertices.
  colpa::datastructures::Edge getEdge(colpa::datastructures::Vertex, colpa::datastructures::Vertex);

  /// Returns the path from vertex to source.
  colpa::datastructures::Path getPathToSource(colpa::datastructures::Vertex);

  /// Heuristic function.
  double getGraphHeuristic(colpa::datastructures::Vertex v);

  /// Heuristic class
  colpa::datastructures::PathClassType getGraphHeuristicClass(colpa::datastructures::Vertex v);

  /// Evaluates an edge for collision.
  int evaluateVertex(const colpa::datastructures::Vertex& v);

  /// Evaluates an edge for collision.
  int evaluateEdge(const colpa::datastructures::Edge& e);

  /// Calculate Keys for vertex
  colpa::datastructures::Keys calculateKeys(colpa::datastructures::Vertex v);

  /// Update Vertex find a best parent
  void updateVertex(colpa::datastructures::Vertex v);

  /// Compute shortest path
  bool computeShortestPath(colpa::datastructures::Vertex& leafVertex);

  /// Evaluates the search tree when the extension pauses.
  bool evaluatePath(const colpa::datastructures::Vertex& triggeredLeaf, colpa::datastructures::Edge& evaluatedEdge);

  /// Return the path from source to target vertices.
  ompl::base::PathPtr constructSolution(
      const colpa::datastructures::Vertex&, const colpa::datastructures::Vertex&);

  /// Calculate the neighoring radius depending on the current number of samples
  double calculateR() const;

  /// The pointer to the OMPL state space.
  const ompl::base::StateSpacePtr mSpace;

  /// Boost roadmap representation.
  boost::shared_ptr<io::RoadmapFromFile<
      colpa::datastructures::Graph,
      colpa::datastructures::VPStateMap,
      colpa::datastructures::State,
      colpa::datastructures::EPLengthMap>>
      mRoadmap;

  /// Connection radius in the graph.
  double mConnectionRadius;

  /// Collision checking resolution for the edge.
  double mCollisionCheckResolution;

  /// Boolean denoting if the graph has been setup.
  bool mGraphSetup{false};

  /// Best path found so far
  colpa::datastructures::PathClassType mBestPathCost{std::numeric_limits<float>::infinity()};


  /// Flag to check if the planner succeeded.
  PlannerStatus mPlannerStatus{PlannerStatus::NotSolved};

  /// Queue representing the open list to extend.
  colpa::datastructures::Queue mQueue;

  /// Event
  colpa::event::EventPtr mEvent;

  /// The fixed roadmap over which the search is done.
  colpa::datastructures::Graph mGraph;

  /// Source vertex.
  colpa::datastructures::Vertex mSourceVertex;

  /// Target vertex.
  colpa::datastructures::Vertex mTargetVertex;

  /// KNN Structure
  /// NOTE: we use this datastructure for nearestR()
  ompl::NearestNeighborsGNAT<colpa::datastructures::Vertex> knnGraph ;

  /// Dist Function for KNN
  double distFun(const colpa::datastructures::Vertex& v1, const colpa::datastructures::Vertex& v2);

  /// Halton sampler
  std::shared_ptr<ompl::base::HaltonSequence> mHaltonSequence;

  /// constant used for calculateR
  double mRewireFactor{1.1};

  /// TODO (avk): Move these into PlannerStatus class.
  /// Number of Edge Evaluations.
  double mNumberOfEdgeEvaluations{0};

  /// Number of Vertex Expansions
  double mNumberOfVertexExpansions{0};

  /// Debug callback setup
  bool mDebugging{false};

  /// Perceived changed edges
  std::vector<colpa::datastructures::Edge> mPerceivedChangedEdges;

  /// State Classifier
  std::function<int(const ompl::base::State*)> mStateClassifier;

  /// For timing
  double mTotalEdgeEvaluationTime{0};
  double mTotalVertexExpansionTime{0};

  bool mJoined{false};
  bool mDraw{false};
  void call_visualize();

  std::mutex mtx;
  std::condition_variable cv{};
  std::chrono::microseconds time{100};
};

} // namespace colpa

#endif // LCOLPA_LCOLPA_HPP_
