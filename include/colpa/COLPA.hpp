/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#ifndef COLPA_COLPA_HPP_
#define COLPA_COLPA_HPP_

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
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/geometric/PathGeometric.h>
// For geometric equations like unitNBallMeasure
#include <ompl/util/GeometricEquations.h>
// For halton sequence real vector bounds
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>

// COLPA headers. Include all the headers.
#include "colpa/datastructures.hpp"
#include "colpa/io.hpp"
// For Halton sequence sample in n-dimension
#include "colpa/sampler/HaltonSequence.hpp"

namespace colpa {

enum PlannerStatus { Solved, NotSolved };

/// The OMPL Planner class that implements the algorithm.
class COLPA : public ompl::base::Planner {
public:
  /// Constructor.
  /// \param[in] si The OMPL space information manager.
  explicit COLPA(const ompl::base::SpaceInformationPtr& si);

  /// Destructor.
  ~COLPA(void);

  /// Setup the planner.
  void setup() override;

  /// Set the problem definition and define the start, goal.
  /// \param[in] pdef OMPL Problem Definition.
  void setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) override;

  /// Clear the planner setup.
  void clear() override;

  void freshStart();
  /// Solve the planning problem.
  /// \param[in] ptc OMPL Planning Termination Condition.
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition& ptc) override;

  ompl::base::PlannerStatus solve_once();
  /// Set the K neighbors for all nodes.
  void setKNeighbors(int num_neighbors);

  /// Set the K neighbors for connecting target.
  void setGoalKNeighbors(int num_neighbors);

  /// Set number of samples to take per sample call.
  void setSampleMultiplier(double sample_mul);

  /// Set Buffer size.
  void setSampleBufferSize(double buffer);

  /// Set the connection radius of the graph.
  void setConnectionRadius(double radius);

  /// Get mKNeighbors.
  int getKNeighbors();

  /// Get mKNeighbors.
  int getGoalKNeighbors();

  /// Get number of samples to take per sample call.
  double getSampleMultiplier();

  /// Get Buffer size.
  double getSampleBufferSize();

  /// Get the connection radius of the graph.
  double getConnectionRadius();

  /// Set the collision checking resolution along the edge.
  void setCollisionCheckResolution(double resolution);

  /// Get the connection radius of the graph.
  double getCollisionCheckResolution();

  /// Set the roadmap. Loads the graph.
  void setRoadmap(std::string filename);

  /// Set the best path cost.
  // void setBestPathCost(double cost);
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

  // Generate uniform sampled graph within a current space bound
  void generateUniformSamples(int batchSize,bool updateVertices);

  /// Sample a rectangle between start and goal using Halton sampling
  void generateNewSamples(int batchSize, bool updateVertices);

  /// Generate a graph, using samples
  void generateGraph(std::vector<std::pair<double,double>>& states);

  /// Generate a halton sample at the given index.
  std::vector<double> haltonSample(std::size_t index) const;


  /// Get timing info
  double getEdgeEvalTime() {return mTotalEdgeEvaluationTime;};

  /// Get timing info
  double getVertexExpTime() {return mTotalVertexExpansionTime;};

  int getGraphSize() {return knnGraph.size();};

  // for getGraph
  typedef std::tuple<double, double, double, bool> Node;

  /// Get graph
  void getGraph(std::vector<Node>* nodes, std::vector<std::tuple<Node, Node>>* edges);

  /// Get graph & path
  void getGraphWithPaths(
      std::vector<Node>* nodes,
      std::vector<std::tuple<Node, Node>>* edges,
      std::vector<std::vector<Node>>* paths);

  std::vector<colpa::datastructures::Edge> getPerceivedChangedEdges(){return mPerceivedChangedEdges;};

  void setStateClassifier(std::function<int(const ompl::base::State*)> classifier);

  /// Aborts planning
  void abortPlanning();

  /// Set Space Information (needed for dynamic collision checking)
  void setSpaceInformation(const ompl::base::SpaceInformationPtr& si);

  void setPreviousPlanInvalid() {
    mBestPathCost = colpa::datastructures::PathClassType(std::numeric_limits<double>::max());
  }

  /// Sample a rectangle between start and goal using Halton sampling
  void generateNewSamples(double sample_multiplier, double buffer);

  void generateNewSamples(double sample_multiplier, double buffer, bool updateVertices);

  void generateRectangleSamples(double sample_multiplier, double buffer, int minBatchSize, bool updateVertices);

  /// For visuaization of graph
  colpa::datastructures::Graph getGraph(){return mGraph;};
  void setDebugCallback(std::function<void(colpa::datastructures::Graph g)> callback);
  std::function<void(colpa::datastructures::Graph g)> mCallback;
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
  void computeShortestPath();

  /// Return the path from source to target vertices.
  ompl::base::PathPtr constructSolution(
      const colpa::datastructures::Vertex&, const colpa::datastructures::Vertex&);

  /// Calculate the neighoring radius depending on the current number of samples
  double calculateR() const;

  /// Wraps yaw to -pi, pi
  double wrapAngle(const colpa::datastructures::StatePtr stateptr);

  /// Checks if vertex is in state bounds
  bool inBounds(colpa::datastructures::Vertex v);
  /// The pointer to the OMPL state space.
  ompl::base::StateSpacePtr mSpace;

  /// Boost roadmap representation.
  boost::shared_ptr<io::RoadmapFromFile<
      colpa::datastructures::Graph,
      colpa::datastructures::VPStateMap,
      colpa::datastructures::State,
      colpa::datastructures::EPLengthMap>>
      mRoadmap;

  /// Connection radius in the graph.
  double mConnectionRadius;

  /// Number of K nearest neighbors to connect nodes.
  int mKNeighbors;

  /// Number of K nearest neighbors to connect to target.
  int mGoalKNeighbors;

  /// Number of times sampled called in a plan call.
  int mNumSampleCalls;

  /// Distance to increase all sides of rectangle on resample.
  double mSampleBufferSize;

  /// mSampleMultiplier * distance_between_start_goal = Num Samples
  double mSampleMultiplier;

  /// Collision checking resolution for the edge.
  double mCollisionCheckResolution;

  /// Boolean denoting if the graph has been setup.
  bool mGraphSetup{false};

  ///  Best path cost so far found.
  colpa::datastructures::PathClassType mBestPathCost{std::numeric_limits<double>::infinity()};

  /// Whether or not to stop planning.
  bool mPreempt;

  /// Flag to check if the planner succeeded.
  PlannerStatus mPlannerStatus{PlannerStatus::NotSolved};

  /// Queue representing the inconsistent vertices to expand.
  colpa::datastructures::Queue mQueue;

  /// The fixed roadmap over which the search is done.
  colpa::datastructures::Graph mGraph;

  /// Source vertex.
  colpa::datastructures::Vertex mSourceVertex;

  /// Target vertex.
  colpa::datastructures::Vertex mTargetVertex;

  /// Dist Function for KNN
  double distFun(const colpa::datastructures::Vertex& v1, const colpa::datastructures::Vertex& v2);

  /// KNN Structure
  /// NOTE: we use this datastructure for nearestR()
  ompl::NearestNeighborsGNAT<colpa::datastructures::Vertex> knnGraph ;

  /// Setup kNearestNeighbor
  void setupKNN();

  /// Halton sampler
  std::shared_ptr<ompl::base::HaltonSequence> mHaltonSequence;

  double mRewireFactor{1.1};

  /// TODO (avk): Move these into PlannerStatus class.
  /// Number of Edge Evaluations.
  double mNumberOfEdgeEvaluations{0};

  /// Number of Vertex Expansions
  double mNumberOfVertexExpansions{0};

  bool mDebugging{false};

  /// Index to sample Halton points at.
  std::size_t mHaltonIndex{0};

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

#endif // COLPA_COLPA_HPP_
