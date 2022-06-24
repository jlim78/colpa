/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#include "colpa/COLPA.hpp"

#include <cmath>    // pow, sqrt
#include <iostream> // std::invalid_argument
#include <set>      // std::set
#include <assert.h> // debug

#include <Eigen/Geometry>
#include <boost/graph/connected_components.hpp> // connected_components

using colpa::datastructures::PathClassType;
using colpa::datastructures::Keys;
using colpa::datastructures::Edge;
using colpa::datastructures::EdgeIter;
using colpa::datastructures::EdgeProperties;
using colpa::datastructures::EPLengthMap;
using colpa::datastructures::EvaluationStatus;
using colpa::datastructures::Graph;
using colpa::datastructures::NeighborIter;
using colpa::datastructures::Path;
using colpa::datastructures::State;
using colpa::datastructures::StatePtr;
using colpa::datastructures::Vertex;
using colpa::datastructures::VertexIter;
using colpa::datastructures::VertexProperties;
using colpa::datastructures::VPStateMap;


namespace colpa {

COLPA::COLPA(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::Planner(si, "COLPA"), mSpace(si->getStateSpace()) {
  // Set default values for data members.

  // Halton sequence sampler for n dimensional search
  mHaltonSequence = std::make_shared<ompl::base::HaltonSequence>(si->getStateDimension());
}

COLPA::~COLPA() {
  // Do nothing.
}

// ============================================================================
void COLPA::setup() {
  // Check if already setup.
  if (static_cast<bool>(ompl::base::Planner::setup_))
    return;

  // Mark the planner to have been setup.
  ompl::base::Planner::setup();

  // Check if the graph has been setup.
  if (!mGraphSetup)
    std::invalid_argument("Graph has not been provided.");

  // TODO (avk): If the graph is not provided, use implicit representation
  // for the edges using the NearestNeighbor representation.
  // Check if roadmap has been provided.

  OMPL_INFORM("Planner has been setup.");
}

// ============================================================================
void COLPA::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) {
  // Make sure we setup the planner first.
  if (!static_cast<bool>(ompl::base::Planner::setup_)) {
    setup();
  }

  // Mark the planner's problem to be defined.
  ompl::base::Planner::setProblemDefinition(pdef);

  setupPreliminaries();
  OMPL_INFORM("Problem Definition has been setup.");
}

// ============================================================================
void COLPA::setupPreliminaries() {

  setupKNN();

  StatePtr sourceState(new colpa::datastructures::State(mSpace));
  mSpace->copyState(sourceState->getOMPLState(), pdef_->getStartState(0));

  StatePtr targetState(new colpa::datastructures::State(mSpace));
  mSpace->copyState(
      targetState->getOMPLState(), pdef_->getGoal()->as<ompl::base::GoalState>()->getState());

  // Add start and goal vertices to the graph
  mSourceVertex = boost::add_vertex(mGraph);
  mGraph[mSourceVertex].setState(sourceState);

  mTargetVertex = boost::add_vertex(mGraph);
  mGraph[mTargetVertex].setState(targetState);

  // Assign default values.
  mGraph[mSourceVertex].setRHSclass(0);
  // mGraph[mSourceVertex].setCostToCome(std::numeric_limits<double>::infinity());
  mGraph[mSourceVertex].setHclass(getGraphHeuristicClass(mSourceVertex));
  mGraph[mSourceVertex].setParent(mSourceVertex); // TODO : remove parent?

  // mGraph[mTargetVertex].setRHS(std::numeric_limits<double>::infinity());
  // mGraph[mTargetVertex].setCostToCome(std::numeric_limits<double>::infinity());
  mGraph[mTargetVertex].setHclass(0);
  // mGraph[mSourceVertex].setRHSclass(0);

  // Add existing vertices to nearest neighbor structure
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    knnGraph.add(*vi);
  } // End For vertex iteration

  // This is to connect edges to source and target vertices
  std::vector<Vertex> nearestSource;
  std::vector<Vertex> nearestTarget;

  // use knn instead of R-disc to ensure start/goal are connected sufficiently
  knnGraph.nearestK(mSourceVertex, mKNeighbors, nearestSource);
  // Add nearest vertices around the source to the graph
  // knnGraph.nearestR(mSourceVertex, mConnectionRadius, nearestSource);

  Edge uv;
  bool edgeExists;
  for (const auto& v : nearestSource) {
    // skip the source vertex itself
    if (mSourceVertex == v)
      continue;
    // double distance = mSpace->distance(
    //     mGraph[v].getState()->getOMPLState(), mGraph[mSourceVertex].getState()->getOMPLState());
    boost::tie(uv, edgeExists) = edge(mSourceVertex, v, mGraph);
    if (!edgeExists){
        std::pair<Edge, bool> newEdge = boost::add_edge(mSourceVertex, v, mGraph);
        // mGraph[newEdge.first].setLength(distance);
        mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
        assert(newEdge.second);
    }
  }

  // Add nearest vertices around the target to the graph
  knnGraph.nearestK(mTargetVertex, mGoalKNeighbors, nearestTarget);
  // knnGraph.nearestR(mTargetVertex, mConnectionRadius, nearestTarget);

  for (const auto& v : nearestTarget) {
    // skip the target vertex itself
    if (mTargetVertex == v)
      continue;
    // double distance = mSpace->distance(
    //     mGraph[v].getState()->getOMPLState(), mGraph[mTargetVertex].getState()->getOMPLState());
    boost::tie(uv, edgeExists) = edge(mTargetVertex, v, mGraph);
    if (!edgeExists){
        std::pair<Edge, bool> newEdge = boost::add_edge(mTargetVertex, v, mGraph);
        // mGraph[newEdge.first].setLength(distance);
        mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
        assert(newEdge.second);
    }
  }

}


// ============================================================================
void COLPA::freshStart() {

  // Call the base clear
  ompl::base::Planner::clear();

  // Clear the queues.
  mQueue.clear();
  assert(mQueue.isEmpty());

  // Reset the vertices and edges.
  // VertexIter vi, vi_end;
  // for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
  //   knnGraph.remove(*vi);
  // }

  knnGraph.clear();
  mGraph.clear();
  mNumberOfEdgeEvaluations = 0;
  mNumberOfVertexExpansions = 0;
  mPlannerStatus = PlannerStatus::NotSolved;
  OMPL_INFORM("Removed Everything");
}


// ============================================================================
void COLPA::clear() {
  // Call the base clear
  ompl::base::Planner::clear();

  // Clear the queues.
  mQueue.clear();
  assert(mQueue.isEmpty());


  // Reset the vertices and edges.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {

    mGraph[*vi].setGclass(std::numeric_limits<double>::infinity());
    mGraph[*vi].setRHSclass(std::numeric_limits<double>::infinity());
    mGraph[*vi].setHclass(std::numeric_limits<double>::infinity());
    mGraph[*vi].setColor(-1);

  }

  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(mGraph); ei != ei_end; ++ei) {
    mGraph[*ei].setEvaluationStatus(EvaluationStatus::NotEvaluated);
    mGraph[*ei].setColor(-1);
  }

  // Remove from knn the start the the goal
  knnGraph.remove(mSourceVertex);
  knnGraph.remove(mTargetVertex);

  // Remove edges between source, target to other vertices.
  clear_vertex(mSourceVertex, mGraph);
  clear_vertex(mTargetVertex, mGraph);

  // Remove the vertices themselves.
  remove_vertex(mSourceVertex, mGraph);
  remove_vertex(mTargetVertex, mGraph);

  mNumberOfEdgeEvaluations = 0;
  mNumberOfVertexExpansions = 0;
  mPlannerStatus = PlannerStatus::NotSolved;

  // TODO (jil) : clear kNN

  OMPL_INFORM("Cleared Everything");
}

// ============================================================================
void COLPA::setDebugCallback(std::function<void(Graph)> callback) {
    mCallback = callback;
}


// ============================================================================
ompl::base::PlannerStatus COLPA::solve(const ompl::base::PlannerTerminationCondition& ptc) {

  mPreempt = false;
  mPlannerStatus = PlannerStatus::NotSolved;

  // Return if source or target are in collision.
  if (evaluateVertex(mSourceVertex) == TotalClassNumber-1) {
    OMPL_INFORM("Start State is invalid.");
    return ompl::base::PlannerStatus::INVALID_START;
  }

  if (evaluateVertex(mTargetVertex) == TotalClassNumber-1) {
    OMPL_INFORM("Goal State is invalid.");
    return ompl::base::PlannerStatus::INVALID_GOAL;
  }

  // Sample first a batch.
  this->generateNewSamples(mSampleMultiplier, mSampleBufferSize, false);

  // Now update the source vertex for propagation to begin.
  this->updateVertex(mSourceVertex);

  // Main search
  while(!ptc && !mPreempt)
  {
    // Add an additional batch of samples, and updates these newly added ones.
    this->generateNewSamples(mSampleMultiplier, mSampleBufferSize, true);

    // Propagates inconsistency all the way
    this->computeShortestPath();
  }

  if (mPlannerStatus == PlannerStatus::Solved) {
    pdef_->addSolutionPath(constructSolution(mSourceVertex, mTargetVertex));
    this->setBestPathCost(mGraph[mTargetVertex].getGclass());

    OMPL_INFORM("Plan Found.");
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }
  if (mPreempt) {
    OMPL_INFORM("Planning Aborted.");
    return ompl::base::PlannerStatus::ABORT;
  }
  OMPL_INFORM("Planning TIMEOUT.");
  return ompl::base::PlannerStatus::TIMEOUT;
}

// ============================================================================
ompl::base::PlannerStatus COLPA::solve_once() {

  mPreempt = false;
  mPlannerStatus = PlannerStatus::NotSolved;

  // Return if source or target are in collision.
  if (evaluateVertex(mSourceVertex) == TotalClassNumber-1) {
    OMPL_INFORM("Start State is invalid.");
    return ompl::base::PlannerStatus::INVALID_START;
  }

  if (evaluateVertex(mTargetVertex) == TotalClassNumber-1) {
    OMPL_INFORM("Goal State is invalid.");
    return ompl::base::PlannerStatus::INVALID_GOAL;
  }

  // Now update the source vertex for propagation to begin.
  this->updateVertex(mSourceVertex);

  // Propagates inconsistency all the way
  this->computeShortestPath();

  if (mPlannerStatus == PlannerStatus::Solved) {
    pdef_->addSolutionPath(constructSolution(mSourceVertex, mTargetVertex));
    this->setBestPathCost(mGraph[mTargetVertex].getGclass());

    OMPL_INFORM("Plan Found.");
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }
  if (mPreempt) {
    OMPL_INFORM("Planning Aborted.");
    return ompl::base::PlannerStatus::ABORT;
  }
  OMPL_INFORM("Planning TIMEOUT.");
  return ompl::base::PlannerStatus::TIMEOUT;
}

// ============================================================================
Keys COLPA::calculateKeys(Vertex u) {
  // double minval = std::min(mGraph[u].getCostToCome(),mGraph[u].getRHS());
  // return std::make_pair(minval+this->getGraphHeuristic(u), minval);

  PathClassType minval = (mGraph[u].getGclass() <= mGraph[u].getRHSclass())?
                          mGraph[u].getGclass() : mGraph[u].getRHSclass();

  return std::make_pair(minval+getGraphHeuristicClass(u), minval);
}

// ============================================================================
// TODO (jil): Refurbish this based on COLPA.cpp
void COLPA::updateVertex(Vertex v) {
  // Don't find an optimal parent for start vertex
  if (v != mSourceVertex) {

    // Temporary data holder to find an optimal parent
    Vertex newParent;
    PathClassType tempRHS(std::numeric_limits<double>::infinity());
    bool foundNewParent=false;

    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(v, mGraph); ni != ni_end; ++ni) {
      Vertex u = *ni;

      // Get the edge between the two vertices.
      Edge uv = this->getEdge(u, v);

      // Now is the good time to evaluate the edge if we haven't
      if (mGraph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated) {

        this->evaluateEdge(uv);

      } // End If evaluationStatusCheck

      if (mGraph[u].getGclass() + mGraph[uv].getWclass() < tempRHS
       && mGraph[u].getParent()!= v)
         // )
      {
            tempRHS = mGraph[u].getGclass() + mGraph[uv].getWclass();
            newParent = u;
            foundNewParent= true;
      }

    } // End For neighbor

    // Actual assignment to the new optimal parent
    // if (std::isinf(tempRHS))
    // PathClassType infRHS(std::numeric_limits<double>::infinity());
    // // if (tempRHS ==  infRHS)
    if (!foundNewParent)
    {
      mGraph[v].setRHSclass(tempRHS);
      mGraph[v].removeParent();
    }
    else
    {
      mGraph[v].setRHSclass(tempRHS);
      mGraph[v].setParent(newParent);
    }

    // mGraph[v].setRHSclass(tempRHS);
    // mGraph[v].setParent(newParent);

  } // End if non-source vertex

  // Now enqueue this vertex
  // First, Remove this vertex in the queue
  mQueue.removeVertex(v);

  // Insert this if it is inconsistent
  if (!(mGraph[v].getGclass()==mGraph[v].getRHSclass()))
    mQueue.addVertexWithKeys(v, this->calculateKeys(v));

}

// ============================================================================
void COLPA::computeShortestPath() {

  while ( mQueue.keyComparison(mQueue.getTopVertexKeys(), this->calculateKeys(mTargetVertex)) ||
  !(mGraph[mTargetVertex].getRHSclass()==mGraph[mTargetVertex].getGclass()) ) {

    // For visualization only, safe to comment out or remove if not needed.
    // if(mCallback) mCallback(mGraph);

    // // Claim the mutex to check condition_variable
    // std::unique_lock<std::mutex> lck{mtx};
    //
    // // Wait (block this thread) until finish drawing the graph
    // cv.wait(lck);

    // auto tic = std::chrono::high_resolution_clock::now();

    // check if the queue is empty.. which means graph has no connected path
    if (mQueue.isEmpty()){
      OMPL_INFORM("No Path Exists in the graph");
      std::cout << "Vertex Expanded: " << mNumberOfVertexExpansions
        << ", Edge Evaluated: "<< mNumberOfEdgeEvaluations
        << ", Queue Size: " <<  mQueue.getSize() << std::endl;

      return;
    }


    // Pop front vertex from the queue
    Vertex frontVertex = mQueue.popTopVertex();

    // queIterCount++;
    //
    // std::cout << "--------------------" << std::endl;
    // std::cout << queIterCount << std::endl;
    //
    // std::cout << "target-RHS: " << mGraph[mTargetVertex].getRHSclass() << std::endl;
    // std::cout << "target-g: " << mGraph[mTargetVertex].getGclass() << std::endl;



    // std::cout << "front-RHS: " << frontVertex << mGraph[frontVertex].getRHSclass() << std::endl;
    // std::cout << "front-g: " << frontVertex << mGraph[frontVertex].getGclass() << std::endl;
    // std::cout << "--------------------" << std::endl;

    // Count the number of expansion
    mNumberOfVertexExpansions++;

    // Is it overconsistent?
    if (mGraph[frontVertex].getGclass() >mGraph[frontVertex].getRHSclass() ) {
      // Make it consistent
      mGraph[frontVertex].setGclass(mGraph[frontVertex].getRHSclass());
    }
    else {
      // Otherwise it is underconsistent, no consistent vertices are in the queue
      // Make it overconsistent or consistent
      mGraph[frontVertex].setGclass(std::numeric_limits<double>::infinity());

      // Update this vertex
      this->updateVertex(frontVertex);

    }
    // Now update the sucessor vertices
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(frontVertex, mGraph); ni != ni_end; ++ni) {
      Vertex u = *ni;
      this->updateVertex(u);
    } // End for successor vertices

    // auto toc = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    // mTotalVertexExpansionTime += time_span.count();


    // lck.unlock();
    // cv.notify_one();

  } // End while loop

  // std::cout << "End while loop" <<std::endl;
  mPlannerStatus = PlannerStatus::Solved;
  // mQueue.printQueue();
}

// ============================================================================
void COLPA::perceiveNewWorld() {
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    // This is for identifying changed vertices.... only for approximate change detection
    this->evaluateVertex(*vi);
  } // End For vertex iteration
}

// ============================================================================
bool COLPA::perceiveChanges() {

  // Hope no changes
  bool isChanged = false;

  // Flush out the previously perceived changes
  std::vector<colpa::datastructures::Edge> possibleChangedEdges;
  mPerceivedChangedEdges.clear();

  // Reset counters;
  mNumberOfEdgeEvaluations=0;

  mNumberOfVertexExpansions=0;

  std::vector<Vertex> verticesTobeUpdated;

  //////////////////////// Vertex Approximate Evaluation /////////////////
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {

    // Go through each vertex, to detect whether this has been changed.
    int oldStartColor = mGraph[*vi].getColor();

    if (this->evaluateVertex(*vi) != oldStartColor )
    {
      // Yes, this vertex is different than before.
      // Therefore all the incident edges need to be checked.
      // Collect all incident edges.
      NeighborIter ni, ni_end;
      for (boost::tie(ni, ni_end) = adjacent_vertices(*vi, mGraph); ni != ni_end; ++ni)
      {

        Vertex u = *ni;
        // Get the edge between the two vertices.
        Edge edge = this->getEdge(u, *vi);

        possibleChangedEdges.push_back(edge);

      }//End For neighboring edges

    } //End If vertex change

  } // End For vertex iteration

  // So far, we have collected candidate edges that could have been changed.
  std::cout << "Perceives possible "<< possibleChangedEdges.size() <<" edges change" << std::endl;

  // Now go through the candidate edges, and check if it did change.
  for (std::vector<Edge>::iterator it = possibleChangedEdges.begin() ; it != possibleChangedEdges.end(); ++it)
  {
    // Now is the time to evaluate
    int previousEdgeColor = mGraph[*it].getColor();
    // Did it really change? Note that evaluateEdge set the color, so no edges will be double counted
    if (previousEdgeColor!=this->evaluateEdge(*it))
    {
      // yes, indeed this edge is different.
      mPerceivedChangedEdges.push_back(*it);

      // Collect all the vertices to update once
      Vertex startVertex = source(*it, mGraph);

      Vertex endVertex = target(*it, mGraph);

      if (std::find(verticesTobeUpdated.begin(), verticesTobeUpdated.end(), startVertex) == verticesTobeUpdated.end())
      {verticesTobeUpdated.push_back(startVertex);}

      if (std::find(verticesTobeUpdated.begin(), verticesTobeUpdated.end(), endVertex) == verticesTobeUpdated.end())
      {verticesTobeUpdated.push_back(endVertex);}

    } // End If edge changed

  } // End For going through candidate edges

  std::cout <<  mPerceivedChangedEdges.size() <<" edges changed" << std::endl;
  std::cout <<  verticesTobeUpdated.size() <<" vertices to be updated" << std::endl;

  if (!verticesTobeUpdated.empty()){

    // Now update the vertices
    for (std::vector<Vertex>::iterator it = verticesTobeUpdated.begin() ; it != verticesTobeUpdated.end(); ++it) {

      // Important : Make sure if the source Vertex is changed, then make it inconsistent
      // if (*it == mSourceVertex) mGraph[*it].setCostToCome(std::numeric_limits<double>::max());
      if (*it == mSourceVertex) mGraph[*it].setGclass(std::numeric_limits<double>::infinity());

      // Assert that all changed edges are evaluated.
      this->updateVertex(*it);
    }

    // Okay, there is some change, we should re-solve it.
    isChanged = true;

    mPlannerStatus = PlannerStatus::NotSolved;

    pdef_->clearSolutionPaths();
  }

  return isChanged;
}

// ============================================================================
Edge COLPA::getEdge(Vertex u, Vertex v) {
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(u, v, mGraph);
  assert(edgeExists);

  return uv;
}

// ============================================================================
Path COLPA::getPathToSource(Vertex u) {
  Path pathToSource;
  while (u != mSourceVertex) {
    pathToSource.emplace_back(u);
    u = mGraph[u].getParent();
  }
  pathToSource.emplace_back(mSourceVertex);
  return pathToSource;
}


// ============================================================================
// TODO (avk): I should be able to set the heuristic function from the demo
// script. Create a Heuristic Class and send it in. Have a default heuristic
// if nothing has been set.
double COLPA::getGraphHeuristic(Vertex v) {
  double heuristic = mSpace->distance(
      mGraph[mTargetVertex].getState()->getOMPLState(), mGraph[v].getState()->getOMPLState());
  return heuristic;
}
// ============================================================================
PathClassType COLPA::getGraphHeuristicClass(Vertex v) {
  PathClassType hClass(0);
  double heuristic = mSpace->distance(
      mGraph[mTargetVertex].getState()->getOMPLState(), mGraph[v].getState()->getOMPLState());

  // What could be the worst class?
  // int worseClass = std::max(mGraph[v].getColorStatus(), mGraph[mTargetVertex].getColorStatus());
  hClass[0] = heuristic;
  return hClass;
}

// ============================================================================
void COLPA::setSpaceInformation(const ompl::base::SpaceInformationPtr& si) {
  si_ = si;
  mSpace = si->getStateSpace();
}

// ============================================================================
void COLPA::setKNeighbors(int num_neighbors) {
  mKNeighbors = num_neighbors;
}

// ============================================================================
void COLPA::setGoalKNeighbors(int num_neighbors) {
  mGoalKNeighbors = num_neighbors;
}

// ============================================================================
int COLPA::getKNeighbors() {
  return mKNeighbors;
}

// ============================================================================
int COLPA::getGoalKNeighbors() {
  return mGoalKNeighbors;
}

// ============================================================================
void COLPA::setSampleMultiplier(double sample_mul) {
  mSampleMultiplier = sample_mul;
}

// ============================================================================
double COLPA::getSampleMultiplier() {
  return mSampleMultiplier;
}

// ============================================================================
void COLPA::setSampleBufferSize(double buffer) {
  mSampleBufferSize = buffer;
}

// ============================================================================
double COLPA::getSampleBufferSize() {
  return mSampleBufferSize;
}

// ============================================================================
void COLPA::setConnectionRadius(double radius) {
  mConnectionRadius = radius;
}

// ============================================================================
double COLPA::getConnectionRadius() {
  return mConnectionRadius;
}

// Depreciated
// ============================================================================
void COLPA::setCollisionCheckResolution(double resolution) {
  mCollisionCheckResolution = resolution;
  getSpaceInformation()->setStateValidityCheckingResolution(resolution);
}

// ============================================================================
double COLPA::getCollisionCheckResolution() {
  return mCollisionCheckResolution;
}

// ============================================================================
void COLPA::setRoadmap(std::string filename) {
  if (filename == "")
    std::invalid_argument("Roadmap Filename cannot be empty!");

  // Load the graph.
  mRoadmap = boost::shared_ptr<io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>>(
      new io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>(mSpace, filename));

  mRoadmap->generate(
      mGraph, get(&VertexProperties::mState, mGraph), get(&EdgeProperties::mLength, mGraph));

  // Mark the graph to have been setup.
  mGraphSetup = true;
}

// ============================================================================
double COLPA::distFun(const colpa::datastructures::Vertex& v1, const colpa::datastructures::Vertex& v2) {
  mGraph[v1].getState()->getOMPLState();
  return mSpace->distance(
      mGraph[v1].getState()->getOMPLState(), mGraph[v2].getState()->getOMPLState());
}

// ============================================================================
void COLPA::setupKNN() {
  knnGraph.setDistanceFunction(
      std::bind(&COLPA::distFun, this, std::placeholders::_1, std::placeholders::_2));
}

// ============================================================================
void COLPA::setBestPathCost(PathClassType cost) {
  mBestPathCost = cost;
}

// ============================================================================
PathClassType COLPA::getBestPathCost() {
  return mBestPathCost;
}

// ============================================================================
void COLPA::setPlannerStatus(PlannerStatus status) {
  mPlannerStatus = status;
}

// ============================================================================
PlannerStatus COLPA::getPlannerStatus() {
  return mPlannerStatus;
}

// ============================================================================
double COLPA::getNumberOfEdgeEvaluations() {
  return mNumberOfEdgeEvaluations;
}

// ============================================================================
double COLPA::getNumberOfVertexExpansions() {
  return mNumberOfVertexExpansions;
}

// ============================================================================
int COLPA::evaluateVertex(const Vertex& v) {

  auto state = mGraph[v].getState()->getOMPLState();
  int color = mStateClassifier(state);
  mGraph[v].setColor(color);
  return color;

}

// ============================================================================
/* evaluateEdge given an edge, outputs the class of the edge (for perceiving chnages)
   and assigns intrinsically the value and the class of the edge.
*/
int COLPA::evaluateEdge(const Edge& e) {

  // For benchmark purposes
  auto tic = std::chrono::high_resolution_clock::now();
  mNumberOfEdgeEvaluations++;

  // Retreive start and end vertices
  Vertex startVertex = source(e, mGraph);
  Vertex endVertex = target(e, mGraph);

  // Retreive ompl states of start and end vertices
  auto startState = mGraph[startVertex].getState()->getOMPLState();
  auto endState = mGraph[endVertex].getState()->getOMPLState();

  // First check if this edge is valid motion,
  // note that validity check resolution is set for spaceInformation
  if(!getSpaceInformation()->checkMotion(startState, endState))
  {
    // Okay, this motion is not valid (checkMotion uses si stateValidityChecker)
    // Set value to infinty
    mGraph[e].setValue(std::numeric_limits<double>::infinity());
    // Set color to worst
    mGraph[e].setColor(TotalClassNumber-1);
    // Set evaluation status on
    mGraph[e].setEvaluationStatus(EvaluationStatus::Evaluated);
  }
  else
  {
    // Oh, this motion is valid. Let's compute the value and the class
    // Set value to the length of motion
    mGraph[e].setValue(mSpace->distance(startState, endState));
    // Set color the worst color of start and end vertices

    // Method 1 : approximation by checking end vertices
    // mGraph[e].setColor(std::max(this->evaluateVertex(startVertex),  this->evaluateVertex(endVertex)));

    // Method 2a : precisely interpolate using motion validator -> requires custom MV
    // mGraph[e].setColor(getSpaceInformation()->getMotionValidator()->classifyMotion());

    // Method 2b : interpolate using StateSpace
    int color = std::max(mStateClassifier(startState), mStateClassifier(endState));
    int steps = static_cast<unsigned int>(ceil(mGraph[e].getValue()));
    ompl::base::State* testState = si_->allocState();
    for (int i = 0; i < steps-1; i++) {
      mSpace->interpolate(startState, endState, (i+1)/steps, testState);
      color = std::max(mStateClassifier(testState),color);
    }
    si_->freeState(testState);
    mGraph[e].setColor(color);

    // Set evaluation status on
    mGraph[e].setEvaluationStatus(EvaluationStatus::Evaluated);
  }

  auto toc = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
  mTotalEdgeEvaluationTime += time_span.count();

  return mGraph[e].getColor();

  //
  //
  //
  // double edgeValue;
  //
  // // Interpolate the edge
  // int maxSteps = 1.0 / mCollisionCheckResolution;
  // for (int multiplier = 1; multiplier < maxSteps + 1; ++multiplier) {
  //   double interpolationStep = mCollisionCheckResolution * multiplier;
  //   assert(interpolationStep <= 1);
  //   StatePtr midVertex(new colpa::datastructures::State(mSpace));
  //   mSpace->interpolate(startState, endState, interpolationStep, midVertex->getOMPLState());
  //
  //   edgeClass = std::max(mStateClassifier(midVertex->getOMPLState()), edgeClass);
  // } // End For interpolation
  //
  //
  // edgeValue = mSpace->distance(startState, endState);
  // // Actual assignment
  // mGraph[e].setValue(edgeValue);
  // mGraph[e].setColor(edgeClass);
  // mGraph[e].setEvaluationStatus(EvaluationStatus::Evaluated);
}


// ============================================================================
ompl::base::PathPtr COLPA::constructSolution(const Vertex& source, const Vertex& target) {
  ompl::geometric::PathGeometric* path = new ompl::geometric::PathGeometric(si_);
  Vertex v = target;

  // std::cout << "path : " ;
  while (v != source) {
    // std::cout << v << ", " ;
    if(getSpaceInformation()->getStateDimension()>2)
    mGraph[v].getState()->getOMPLState()->as<ompl::base::SE2StateSpace::StateType>()->setYaw(
        wrapAngle(mGraph[v].getState()));

    path->append(mGraph[v].getState()->getOMPLState());
    v = mGraph[v].getParent();
  }
  // std::cout << std::endl;
  if (v == source) {
    path->append(mGraph[source].getState()->getOMPLState());
  }
  path->reverse();
  return ompl::base::PathPtr(path);
}


// ============================================================================

void COLPA::setStateClassifier(std::function<int(const ompl::base::State*)> classifier)
{
  mStateClassifier = classifier;
  OMPL_INFORM("StateClassifier Setup.");
}



// ============================================================================
std::vector<double> COLPA::haltonSample(std::size_t index) const {
  std::vector<int> bases{2, 3};

  // Generate a new sample.
  std::vector<double> sample;
  for (const auto& base : bases) {
    auto tempIndex = index;
    double result = 0.0;
    double f = 1.0;
    while (tempIndex > 0) {
      f /= base;
      result += f * (tempIndex % base);
      tempIndex /= base;
    }
    sample.push_back(result);
  }
  return sample;

}

// ============================================================================
void COLPA::generateGraph(std::vector<std::pair<double,double>>& states) {

  // Reset counters;
  mNumberOfEdgeEvaluations=0;

  mNumberOfVertexExpansions=0;

  mTotalEdgeEvaluationTime=0;

  mTotalVertexExpansionTime=0;

  // Collect near samples
  std::vector<Vertex> nearestSamples;

  auto validityChecker = si_->getStateValidityChecker();
  unsigned int dim = si_->getStateDimension();

  auto uniformSampler = si_->allocStateSampler();

  // Scale to required limits.
  int numSampled = 0;
  for (std::vector<std::pair<double,double>>::iterator state_it = states.begin() ; state_it != states.end(); ++state_it)
  {
    // ================= Uniform Sampler for theta ====================//
    StatePtr sampledState(new colpa::datastructures::State(mSpace));
    uniformSampler->sampleUniform(sampledState->getOMPLState());

    // Override x,y value with given state value
    sampledState->getOMPLState()->as<ompl::base::SE2StateSpace::StateType>()->setX(state_it->first);
    sampledState->getOMPLState()->as<ompl::base::SE2StateSpace::StateType>()->setY(state_it->second);

    int stateColor = mStateClassifier(sampledState->getOMPLState());

    // If the sampled state is in known region, but in collision, ignore.
    if(stateColor == TotalClassNumber-1){
      continue;
    }

    // Create a new vertex in the graph.
    Vertex sampleVertex = boost::add_vertex(mGraph);
    mGraph[sampleVertex].setState(std::make_shared<State>(mSpace, sampledState->getOMPLState()));
    mGraph[sampleVertex].setColor(stateColor);
    // Do we need to assign default values?

    knnGraph.nearestK(sampleVertex, mKNeighbors, nearestSamples);
    for (const auto& v : nearestSamples) {
        double distance = mSpace->distance(
            mGraph[sampleVertex].getState()->getOMPLState(), mGraph[v].getState()->getOMPLState());
        std::pair<Edge, bool> newEdge = boost::add_edge(sampleVertex, v, mGraph);
        mGraph[newEdge.first].setLength(distance);
        mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
        assert(newEdge.second);
    }

    // Now add to the graph
    knnGraph.add(sampleVertex);

  }

}

// ============================================================================
void COLPA::generateUniformSamples(int batchSize,bool updateVertices) {

  // Reset counters;
  mNumberOfEdgeEvaluations=0;

  mNumberOfVertexExpansions=0;

  mTotalEdgeEvaluationTime=0;

  mTotalVertexExpansionTime=0;

  // Vertices to be updated if updateVertices is set True
  std::vector<Vertex> verticesTobeUpdated;
  // Collect near samples
  std::vector<Vertex> nearestSamples;

  auto validityChecker = si_->getStateValidityChecker();
  unsigned int dim = si_->getStateDimension();


  const ompl::base::RealVectorBounds &bounds
    = static_cast<const ompl::base::RealVectorStateSpace*>(mSpace.get())->getBounds();

  auto uniformSampler = si_->allocStateSampler();

  // Scale to required limits.
  int numSampled = 0;
  while (numSampled < batchSize) {

    // ================= Uniform Sampler  ====================//
    // Our ompl::base::State* wrapper
    StatePtr sampledState(new colpa::datastructures::State(mSpace));

    uniformSampler->sampleUniform(sampledState->getOMPLState());
    // mSpace->copyFromReals(sampledState->getOMPLState(), newPosition);


    int stateColor = mStateClassifier(sampledState->getOMPLState());

    // If the sampled state is in known region, but in collision, ignore.
    if(stateColor == TotalClassNumber-1){
      continue;
    }


    // Since we have a valid sample, increment the numSampled.
    numSampled++;

    // Create a new vertex in the graph.
    Vertex sampleVertex = boost::add_vertex(mGraph);
    // mOnlineVertices.push_back(sampleVertex);
    mGraph[sampleVertex].setState(std::make_shared<State>(mSpace, sampledState->getOMPLState()));
    mGraph[sampleVertex].setColor(stateColor);
    // Do we need to assign default values?

    knnGraph.nearestK(sampleVertex, mKNeighbors, nearestSamples);
    for (const auto& v : nearestSamples) {
        double distance = mSpace->distance(
            mGraph[sampleVertex].getState()->getOMPLState(), mGraph[v].getState()->getOMPLState());
        std::pair<Edge, bool> newEdge = boost::add_edge(sampleVertex, v, mGraph);
        mGraph[newEdge.first].setLength(distance);
        mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
        assert(newEdge.second);
    }

    // Now add to the graph
    knnGraph.add(sampleVertex);
    verticesTobeUpdated.push_back(sampleVertex);

  } // End while a batch is sampled.

  // Update newly added vertices
  if (updateVertices)
  {
    // Now update the vertices
    for (std::vector<Vertex>::iterator it = verticesTobeUpdated.begin() ; it != verticesTobeUpdated.end(); ++it) {
      this->updateVertex(*it);
    }
    // Okay, there is some change, we should re-solve it.
    mPlannerStatus = PlannerStatus::NotSolved;

    pdef_->clearSolutionPaths();
  }

}

// ============================================================================
void COLPA::generateNewSamples(int batchSize, bool updateVertices) {

  // Reset counters;
  mNumberOfEdgeEvaluations=0;

  mNumberOfVertexExpansions=0;

  mTotalEdgeEvaluationTime=0;

  mTotalVertexExpansionTime=0;

  // Vertices to be updated if updateVertices is set True
  std::vector<Vertex> verticesTobeUpdated;
  // Collect near samples
  std::vector<Vertex> nearestSamples;

  auto validityChecker = si_->getStateValidityChecker();
  unsigned int dim = si_->getStateDimension();


  const ompl::base::RealVectorBounds &bounds
    = static_cast<const ompl::base::RealVectorStateSpace*>(mSpace.get())->getBounds();


  // Scale to required limits.
  int numSampled = 0;
  while (numSampled < batchSize) {
    // Generate a halton sample and increment the index.
    // Generate a halton sample and increment the index.
    auto newPosition = haltonSample(mHaltonIndex);
    mHaltonIndex++;


    for (unsigned int i = 0; i < dim; ++i)
      newPosition[i] = bounds.low[i] + newPosition[i] * (bounds.high[i] - bounds.low[i]);

    // Our ompl::base::State* wrapper
    StatePtr sampledState(new colpa::datastructures::State(mSpace));

    // copy the Halton sequenced point to ompl::base::State*
    mSpace->copyFromReals(sampledState->getOMPLState(), newPosition);

    // Since we have a valid sample, increment the numSampled.
    numSampled++;

    // Create a new vertex in the graph.
    Vertex sampleVertex = boost::add_vertex(mGraph);
    mGraph[sampleVertex].setState(sampledState);

    // Evaluate this vertex
    // this->evaluateVertex(sampleVertex);

    // Now add to the graph
    knnGraph.add(sampleVertex);
    verticesTobeUpdated.push_back(sampleVertex);

  } // End while a batch is sampled.

  // Update radius
  double connectionRadius = this->calculateR();
  std::cout << "current Connection Raidus: " << connectionRadius << std::endl;

  // Now Connect edges
  for (std::vector<Vertex>::iterator it = verticesTobeUpdated.begin() ; it != verticesTobeUpdated.end(); ++it)
  {
    // Collect near samples
    nearestSamples.clear();
    knnGraph.nearestR(*it, connectionRadius, nearestSamples);
    // if (updateVertices)  this->updateVertex(*it);
    // std::cout << "connecting "<<*it << " with: ";
    Edge uv;
    bool edgeExists;

    for (const auto& v : nearestSamples) {
      if(*it==v) continue;
      // std::cout << v << ", ";
      boost::tie(uv, edgeExists) = edge(*it, v, mGraph);
      if (!edgeExists)
      {
        double distance = mSpace->distance(
            mGraph[v].getState()->getOMPLState(), mGraph[*it].getState()->getOMPLState());
        std::pair<Edge, bool> newEdge = boost::add_edge(*it, v, mGraph);
        mGraph[newEdge.first].setLength(distance);
        mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
        assert(newEdge.second);
      }
      // if (updateVertices)
      // {
      //   this->updateVertex(v);
      // }
    }
    // std::cout << std::endl;

  }

  // Update newly added vertices
  if (updateVertices)
  {
    // Now update the vertices
    for (std::vector<Vertex>::iterator it = verticesTobeUpdated.begin() ; it != verticesTobeUpdated.end(); ++it) {
      this->updateVertex(*it);
    }
    // Okay, there is some change, we should re-solve it.
    mPlannerStatus = PlannerStatus::NotSolved;

    pdef_->clearSolutionPaths();
  }


  OMPL_INFORM("A new batch of %d samples generated",batchSize);
}


// ============================================================================
void COLPA::generateNewSamples(double sample_multiplier, double buffer, bool updateVertices) {

  std::vector<double> sourcePosition;
  std::vector<double> targetPosition;
  mSpace->copyToReals(sourcePosition, mGraph[mSourceVertex].getState()->getOMPLState());
  mSpace->copyToReals(targetPosition, mGraph[mTargetVertex].getState()->getOMPLState());

  // Find euc dist but cut off w
  Eigen::Vector2d sourceVect = Eigen::Vector2d(sourcePosition.data());
  Eigen::Vector2d targetVect = Eigen::Vector2d(targetPosition.data());

  double euc_dist = this->getGraphHeuristic(mSourceVertex);

  // Setup
  Eigen::Rotation2D<double> rot90Clock(-M_PI / 2);
  Eigen::Rotation2D<double> rot90CounterClock(M_PI / 2);

  // Define origin x_axis/y_axis vectors for sampling points in rectangle
  Eigen::Vector2d buffSource = (1 + buffer / euc_dist) * (sourceVect - targetVect) + targetVect;
  Eigen::Vector2d x_axis = (1 + 2 * buffer / euc_dist) * (targetVect - sourceVect);
  Eigen::Vector2d y_axis = rot90CounterClock.toRotationMatrix() * x_axis;
  Eigen::Vector2d bottom_v = rot90Clock.toRotationMatrix() * x_axis;
  Eigen::Vector2d origin = buffSource + 0.5 * bottom_v;
  double zmin = -M_PI;
  double zmax = M_PI;

  // Sample points inside the space.
  int minBatchSize = 100;
  int batchSize = std::floor(sample_multiplier * euc_dist) > minBatchSize ? std::floor(sample_multiplier * euc_dist) : minBatchSize;
  ompl::base::State* sampledState = mSpace->allocState();

  // Vertices to be updated if updateVertices is set True
  std::vector<Vertex> verticesTobeUpdated;

  // Scale to required limits.
  int numSampled = 0;
  // mOnlineVertices.reserve(mOnlineVertices.size() + batchSize);
  while (numSampled < batchSize) {

    // assert ReedsShepp
    std::vector<double> newPosition = mHaltonSequence->sample();

    // Scale the halton sample to between the limits.
    Eigen::Vector2d nPosition = Eigen::Vector2d(newPosition.data());
    nPosition = origin + nPosition[0] * x_axis + nPosition[1] * y_axis;
    newPosition = std::vector<double>{
        &nPosition[0], nPosition.data() + nPosition.cols() * nPosition.rows()};

    if (newPosition.size()>2)
      newPosition[2] = zmin + (zmax - zmin) * newPosition[2];

    mSpace->copyFromReals(sampledState, newPosition);

    int stateColor = mStateClassifier(sampledState);

    // If the sampled state is in known region, but in collision, ignore.
    if(stateColor == TotalClassNumber-1){
      continue;
    }


    // Since we have a valid sample, increment the numSampled.
    numSampled++;

    // Create a new vertex in the graph.
    Vertex sampleVertex = boost::add_vertex(mGraph);
    // mOnlineVertices.push_back(sampleVertex);
    mGraph[sampleVertex].setState(std::make_shared<State>(mSpace, sampledState));
    mGraph[sampleVertex].setColor(stateColor);
    // Do we need to assign default values?

    // 3. Connect edges
    std::vector<Vertex> nearestSamples;
    // knnGraph.nearestR(sampleVertex, mConnectionRadius, nearestSamples);
    knnGraph.nearestK(sampleVertex, mKNeighbors, nearestSamples);
    // std::cout << "Found " << nearestSamples.size() << "neighors" <<std::endl;
    for (const auto& v : nearestSamples) {
      // No need to assign distance, we are not using any edge heuristic.

      std::pair<Edge, bool> newEdge = boost::add_edge(sampleVertex, v, mGraph);
      // double distance = mSpace->distance(
      //     mGraph[v].getState()->getOMPLState(), mGraph[sampleVertex].getState()->getOMPLState());
      // mGraph[newEdge.first].setLength(distance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
      mGraph[newEdge.first].setColor(-1);
      assert(newEdge.second);
    }

    knnGraph.add(sampleVertex);
    verticesTobeUpdated.push_back(sampleVertex);
  }

  // Update newly added vertices
  if (updateVertices)
  {
    // Now update the vertices
    for (std::vector<Vertex>::iterator it = verticesTobeUpdated.begin() ; it != verticesTobeUpdated.end(); ++it) {
      this->updateVertex(*it);
    }
    // Okay, there is some change, we should re-solve it.
    mPlannerStatus = PlannerStatus::NotSolved;

    pdef_->clearSolutionPaths();
  }

  mNumSampleCalls++;
  // OMPL_INFORM("Added %d %dD samples", numSampled, getSpaceInformation()->getStateDimension());
}

// ============================================================================
double COLPA::calculateR() const
{
    // Cast to double for readability. (?)
    auto stateDimension = static_cast<double>(si_->getStateDimension());
    auto graphCardinality = static_cast<double>(boost::num_vertices(mGraph));
    // auto graphCardinality = static_cast<double>(mNumSamples);
    double approximateMesaure = si_->getSpaceMeasure();

    double minimumRGG = std::pow(2.0 * (1.0 + 1.0 / stateDimension) *
                        (approximateMesaure / ompl::unitNBallMeasure(si_->getStateDimension())),
                    1.0 / stateDimension);

    // Calculate the term and return.
    return mRewireFactor * minimumRGG *
           std::pow(std::log(graphCardinality) / graphCardinality, 1.0 / stateDimension);
}


// ============================================================================
// void COLPA::call_visualize()
// {
//
//   // Run this clock until joined
//   while(!mJoined)
//   {
//     // wait for a few miliseconds
//     std::this_thread::sleep_for(std::chrono::milliseconds(10));
//
//     std::unique_lock<std::mutex> lck{mtx};
//
//     // Now, draw the graph!
//     if(mCallback) mCallback(mGraph);
//
//     // Done drawing, let go of the lock
//     lck.unlock();
//     // I am done drawing, notify the main solve thread to continue;
//     cv.notify_one();
//   }
// }

// ============================================================================
void COLPA::abortPlanning() {
  mPreempt = true;
}

// ============================================================================
double COLPA::wrapAngle(const StatePtr stateptr) {
  double pi = 2 * acos(0.0);
  double x = stateptr->getOMPLState()->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
  x = fmod(x + pi, 2 * pi);
  if (x < 0)
    x += 2 * pi;
  return x - pi;
}

// ============================================================================
void COLPA::getGraph(std::vector<Node>* nodes, std::vector<std::tuple<Node, Node>>* edges) {
  if (mGraphSetup) {
    VertexIter vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
      // bool nodeInBounds = *vi == mTargetVertex || inBounds(*vi);
      bool nodeInBounds = true;
      Node vertex{
          mGraph[*vi]
              .getState()
              ->getOMPLState()
              ->as<ompl::base::SE2StateSpace::StateType>()
              ->getX(),
          mGraph[*vi]
              .getState()
              ->getOMPLState()
              ->as<ompl::base::SE2StateSpace::StateType>()
              ->getY(),
          mGraph[*vi]
              .getState()
              ->getOMPLState()
              ->as<ompl::base::SE2StateSpace::StateType>()
              ->getYaw(),
          nodeInBounds};
      nodes->push_back(vertex);

      NeighborIter ni, ni_end;
      Vertex u = *vi;
      for (boost::tie(ni, ni_end) = adjacent_vertices(u, mGraph); ni != ni_end; ++ni) {
        // bool nodeInBounds = *ni == mTargetVertex || inBounds(*ni);
        bool nodeInBounds = true;
        Node neighbor{
            mGraph[*ni]
                .getState()
                ->getOMPLState()
                ->as<ompl::base::SE2StateSpace::StateType>()
                ->getX(),
            mGraph[*ni]
                .getState()
                ->getOMPLState()
                ->as<ompl::base::SE2StateSpace::StateType>()
                ->getY(),
            mGraph[*ni]
                .getState()
                ->getOMPLState()
                ->as<ompl::base::SE2StateSpace::StateType>()
                ->getYaw(),
            nodeInBounds};
        edges->push_back(std::tuple<Node, Node>{vertex, neighbor});
      }
    }
  }
}

// ============================================================================
void COLPA::getGraphWithPaths(
    std::vector<Node>* nodes,
    std::vector<std::tuple<Node, Node>>* edges,
    std::vector<std::vector<Node>>* paths) {

  if (mGraphSetup) {
    VertexIter vi, vi_end;
    StatePtr midVertex(new colpa::datastructures::State(mSpace));
    paths->reserve(num_edges(mGraph));

    for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
      // bool nodeInBounds = *vi == mTargetVertex || inBounds(*vi);
      bool nodeInBounds = true;
      Node vertex{
          mGraph[*vi]
              .getState()
              ->getOMPLState()
              ->as<ompl::base::SE2StateSpace::StateType>()
              ->getX(),
          mGraph[*vi]
              .getState()
              ->getOMPLState()
              ->as<ompl::base::SE2StateSpace::StateType>()
              ->getY(),
          mGraph[*vi]
              .getState()
              ->getOMPLState()
              ->as<ompl::base::SE2StateSpace::StateType>()
              ->getYaw(),
          nodeInBounds};
      nodes->push_back(vertex);

      NeighborIter ni, ni_end;
      Vertex u = *vi;
      for (boost::tie(ni, ni_end) = adjacent_vertices(u, mGraph); ni != ni_end; ++ni) {
        // bool nodeInBounds = *ni == mTargetVertex || inBounds(*ni);
        bool nodeInBounds = true;
        Node neighbor{
            mGraph[*ni]
                .getState()
                ->getOMPLState()
                ->as<ompl::base::SE2StateSpace::StateType>()
                ->getX(),
            mGraph[*ni]
                .getState()
                ->getOMPLState()
                ->as<ompl::base::SE2StateSpace::StateType>()
                ->getY(),
            mGraph[*ni]
                .getState()
                ->getOMPLState()
                ->as<ompl::base::SE2StateSpace::StateType>()
                ->getYaw(),
            nodeInBounds};
        edges->push_back(std::tuple<Node, Node>{vertex, neighbor});

        // Generate path along the edge
        std::vector<Node> path;
        int maxSteps = 1.0 / mCollisionCheckResolution;
        path.reserve(maxSteps + 1);
        path.emplace_back(vertex);
        for (int multiplier = 1; multiplier < maxSteps + 1; ++multiplier) {
          double interpolationStep = mCollisionCheckResolution * multiplier;
          assert(interpolationStep <= 1);
          mSpace->interpolate(
              mGraph[*vi].getState()->getOMPLState(),
              mGraph[*ni].getState()->getOMPLState(),
              interpolationStep,
              midVertex->getOMPLState());
          Node midpoint{
              midVertex->getOMPLState()->as<ompl::base::SE2StateSpace::StateType>()->getX(),
              midVertex->getOMPLState()->as<ompl::base::SE2StateSpace::StateType>()->getY(),
              midVertex->getOMPLState()->as<ompl::base::SE2StateSpace::StateType>()->getYaw(),
              true};
          path.emplace_back(midpoint);
        }
        path.emplace_back(neighbor);
        paths->push_back(path);
      }
    }
  }
}
} // namespace colpa
