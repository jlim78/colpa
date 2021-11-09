/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#include "colpa/LCOLPA.hpp"

#include <cmath>    // pow, sqrt
#include <iostream> // std::invalid_argument
#include <set>      // std::set
#include <assert.h> // debug

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
using colpa::event::ConstEventPtr;
using colpa::event::EventPtr;

namespace colpa {

LCOLPA::LCOLPA(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::Planner(si, "LCOLPA"), mSpace(si->getStateSpace()) {
  // Set default values for data members.

  // Halton sequence sampler for n dimensional search
  mHaltonSequence = std::make_shared<ompl::base::HaltonSequence>(si->getStateDimension());
}

LCOLPA::~LCOLPA() {
  // Do nothing.
}

// ============================================================================
void LCOLPA::setup() {
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
void LCOLPA::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) {
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
void LCOLPA::setupPreliminaries() {

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
  mGraph[mSourceVertex].setHclass(getGraphHeuristicClass(mSourceVertex));
  mGraph[mSourceVertex].setParent(mSourceVertex);

  mGraph[mTargetVertex].setHclass(0);

  // Add existing vertices to nearest neighbor structure
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    knnGraph.add(*vi);
  } // End For vertex iteration

  // This is to connect edges to source and target vertices
  std::vector<Vertex> nearestSource;
  std::vector<Vertex> nearestTarget;

  // Add nearest vertices around the source to the graph
  knnGraph.nearestR(mSourceVertex, mConnectionRadius, nearestSource);

  Edge uv;
  bool edgeExists;
  for (const auto& v : nearestSource) {
    // skip the source vertex itself
    if (mSourceVertex == v) continue;

    boost::tie(uv, edgeExists) = edge(mSourceVertex, v, mGraph);
    if (!edgeExists){
      double distance = mSpace->distance(
          mGraph[v].getState()->getOMPLState(), mGraph[mSourceVertex].getState()->getOMPLState());
      std::pair<Edge, bool> newEdge = boost::add_edge(mSourceVertex, v, mGraph);
      mGraph[newEdge.first].setLength(distance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
      assert(newEdge.second);
    }
  }

  // Add nearest vertices around the target to the graph
  knnGraph.nearestR(mTargetVertex, mConnectionRadius, nearestTarget);

  for (const auto& v : nearestTarget) {
    // skip the target vertex itself
    if (mTargetVertex == v) continue;
    boost::tie(uv, edgeExists) = edge(mTargetVertex, v, mGraph);
    if (!edgeExists){
      double distance = mSpace->distance(
          mGraph[v].getState()->getOMPLState(), mGraph[mTargetVertex].getState()->getOMPLState());
      std::pair<Edge, bool> newEdge = boost::add_edge(mTargetVertex, v, mGraph);
      mGraph[newEdge.first].setLength(distance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
      assert(newEdge.second);
    }
  }

  // Additionally connect the source and target with a straight line to snap.
  // std::pair<Edge, bool> newEdge = boost::add_edge(mSourceVertex, mTargetVertex, mGraph);
  // mGraph[newEdge.first].setLength(
  //     mSpace->distance(sourceState->getOMPLState(), targetState->getOMPLState()));
  // mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
  // mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);

  // Setup the event.
  mEvent->setup(&mGraph, mSourceVertex, mTargetVertex);


  // Initialize the search -- make sure initialize after the first batch is added
  // this->updateVertex(mSourceVertex);

}

// ============================================================================
void LCOLPA::clear() {
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

  // Remove edges between source, target to other vertices.
  clear_vertex(mSourceVertex, mGraph);
  clear_vertex(mTargetVertex, mGraph);

  // Remove the vertices themselves.
  remove_vertex(mSourceVertex, mGraph);
  remove_vertex(mTargetVertex, mGraph);

  PathClassType infclass(std::numeric_limits<double>::infinity());
  setBestPathCost(infclass);

  mNumberOfEdgeEvaluations = 0;
  mNumberOfVertexExpansions = 0;
  mPlannerStatus = PlannerStatus::NotSolved;

  // TODO(avk): Clear the selector and event.
  // TODO (jil) : clear kNN

  OMPL_INFORM("Cleared Everything");
}

// ============================================================================
void LCOLPA::setDebugCallback(std::function<void(Graph)> callback) {
    mCallback = callback;
}


// ============================================================================
ompl::base::PlannerStatus LCOLPA::solve(const ompl::base::PlannerTerminationCondition& /*ptc*/) {
  // TODO (avk): Use ptc to terminate the search.
  // mJoined = false;
  // std::thread visualize_thread{[this]() {call_visualize(); }};

  this->updateVertex(mSourceVertex);

  // Run inner loop.
  while (mPlannerStatus != PlannerStatus::Solved) {

    // For visualization only, safe to comment out or remove if not needed.
    // if(mCallback) mCallback(mGraph);

    // // Claim the mutex to use condition_variable to block this thread
    // std::unique_lock<std::mutex> lck{mtx};
    //
    // // Wait (block this thread )until finish drawing the graph
    // cv.wait(lck);

    /// Repair the tree till the event is triggered. returns the leaf
    Vertex triggeredLeaf;
    if(this->computeShortestPath(triggeredLeaf))
    {
      // std::cout << "Lazy LPA* returns a path "<< triggeredLeaf << std::endl;
      /// Evaluate along the subpath to the leaf, returns the first inconsis edge
      Edge inconsistentEdge ;
      bool inconsistencyExist = this->evaluatePath(triggeredLeaf,inconsistentEdge);

      // std::cout << "  evauated this path"<< triggeredLeaf << std::endl;

      /// Let the lazy LPA* handle the inconsistency
      if (inconsistencyExist) {
        // this->updateVertex(target(inconsistentEdge, mGraph));
        // std::cout<< "inconsistent edge found updated" <<std::endl;
        Vertex u = source(inconsistentEdge, mGraph);
        Vertex v = target(inconsistentEdge, mGraph);
        this->updateVertex(u);
        this->updateVertex(v);
      }
      else
      {
        // NO inconsistent edge is found,
        // if the triggering vertex is not a goal, we need to keep growing
        // but updating triggeredLeaf won't do anything, since it is consistent,
        // Hence, propagate forward.
        // if (triggeredLeaf != mTargetVertex)
        // {
        //   // Now update the sucessor vertices
        //   NeighborIter ni, ni_end;
        //   for (boost::tie(ni, ni_end) = adjacent_vertices(triggeredLeaf, mGraph); ni != ni_end; ++ni)
        //   {
        //     this->updateVertex(*ni);
        //   } // End for successor vertices
        // }
        // else // No inconsistentEdge is found, and triggered Leaf is the target. Solved!
        // {
        //   mPlannerStatus = PlannerStatus::Solved;
        // }
        if (triggeredLeaf == mTargetVertex)
        {
          mPlannerStatus = PlannerStatus::Solved;
        }
      }

      /// Check if we have found the optimal solution
      // if (!inconsistencyExist && triggeredLeaf == mTargetVertex) {
      //     mPlannerStatus = PlannerStatus::Solved;
      // }

    } else
    {
      // No triggering vertex exists
      OMPL_INFORM("No Trigerring Vertex Exists in the graph");
      std::cout << "Vertex Expanded: " << mNumberOfVertexExpansions
        << ", Edge Evaluated: "<< mNumberOfEdgeEvaluations
        << ", Queue Size: " <<  mQueue.getSize() << std::endl;
      break;
    }

    // lck.unlock();
    // cv.notify_one();

  } // End while loop

  // mJoined = true;
  // visualize_thread.join();

  std::cout << "Average Time to evaluate an edge : " << mTotalEdgeEvaluationTime/mNumberOfEdgeEvaluations << " s" <<std::endl;
  std::cout << "Average Time to exapnd a vertex : " << mTotalVertexExpansionTime/mNumberOfVertexExpansions << " s" <<std::endl;

  PathClassType infRHS(std::numeric_limits<double>::infinity());
  if (mPlannerStatus == PlannerStatus::Solved && !(mGraph[mTargetVertex].getGclass()==infRHS)) {
    setBestPathCost(mGraph[mTargetVertex].getGclass());
    pdef_->addSolutionPath(constructSolution(mSourceVertex, mTargetVertex));
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  } else {
    OMPL_INFORM("No Solution Found.");
    return ompl::base::PlannerStatus::TIMEOUT;
  }
}

// ============================================================================
Keys LCOLPA::calculateKeys(Vertex u) {

  PathClassType minval = (mGraph[u].getGclass() <= mGraph[u].getRHSclass())?
                          mGraph[u].getGclass() : mGraph[u].getRHSclass();

  return std::make_pair(minval+this->getGraphHeuristicClass(u), minval);

  // TODO: (jil) Do we need to store vertex costToGo heurstic?
  // return std::make_pair(minval+mGraph[u].getHeuristic(), minval);
}

// ============================================================================
void LCOLPA::updateVertex(Vertex v) {
  // Don't find an optimal parent for start vertex
  if (v != mSourceVertex) {

    // Temporary data holder to find an optimal parent
    Vertex newParent= boost::graph_traits<colpa::datastructures::BasicGraph>::null_vertex();
    PathClassType tempRHS(std::numeric_limits<double>::infinity());
    // bool foundNewParent=false;

    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(v, mGraph); ni != ni_end; ++ni) {
      Vertex u = *ni;

      // Get the edge between the two vertices.
      Edge uv = this->getEdge(u, v);

      // Get heuristic edge class if not evaluated yet
      if (mGraph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated) {
        //assign admissible color
        Vertex startVertex = source(uv, mGraph);
        Vertex endVertex = target(uv, mGraph);
        int edgeClass = std::max(this->evaluateVertex(startVertex),  this->evaluateVertex(endVertex));
        mGraph[uv].setColor(edgeClass);
        // mGraph[uv].setColor(0);
      }

      if (mGraph[u].getGclass() + mGraph[uv].getWclass() < tempRHS ){
          // && mGraph[u].getParent()!= v) {
            tempRHS = mGraph[u].getGclass() + mGraph[uv].getWclass();
            newParent = u;
            // foundNewParent= true;
      }


    } // End For neighbor

    // Actual assignment to the new optimal parent
    // Note that tempRHS is finite only if the new parent is found
    // mGraph[v].setRHSclass(tempRHS);
    // if (!foundNewParent)
    // {
    //   mGraph[v].removeParent();
    // }
    // else
    // {
    //   mGraph[v].setParent(newParent);
    // }
    mGraph[v].setRHSclass(tempRHS);
    mGraph[v].setParent(newParent);

  } // End if non-source vertex

  // Update the vertex property if it is connected to the tree, for Event trigger;
  //if(mGraph[v].hasParent()) mEvent->updateVertexProperties(v);

  // Now enqueue this vertex
  // First, Remove this vertex in the queue
  mQueue.removeVertex(v);

  // Insert this if it is inconsistent
  if(!(mGraph[v].getGclass()==mGraph[v].getRHSclass()))
    mQueue.addVertexWithKeys(v, this->calculateKeys(v));

}

// ============================================================================
bool LCOLPA::computeShortestPath(Vertex& triggeredLeafVertex) {


  while (mQueue.keyComparison(mQueue.getTopVertexKeys(), this->calculateKeys(mTargetVertex)) ||
  !(mGraph[mTargetVertex].getRHSclass()==mGraph[mTargetVertex].getGclass()) ) {

    // For visualization only, safe to comment out or remove if not needed.
    if(mCallback) mCallback(mGraph);

    auto tic = std::chrono::high_resolution_clock::now();

    if (mQueue.isEmpty()){
      OMPL_INFORM("No Path Exists in the graph");
      std::cout << "Vertex Expanded: " << mNumberOfVertexExpansions
        << ", Edge Evaluated: "<< mNumberOfEdgeEvaluations
        << ", Queue Size: " <<  mQueue.getSize() << std::endl;

      return false;
    }

    // Pop front vertex from the queue
    Vertex frontVertex = mQueue.popTopVertex();

    // std::cout << "--------------------" << std::endl;
    // std::cout << "target-RHS: " << mGraph[mTargetVertex].getRHSclass() << std::endl;
    // std::cout << "target-g: " << mGraph[mTargetVertex].getGclass() << std::endl;
    //
    //
    //
    // std::cout << "front-RHS: " << frontVertex << mGraph[frontVertex].getRHSclass() << std::endl;
    // std::cout << "front-g: " << frontVertex << mGraph[frontVertex].getGclass() << std::endl;
    // std::cout << "--------------------" << std::endl;

    // Count the number of expansion
    mNumberOfVertexExpansions++;


    // Is it overconsistent?
    if (mGraph[frontVertex].getGclass() >mGraph[frontVertex].getRHSclass() ) {

      // Yes, Make it consistent
      mGraph[frontVertex].setGclass(mGraph[frontVertex].getRHSclass());

      // Right after making it consistent, check if this triggers event
      if (mEvent->isTriggered(frontVertex)) {
        triggeredLeafVertex = frontVertex;
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
        mTotalVertexExpansionTime += time_span.count();

        // std::cout << "triggered ====== " <<std::endl;
        return true;
      }// No else, continue repairing.

    }
    else {
      // Otherwise it is underconsistent, since no consistent vertices are in the queue

      // Make it overconsistent (or consistent when rhs=inf)
      mGraph[frontVertex].setGclass(std::numeric_limits<double>::infinity());

      // Update this vertex
      this->updateVertex(frontVertex);

    }

    // Now update the sucessor vertices
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(frontVertex, mGraph); ni != ni_end; ++ni) {
      Vertex u = *ni;
      this->updateVertex(u);
      // std::cout << "updates neighbor: " << u << ", Queue :" << mQueue.getSize() <<std::endl;
    } // End for successor vertices

    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    mTotalVertexExpansionTime += time_span.count();

  } // End while loop

  // Okay, the tree seems to be consistent all the way up to the goal.
  // Let's return the goal
  triggeredLeafVertex = mTargetVertex;
  return true;

}

// ============================================================================
void LCOLPA::perceiveNewWorld() {
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    // This is for identifying changed vertices.... only for approximate change detection
    this->evaluateVertex(*vi);
  } // End For vertex iteration
}

// ============================================================================
bool LCOLPA::perceiveChanges() {

  // Hope no changes
  bool isChanged = false;

  // Flush out the previously perceived changes
  std::vector<colpa::datastructures::Edge> possibleChangedEdges;
  mPerceivedChangedEdges.clear();

  // Reset counters;
  mNumberOfEdgeEvaluations=0;

  mNumberOfVertexExpansions=0;

  mTotalEdgeEvaluationTime=0;

  mTotalVertexExpansionTime=0;

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

        mPerceivedChangedEdges.push_back(edge);

      }//End For neighboring edges

    } //End If vertex change

  } // End For vertex iteration

  // So far, we have collected candidate edges that could have been changed.
  std::cout << "Perceives possible "<< mPerceivedChangedEdges.size() <<" edges change" << std::endl;

  // Now go through the candidate edges, and check if it did change.
  for (std::vector<Edge>::iterator it = mPerceivedChangedEdges.begin() ; it != mPerceivedChangedEdges.end(); ++it)
  {
      // No need to insert already unclassified edge
      if (mGraph[*it].getColor() != -1)
      {
        // Make this edge unclassified
        mGraph[*it].setColor(-1);

        mGraph[*it].setEvaluationStatus(EvaluationStatus::NotEvaluated);

        mGraph[*it].setValue(mGraph[*it].getLength());

        // Collect all the vertices to update once
        Vertex startVertex = source(*it, mGraph);

        Vertex endVertex = target(*it, mGraph);

        if (std::find(verticesTobeUpdated.begin(), verticesTobeUpdated.end(), startVertex) == verticesTobeUpdated.end())
        {verticesTobeUpdated.push_back(startVertex);}

        if (std::find(verticesTobeUpdated.begin(), verticesTobeUpdated.end(), endVertex) == verticesTobeUpdated.end())
        {verticesTobeUpdated.push_back(endVertex);}

      } // End If edge unknown

  } // End For going through candidate edges


  std::cout <<  verticesTobeUpdated.size() <<" vertices to be updated" << std::endl;

  if (!verticesTobeUpdated.empty()){

    // Now update the vertices
    for (std::vector<Vertex>::iterator it = verticesTobeUpdated.begin() ; it != verticesTobeUpdated.end(); ++it) {

      // Important : Make sure if the source Vertex is changed, then make it inconsistent
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
void LCOLPA::setEvent(EventPtr event) {
  mEvent = event;
}

// ============================================================================
ConstEventPtr LCOLPA::getEvent() const {
  return mEvent;
}

// ============================================================================
Edge LCOLPA::getEdge(Vertex u, Vertex v) {
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(u, v, mGraph);
  assert(edgeExists);

  return uv;
}

// ============================================================================
Path LCOLPA::getPathToSource(Vertex u) {
  Path pathToSource;
  while (u != mSourceVertex) {
    pathToSource.emplace_back(u);
    assert(mGraph[u].hasParent());
    u = mGraph[u].getParent();
  }
  pathToSource.emplace_back(mSourceVertex);
  return pathToSource;
}

// ============================================================================
// TODO (avk): I should be able to set the heuristic function from the demo
// script. Create a Heuristic Class and send it in. Have a default heuristic
// if nothing has been set.
double LCOLPA::getGraphHeuristic(Vertex v) {
  double heuristic = mSpace->distance(
      mGraph[mTargetVertex].getState()->getOMPLState(), mGraph[v].getState()->getOMPLState());
  return heuristic;
}

// ============================================================================
PathClassType LCOLPA::getGraphHeuristicClass(Vertex v) {
  PathClassType hClass(0);
  double heuristic = mSpace->distance(
      mGraph[mTargetVertex].getState()->getOMPLState(), mGraph[v].getState()->getOMPLState());

  hClass[0] = heuristic;
  return hClass;
}

// ============================================================================
void LCOLPA::setConnectionRadius(double radius) {
  mConnectionRadius = radius;
}

// ============================================================================
double LCOLPA::getConnectionRadius() {
  return mConnectionRadius;
}

// ============================================================================
void LCOLPA::setCollisionCheckResolution(double resolution) {
  mCollisionCheckResolution = resolution;
}

// ============================================================================
double LCOLPA::getCollisionCheckResolution() {
  return mCollisionCheckResolution;
}

// ============================================================================
void LCOLPA::setRoadmap(std::string filename) {
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
double LCOLPA::distFun(const colpa::datastructures::Vertex& v1, const colpa::datastructures::Vertex& v2) {
  mGraph[v1].getState()->getOMPLState();
  return mSpace->distance(
      mGraph[v1].getState()->getOMPLState(), mGraph[v2].getState()->getOMPLState());
}

// ============================================================================
void LCOLPA::setupKNN() {
  knnGraph.setDistanceFunction(
      std::bind(&LCOLPA::distFun, this, std::placeholders::_1, std::placeholders::_2));
}

// ============================================================================
void LCOLPA::setBestPathCost(PathClassType cost) {
  mBestPathCost = cost;
}

// ============================================================================
PathClassType LCOLPA::getBestPathCost() {
  return mBestPathCost;
}

// ============================================================================
void LCOLPA::setPlannerStatus(PlannerStatus status) {
  mPlannerStatus = status;
}

// ============================================================================
PlannerStatus LCOLPA::getPlannerStatus() {
  return mPlannerStatus;
}

// ============================================================================
double LCOLPA::getNumberOfEdgeEvaluations() {
  return mNumberOfEdgeEvaluations;
}

// ============================================================================
double LCOLPA::getNumberOfVertexExpansions() {
  return mNumberOfVertexExpansions;
}

// ============================================================================
int LCOLPA::evaluateVertex(const Vertex& v) {

  auto state = mGraph[v].getState()->getOMPLState();
  int color = mStateClassifier(state);
  mGraph[v].setColor(color);
  return color;

}


// ============================================================================
int LCOLPA::evaluateEdge(const Edge& e) {

  auto tic = std::chrono::high_resolution_clock::now();

  mNumberOfEdgeEvaluations++;

  // Collision check the start and goal.
  Vertex startVertex = source(e, mGraph);
  Vertex endVertex = target(e, mGraph);

  auto startState = mGraph[startVertex].getState()->getOMPLState();
  auto endState = mGraph[endVertex].getState()->getOMPLState();

  int edgeClass = std::max(this->evaluateVertex(startVertex),  this->evaluateVertex(endVertex));

  double edgeValue;

  // Interpolate the edge
  int maxSteps = 1.0 / mCollisionCheckResolution;
  for (int multiplier = 1; multiplier < maxSteps + 1; ++multiplier) {
    double interpolationStep = mCollisionCheckResolution * multiplier;
    assert(interpolationStep <= 1);
    StatePtr midVertex(new colpa::datastructures::State(mSpace));
    mSpace->interpolate(startState, endState, interpolationStep, midVertex->getOMPLState());

    edgeClass = std::max(mStateClassifier(midVertex->getOMPLState()), edgeClass);
  } // End For interpolation


  edgeValue = mSpace->distance(startState, endState);
  // Actual assignment
  mGraph[e].setValue(edgeValue);
  mGraph[e].setColor(edgeClass);
  mGraph[e].setEvaluationStatus(EvaluationStatus::Evaluated);

  auto toc = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
  mTotalEdgeEvaluationTime += time_span.count();

  return edgeClass;

}

// ============================================================================
bool LCOLPA::evaluatePath(const Vertex& triggeredLeaf, Edge& inconsistentEdge) {

  Path path2Evaluate = this->getPathToSource(triggeredLeaf);

  Edge edgeToEvaluate;
  // Return the first unevaluated edge closest to source.
  for (std::size_t i = path2Evaluate.size() - 1; i > 0; --i) {
    // bool edgeExists;
    // boost::tie(edgeToEvaluate, edgeExists) = edge(path2Evaluate[i], path2Evaluate[i-1], mGraph);
    // assert(edgeExists);

    edgeToEvaluate= this->getEdge(path2Evaluate[i], path2Evaluate[i-1]);

    PathClassType oldClass = mGraph[edgeToEvaluate].getWclass();
    // int oldColor = mGraph[edgeToEvaluate].getColor();

    if (mGraph[edgeToEvaluate].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
    {
      // PathClassType oldClass = mGraph[edgeToEvaluate].getWclass();
      // Now is the good time to evaluate
      this->evaluateEdge(edgeToEvaluate);

      // Return this evaluated edge if it is inconsistent to the previous color.
      if (!(mGraph[edgeToEvaluate].getWclass() == oldClass)) {
      // if (!(mGraph[edgeToEvaluate].getColor() == oldColor)) {
          inconsistentEdge =  edgeToEvaluate;
          return true;
      }

    } //End If not evaluated edges

  } // End For path iteration

  // Nay, nothing special
  return false;

}

// ============================================================================
ompl::base::PathPtr LCOLPA::constructSolution(const Vertex& source, const Vertex& target) {
  ompl::geometric::PathGeometric* path = new ompl::geometric::PathGeometric(si_);
  Vertex v = target;

  // std::cout << "path : " ;
  while (v != source ) {
    // std::cout << v << ", " ;
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

void LCOLPA::setStateClassifier(std::function<int(const ompl::base::State*)> classifier)
{
  mStateClassifier = classifier;
}


// ============================================================================
std::vector<double> LCOLPA::haltonSample(std::size_t index) const {
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
void LCOLPA::generateNewSamples(int batchSize, bool updateVertices) {

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
    // auto newPosition = haltonSample(mHaltonIndex);
    // mHaltonIndex++;

    auto newPosition = mHaltonSequence->sample();

    // std::cout << numSampled << ": ";
    for (unsigned int i = 0; i < dim; ++i)
    {
      newPosition[i] = bounds.low[i] + newPosition[i] * (bounds.high[i] - bounds.low[i]);
      // std::cout << newPosition[i] <<", ";
    }
    // std::cout << std::endl;


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
      // if (v==mSourceVertex || v == mTargetVertex)
      // {
      //   std::cout <<"Found " << v << std::endl;
      // }
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
double LCOLPA::calculateR() const
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
void LCOLPA::call_visualize()
{

  // Run this clock until joined
  while(!mJoined)
  {
    // wait for a few miliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::unique_lock<std::mutex> lck{mtx};

    // Now, draw the graph!
    if(mCallback) mCallback(mGraph);

    // Done drawing, let go of the lock
    lck.unlock();
    // I am done drawing, notify the main solve thread to continue;
    cv.notify_one();
  }
}


} // namespace colpa
