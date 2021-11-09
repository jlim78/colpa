#include "colpa/event/ConstantDepthEvent.hpp"

namespace colpa {
namespace event {

using colpa::datastructures::Edge;
using colpa::datastructures::EvaluationStatus;
using colpa::datastructures::Graph;
using colpa::datastructures::Vertex;

//==============================================================================
ConstantDepthEvent::ConstantDepthEvent(std::size_t depth) : mDepthThreshold(depth) {
  // Do nothing.
}

//==============================================================================
bool ConstantDepthEvent::isTriggered(const Vertex& vertex) {
  if (vertex == mTargetVertex) {
    return true;
  }

  if (getDepth(vertex) == mDepthThreshold) {
    return true;
  }
  return false;
}

//==============================================================================
void ConstantDepthEvent::updateVertexProperties(Vertex& vertex) {
  // Do nothing
}

//==============================================================================
std::size_t ConstantDepthEvent::getDepth(const Vertex& vertex) {

  size_t unevaluatedEdges = 0 ;

  Vertex currentVertex = vertex;
  // EXTREMELY IMPORTANT : DO NOT CALL auto FOR ACCESSING GRAPH, IT WILL COPY IT.
  // Access the graph.
  // auto graph = *mGraph;
  while (currentVertex!= mSourceVertex)
  {
    Vertex parentVertex = (*mGraph)[currentVertex].getParent();
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(parentVertex, currentVertex, (*mGraph));
    if ((*mGraph)[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
    {
        unevaluatedEdges++;
    }
    currentVertex = parentVertex;
  }

  return unevaluatedEdges;
}



} // namespace event
} // namespace colpa
