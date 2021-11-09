/* Authors: Aditya Vamsikrishna Mandalika */

#include "colpa/event/Event.hpp"

namespace colpa {
namespace event {

using colpa::datastructures::Graph;
using colpa::datastructures::Queue;
using colpa::datastructures::Vertex;

//==============================================================================
Event::Event() {
  // Do nothing.
}

//==============================================================================
void Event::setup(Graph* graph, Vertex& source, Vertex& target) {
  mGraph = graph;
  mSourceVertex = source;
  mTargetVertex = target;
}

//==============================================================================
void Event::updateVertexProperties(Queue& updateQueue) {
  // Access the graph.
  // auto graph = *mGraph;
  //
  // while (!updateQueue.isEmpty()) {
  //   // Update the top vertex.
  //   Vertex vertex = updateQueue.popTopVertex();
  //   updateVertexProperties(vertex);
  //
  //   auto children = graph[vertex].getChildren();
  //   for (auto iterV = children.begin(); iterV != children.end(); ++iterV) {
  //     // Add the children into the queue for update.
  //     assert(!updateQueue.hasVertexWithValue(*iterV, graph[*iterV].getCostToCome()));
  //     updateQueue.addVertexWithValue(*iterV, graph[*iterV].getCostToCome());
  //   }
  // }
}

} // namespace event
} // namespace colpa
