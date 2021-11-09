#include "colpa/event/ShortestPathEvent.hpp"

namespace colpa {
namespace event {

using colpa::datastructures::Graph;
using colpa::datastructures::Queue;
using colpa::datastructures::Vertex;

//==============================================================================
ShortestPathEvent::ShortestPathEvent() {
  // Do nothing.
}

//==============================================================================
bool ShortestPathEvent::isTriggered(const Vertex& vertex) {
  if (vertex == mTargetVertex) {
    return true;
  }
  return false;
}

//==============================================================================
void ShortestPathEvent::updateVertexProperties(Vertex& /*vertex*/) {
  // Do nothing.
}

} // namespace event
} // namespace colpa
