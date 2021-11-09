#ifndef COLPA_EVENT_CONSTANTDEPTHEVENT2_HPP_
#define COLPA_EVENT_CONSTANTDEPTHEVENT2_HPP_

#include <unordered_map>

#include "colpa/event/Event.hpp"

namespace colpa {
namespace event {

/// Event that triggers when the search tree reaches a particular depth.
/// Additionally, the event also triggers when the vertex is the target.
class ConstantDepthEvent : public Event {
public:
  /// Constructor.
  explicit ConstantDepthEvent(std::size_t depth);

  /// Documentation inherited.
  bool isTriggered(const colpa::datastructures::Vertex& vertex) override;

  /// Documentation inherited.
  void updateVertexProperties(colpa::datastructures::Vertex& vertex) override;

private:
  /// Get the depth of the vertex.
  std::size_t getDepth(const colpa::datastructures::Vertex& vertex);

  /// The threshold over depth.
  std::size_t mDepthThreshold;

};

} // namespace event
} // namespace colpa

#endif // COLPA_EVENT_CONSTANTDEPTHEVENT2_HPP_
