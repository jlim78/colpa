#ifndef COLPA_EVENT_SHORTESTPATHEVENT_HPP_
#define COLPA_EVENT_SHORTESTPATHEVENT_HPP_

#include "colpa/event/Event.hpp"

namespace colpa {
namespace event {

/// Event that triggers when a shortest path to goal is found.
class ShortestPathEvent : public Event {
public:
  /// Constructor.
  ShortestPathEvent();

  /// Documentation inherited.
  bool isTriggered(const colpa::datastructures::Vertex& vertex) override;

  /// Documentation inherited.
  void updateVertexProperties(colpa::datastructures::Vertex& vertex) override;
};

} // namespace event
} // namespace colpa

#endif // COLPA_EVENT_SHORTESTPATHEVENT_HPP_
