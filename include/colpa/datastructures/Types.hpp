/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#ifndef COLPA_DATASTRUCTURES_TYPES_HPP_
#define COLPA_DATASTRUCTURES_TYPES_HPP_

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

#include "colpa/datastructures/PathClass.hpp" // For PathClassType def

// This is to define the total number of classes in the world
#define TotalClassNumber 4

namespace colpa {
namespace datastructures {

typedef PathClass<TotalClassNumber> PathClassType;

typedef std::pair<PathClassType, PathClassType> Keys;
// typedef std::pair<double, double> Keys;

/// Basic Boost graph.
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> BasicGraph;

/// Shared pointer to a BasicGraph.
typedef std::shared_ptr<BasicGraph> BasicGraphPtr;

/// Shared pointer to a const BasicGraph.
typedef std::shared_ptr<const BasicGraph> ConstBasicGraphPtr;

/// Boost vertex.
typedef boost::graph_traits<BasicGraph>::vertex_descriptor Vertex;

/// Shared pointer to a Vertex.
typedef std::shared_ptr<Vertex> VertexPtr;

/// Shared pointer to a const Vertex.
typedef std::shared_ptr<const Vertex> ConstVertexPtr;

/// Path represented by a series of vertices.
typedef std::vector<Vertex> Path;

/// Shared pointer to a Path.
typedef std::shared_ptr<Path> PathPtr;

/// Shared pointer to a const Path.
typedef std::shared_ptr<const Path> ConstPathPtr;

/// Boost edge
typedef boost::graph_traits<BasicGraph>::edge_descriptor Edge;

/// Shared pointer to an Edge.
typedef std::shared_ptr<Edge> EdgePtr;

/// Shared pointer to a const Edge.
typedef std::shared_ptr<const Edge> ConstEdgePtr;

} // namespace datastructures
} // namespace colpa

#endif // COLPA_DATASTRUCTURES_TYPES_HPP_
