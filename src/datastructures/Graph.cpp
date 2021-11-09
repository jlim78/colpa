/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#include "colpa/datastructures/Graph.hpp"

namespace colpa {
namespace datastructures {

// ============================================================================
void VertexProperties::setState(StatePtr state) {
  mState = state;
}

// ============================================================================
StatePtr VertexProperties::getState() {
  return mState;
}

// ============================================================================
void VertexProperties::setGclass(PathClassType g_class) {
  mGclass = g_class;
}

// ============================================================================
PathClassType VertexProperties::getGclass() {
  return mGclass;
}

// ============================================================================
void VertexProperties::setRHSclass(PathClassType rhs_class) {
  mRHSclass = rhs_class;
}

// ============================================================================
PathClassType VertexProperties::getRHSclass() {
  return mRHSclass;
}

// ============================================================================
void VertexProperties::setHclass(PathClassType h_class) {
  mHclass = h_class;
}

// ============================================================================
PathClassType VertexProperties::getHclass() {
  return mHclass;
}


// ============================================================================
void VertexProperties::setCostToCome(double cost) {
  mCostToCome = cost;
}

// ============================================================================
double VertexProperties::getCostToCome() {
  return mCostToCome;
}

// ============================================================================
void VertexProperties::setRHS(double cost) {
  mRHS = cost;
}

// ============================================================================
double VertexProperties::getRHS() {
  return mRHS;
}

// ============================================================================
void VertexProperties::setHeuristic(double heuristic) {
  mHeuristic = heuristic;
}

// ============================================================================
double VertexProperties::getHeuristic() {
  return mHeuristic;
}

// ============================================================================
double VertexProperties::getEstimatedTotalCost() {
  return mCostToCome + mHeuristic;
}

// ============================================================================
void VertexProperties::setParent(Vertex parent) {
  mParent = parent;
}

// ============================================================================
Vertex VertexProperties::getParent() {
  return mParent;
}

// ============================================================================
bool VertexProperties::hasParent() {
  return mParent != boost::graph_traits<BasicGraph>::null_vertex();
}

// ============================================================================
void VertexProperties::removeParent( ) {
  mParent = boost::graph_traits<BasicGraph>::null_vertex();
}

// ============================================================================
void VertexProperties::setColor(int color) {
  mColor = color;
}

// ============================================================================
int VertexProperties::getColor() {
  return mColor;
}


// ============================================================================
void EdgeProperties::setLength(double length) {
  mLength = length;
}

// ============================================================================
double EdgeProperties::getLength() {
  return mLength;
}

// ============================================================================
void EdgeProperties::setValue(double value) {
  mValue = value;
  mValueEvaluated = true;
}

// ============================================================================
double EdgeProperties::getValue() {
  if (mValueEvaluated)
    return mValue;
  else
    return mLength;
}

// ============================================================================
PathClassType EdgeProperties::getWclass() {
  PathClassType wClass(0);
  if (mValueEvaluated)
    wClass[mColor]= mValue;
  else
    wClass[mColor]= mLength;
    // wClass[0]= mLength;

  return wClass;
}

// ============================================================================
void EdgeProperties::setEvaluationStatus(EvaluationStatus evaluationStatus) {
  mEvaluationStatus = evaluationStatus;
}

// ============================================================================
EvaluationStatus EdgeProperties::getEvaluationStatus() {
  return mEvaluationStatus;
}

// ============================================================================
void EdgeProperties::setColor(int color) {
  mColor = color;
}

// ============================================================================
int EdgeProperties::getColor() {
  return mColor;
}


} // namespace datastructures
} // namespace colpa
