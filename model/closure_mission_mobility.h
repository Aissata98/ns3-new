#ifndef CLOSURE_MISSION_MOBILITY_H
#define CLOSURE_MISSION_MOBILITY_H

#include "ns3/mobility-model.h"
#include "ns3/simulator.h"
#include <functional>

// Every acoustic propagation and optical contact query obtains the same
// continuous trajectory at the actual ns-3 event time, not a tick-held position.
// Before evaluators are installed, MobilityHelper may set the initial pose.
class ReviewedMissionMobility : public ns3::MobilityModel
{
public:
  static ns3::TypeId GetTypeId()
  {
    static ns3::TypeId id = ns3::TypeId("ns3::ReviewedMissionMobility")
        .SetParent<ns3::MobilityModel>()
        .SetGroupName("Mobility")
        .AddConstructor<ReviewedMissionMobility>();
    return id;
  }
  void SetEvaluators(std::function<ns3::Vector(double)> position,
                     std::function<ns3::Vector(double)> velocity)
  {
    m_position = std::move(position);
    m_velocity = std::move(velocity);
    NotifyCourseChange();
  }
private:
  ns3::Vector DoGetPosition() const override
  { return m_position ? m_position(ns3::Simulator::Now().GetSeconds()) : m_initial; }
  ns3::Vector DoGetVelocity() const override
  { return m_velocity ? m_velocity(ns3::Simulator::Now().GetSeconds()) : ns3::Vector(); }
  void DoSetPosition(const ns3::Vector& value) override
  {
    if (!m_position) { m_initial = value; NotifyCourseChange(); }
    // Once bound, the checked analytic mission is the only position owner.
  }
  ns3::Vector m_initial;
  std::function<ns3::Vector(double)> m_position, m_velocity;
};

#endif
