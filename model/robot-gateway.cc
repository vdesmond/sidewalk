// SPDX-License-Identifier: GPL-2.0-only
#include "robot-gateway.h"

#include "ns3/external-mobility-model.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/triggered-send-application.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("RobotGateway");

RobotGateway::RobotGateway(NodeContainer robots, Time txEnable)
    : Gateway(robots.GetN()),
      m_robots(robots),
      m_txEnable(txEnable),
      m_deliveries(robots.GetN())
{
}

void
RobotGateway::Attach()
{
    for (uint32_t i = 0; i < m_robots.GetN(); i++)
    {
        Ptr<Node> node = m_robots.Get(i);
        NS_ABORT_MSG_IF(node->GetNApplications() < 2, "robot needs a sender and a sink app");
        node->GetApplication(0)->TraceConnectWithoutContext(
            "Tx",
            MakeBoundCallback(
                +[](RobotGateway* self, uint32_t r, Ptr<const Packet> p) { self->HandleTx(r, p); },
                this,
                i));
        node->GetApplication(1)->TraceConnectWithoutContext(
            "Rx",
            MakeBoundCallback(+[](RobotGateway* self,
                                  uint32_t r,
                                  Ptr<const Packet> p,
                                  const Address& a) { self->HandleRx(r, p, a); },
                              this,
                              i));
    }
}

void
RobotGateway::DoInitialize(const std::vector<std::string>& data)
{
    DoUpdate(data);
}

void
RobotGateway::DoUpdate(const std::vector<std::string>& data)
{
    static const uint32_t FIELDS = 4; // x y z send
    NS_ABORT_MSG_IF(data.size() < FIELDS * m_robots.GetN(),
                    "gateway update has " << data.size() << " fields, expected "
                                          << FIELDS * m_robots.GetN());
    const Time now = Simulator::Now();
    for (uint32_t i = 0; i < m_robots.GetN(); i++)
    {
        const uint32_t k = i * FIELDS;
        Ptr<Node> node = m_robots.Get(i);
        Ptr<ExternalMobilityModel> mob = node->GetObject<ExternalMobilityModel>();
        NS_ABORT_MSG_IF(!mob, "robot " << i << " has no ExternalMobilityModel");
        mob->SetPosition(Vector(std::stod(data[k]), std::stod(data[k + 1]), std::stod(data[k + 2])));
        if (std::stoi(data[k + 3]) && now >= m_txEnable)
        {
            DynamicCast<TriggeredSendApplication>(node->GetApplication(0))->Send(1);
        }
        SetValue(i, m_deliveries[i]);
        m_deliveries[i].clear();
    }
    SendResponse();

    // forget transmissions older than a second; every receiver has had its chance
    for (auto it = m_tx.begin(); it != m_tx.end();)
    {
        it = (now - it->second.time > Seconds(1)) ? m_tx.erase(it) : std::next(it);
    }
    if (m_numTx && m_numTx % 1000 == 0)
    {
        NS_LOG_INFO(now.As(Time::S) << " tx=" << m_numTx << " rx=" << m_numRx);
    }
}

void
RobotGateway::HandleTx(uint32_t robot, Ptr<const Packet> packet)
{
    m_tx[packet->GetUid()] = {robot, Simulator::Now()};
    m_numTx++;
}

void
RobotGateway::HandleRx(uint32_t robot, Ptr<const Packet> packet, const Address& /*from*/)
{
    auto it = m_tx.find(packet->GetUid());
    if (it == m_tx.end())
    {
        return; // not one of ours (or older than the retention window)
    }
    const int64_t latencyUs = (Simulator::Now() - it->second.time).GetMicroSeconds();
    std::string& d = m_deliveries[robot];
    if (!d.empty())
    {
        d += ';';
    }
    d += std::to_string(it->second.robot) + ':' + std::to_string(latencyUs);
    m_numRx++;
}

} // namespace ns3
