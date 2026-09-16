// SPDX-License-Identifier: GPL-2.0-only
#ifndef SIDEWALK_ROBOT_GATEWAY_H
#define SIDEWALK_ROBOT_GATEWAY_H

#include "ns3/gateway.h"
#include "ns3/network-module.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{

/*
 * ns3-cosim Gateway that lets a ROS 2 process drive robot mobility and
 *        message emission, and reports sidelink deliveries back.
 * Protocol (space-separated fields, "\r\n"-terminated, see ns3::Gateway):
 *   ROS -> ns-3, one message per time step:  sec nsec  [x y z send]*N
 *     x y z   position of robot i in metres (applied to its ExternalMobilityModel)
 *     send    1 = trigger one state-message transmission from robot i this step
 *   ns-3 -> ROS, the reply to each step:      [deliveries_i]*N
 *     deliveries_i  ";"-separated "src:latency_us" for every packet robot i
 *                   received since the previous step ("" if none)
 * Deliveries are matched to their transmission by ns-3 packet UID (app Tx
 * trace -> PacketSink Rx trace), so no application header is required.
 */
class RobotGateway : public Gateway
{
  public:
    /*
     * robots:   the robot nodes; index in the container == robot id
     * txEnable: transmissions requested before this time are ignored
     *                 (sidelink bearers are not active yet)
     */
    RobotGateway(NodeContainer robots, Time txEnable);

    // Hook the Tx trace of application 0 and the Rx trace of application 1 on every robot.
    void Attach();

  private:
    void DoInitialize(const std::vector<std::string>& data) override;
    void DoUpdate(const std::vector<std::string>& data) override;

    void HandleTx(uint32_t robot, Ptr<const Packet> packet);
    void HandleRx(uint32_t robot, Ptr<const Packet> packet, const Address& from);

    struct TxRecord
    {
        uint32_t robot;
        Time time;
    };

    NodeContainer m_robots;
    Time m_txEnable;
    std::map<uint64_t, TxRecord> m_tx;        // packet uid -> transmitter and tx time
    std::vector<std::string> m_deliveries;    // per robot, deliveries since last step
    uint64_t m_numTx{0};
    uint64_t m_numRx{0};
};

} // namespace ns3

#endif // SIDEWALK_ROBOT_GATEWAY_H
