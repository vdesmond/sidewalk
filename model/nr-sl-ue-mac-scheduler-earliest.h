// SPDX-License-Identifier: GPL-2.0-only
#ifndef SIDEWALK_NR_SL_UE_MAC_SCHEDULER_EARLIEST_H
#define SIDEWALK_NR_SL_UE_MAC_SCHEDULER_EARLIEST_H

#include "ns3/nr-sl-ue-mac-scheduler-fixed-mcs.h"

namespace ns3
{

/**
 * \brief Latency-aware Mode 2 resource selection.
 *
 * TS 38.321 §5.22.1.1 says the MAC picks uniformly at random among the
 * candidate resources that survived sensing (TS 38.214 §8.1.4). For periodic
 * traffic on a semi-persistent grant with RRI == message period, the slot
 * picked at (re)selection fixes the arrival-to-grant offset of every later
 * packet, so a uniform pick costs on average half the selection window.
 *
 * This scheduler keeps only the earliest \c SlotFraction of the candidate
 * slots (never fewer slots than blind retransmissions need) and hands that
 * list to NrSlUeMacSchedulerFixedMcs::DoNrSlAllocation, which still does the
 * random draw, the PSFCH/min-time-gap constraints and the grant formatting.
 * SlotFraction = 1 reproduces the stock scheduler exactly.
 */
class NrSlUeMacSchedulerEarliest : public NrSlUeMacSchedulerFixedMcs
{
  public:
    static TypeId GetTypeId();
    NrSlUeMacSchedulerEarliest() = default;

  protected:
    bool DoNrSlAllocation(const std::list<SlResourceInfo>& candResources,
                          const std::shared_ptr<NrSlUeMacSchedulerDstInfo>& dstInfo,
                          std::set<SlGrantResource>& slotAllocList,
                          const AllocationInfo& allocationInfo) override;

  private:
    double m_slotFraction{0.25}; //!< fraction of earliest candidate slots kept
};

} // namespace ns3

#endif // SIDEWALK_NR_SL_UE_MAC_SCHEDULER_EARLIEST_H
