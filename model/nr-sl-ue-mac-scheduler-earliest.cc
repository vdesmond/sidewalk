// SPDX-License-Identifier: GPL-2.0-only
#include "nr-sl-ue-mac-scheduler-earliest.h"

#include "ns3/double.h"
#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrSlUeMacSchedulerEarliest");
NS_OBJECT_ENSURE_REGISTERED(NrSlUeMacSchedulerEarliest);

TypeId
NrSlUeMacSchedulerEarliest::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrSlUeMacSchedulerEarliest")
            .SetParent<NrSlUeMacSchedulerFixedMcs>()
            .AddConstructor<NrSlUeMacSchedulerEarliest>()
            .AddAttribute("SlotFraction",
                          "Fraction of the earliest candidate slots the random draw is "
                          "restricted to (1 = stock uniform selection over the whole window)",
                          DoubleValue(0.25),
                          MakeDoubleAccessor(&NrSlUeMacSchedulerEarliest::m_slotFraction),
                          MakeDoubleChecker<double>(0.0, 1.0));
    return tid;
}

bool
NrSlUeMacSchedulerEarliest::DoNrSlAllocation(
    const std::list<SlResourceInfo>& candResources,
    const std::shared_ptr<NrSlUeMacSchedulerDstInfo>& dstInfo,
    std::set<SlGrantResource>& slotAllocList,
    const AllocationInfo& allocationInfo)
{
    NS_LOG_FUNCTION(this << candResources.size());

    // Distinct slots in the (sensing-cleared) candidate set, in time order.
    std::set<SfnSf> slots;
    for (const auto& res : candResources)
    {
        slots.insert(res.sfn);
    }

    // Keep the earliest ceil(fraction * N) slots, but never fewer than the
    // number of transmissions a grant needs (blind retransmissions must land
    // in distinct slots) so the base class can still fill the grant.
    const std::size_t minSlots = std::max<std::size_t>(1, GetSlMaxTxTransNumPssch());
    std::size_t keep = static_cast<std::size_t>(std::ceil(m_slotFraction * slots.size()));
    keep = std::clamp(keep, std::min(minSlots, slots.size()), slots.size());

    auto cutoff = slots.begin();
    std::advance(cutoff, keep - 1);
    const SfnSf lastKept = *cutoff;

    std::list<SlResourceInfo> earliest;
    for (const auto& res : candResources)
    {
        if (!(lastKept < res.sfn))
        {
            earliest.push_back(res);
        }
    }
    NS_LOG_INFO("kept " << keep << " of " << slots.size() << " candidate slots ("
                        << earliest.size() << " of " << candResources.size() << " resources)");

    return NrSlUeMacSchedulerFixedMcs::DoNrSlAllocation(earliest,
                                                         dstInfo,
                                                         slotAllocList,
                                                         allocationInfo);
}

} // namespace ns3
