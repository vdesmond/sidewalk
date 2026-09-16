// SPDX-License-Identifier: GPL-2.0-only
#ifndef SIDEWALK_SENSING_TRACE_SINK_H
#define SIDEWALK_SENSING_TRACE_SINK_H

#include "ns3/nr-sl-ue-mac.h"

#include <fstream>
#include <string>

namespace ns3
{

/**
 * \brief Writes one CSV row per execution of the Mode 2 sensing algorithm
 *        (TS 38.214 §8.1.4) by any UE MAC in the simulation.
 *
 * Connect with Config::Connect (with context) to
 * ".../NrUeMac/SensingAlgorithm"; the node id is parsed from the context.
 * Columns: time_s,node,slot,t0,tproc0,t1,t2,subchannels,l_subch,resource_pct,
 *          init_cand_slots,s_a_step4,s_a_step5,rsrp_thr_init_dbm,rsrp_thr_final_dbm,
 *          backoff_steps,cand_out,sensing_entries,tx_history
 */
class SensingTraceSink
{
  public:
    explicit SensingTraceSink(const std::string& csvPath);
    ~SensingTraceSink();

    /// Attach to every UE MAC's SensingAlgorithm trace source.
    void ConnectAll();

    void Save(std::string context,
              const NrSlUeMac::SensingTraceReport& report,
              const std::list<SlResourceInfo>& candidateResources,
              const std::list<SensingData>& sensingData,
              const std::list<SfnSf>& transmitHistory);

  private:
    std::ofstream m_out;
};

} // namespace ns3

#endif // SIDEWALK_SENSING_TRACE_SINK_H
