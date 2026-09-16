// SPDX-License-Identifier: GPL-2.0-only
#include "sensing-trace-sink.h"

#include "ns3/config.h"
#include "ns3/simulator.h"

namespace ns3
{

SensingTraceSink::SensingTraceSink(const std::string& csvPath)
    : m_out(csvPath)
{
    m_out << "time_s,node,slot,t0,tproc0,t1,t2,subchannels,l_subch,resource_pct,"
             "init_cand_slots,s_a_step4,s_a_step5,rsrp_thr_init_dbm,rsrp_thr_final_dbm,"
             "backoff_steps,cand_out,sensing_entries,tx_history\n";
}

SensingTraceSink::~SensingTraceSink()
{
    m_out.close();
}

void
SensingTraceSink::ConnectAll()
{
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                    "ComponentCarrierMapUe/*/NrUeMac/SensingAlgorithm",
                    MakeCallback(&SensingTraceSink::Save, this));
}

void
SensingTraceSink::Save(std::string context,
                       const NrSlUeMac::SensingTraceReport& r,
                       const std::list<SlResourceInfo>& candidateResources,
                       const std::list<SensingData>& sensingData,
                       const std::list<SfnSf>& transmitHistory)
{
    // context looks like /NodeList/<id>/DeviceList/...
    const auto nodeId = std::stoul(context.substr(10, context.find('/', 10) - 10));
    // The MAC raises the threshold in 3 dB steps until >= X% of candidates survive.
    // Upstream leaves both thresholds uninitialised when the sensing window is empty
    // (first selection after start-up), so only trust them when there was data.
    const bool valid = !sensingData.empty();
    const int backoffSteps = valid ? (r.m_finalRsrpThreshold - r.m_initialRsrpThreshold) / 3 : 0;

    m_out << Simulator::Now().GetSeconds() << ',' << nodeId << ',' << r.m_sfn.Normalize() << ','
          << r.m_t0 << ',' << +r.m_tProc0 << ',' << +r.m_t1 << ',' << r.m_t2 << ','
          << r.m_subchannels << ',' << r.m_lSubch << ',' << +r.m_resourcePercentage << ','
          << r.m_initialCandidateSlotsSize << ',' << r.m_initialCandidateResourcesSize << ','
          << r.m_candidateResourcesSizeAfterStep5 << ',';
    if (valid)
    {
        m_out << r.m_initialRsrpThreshold << ',' << r.m_finalRsrpThreshold;
    }
    else
    {
        m_out << ',';
    }
    m_out << ',' << backoffSteps << ',' << candidateResources.size()
          << ',' << sensingData.size() << ',' << transmitHistory.size() << '\n';
}

} // namespace ns3
