/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
// Copyright (c) 2026 Vignesh Mohan (sidewalk modifications)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * \ingroup examples
 * \file sidewalk-robots.cc
 * \brief Cooperative robots exchanging periodic state messages over NR sidelink Mode 2.
 *
 * Derived from nr-v2x-west-to-east-highway.cc (5G-LENA). The highway lanes are
 * replaced by N robots moving in a square area (random waypoint), every robot
 * groupcasts a short state message every msgPeriod ms and receives everyone
 * else's. Everything else -- out-of-coverage sidelink, Mode 2 sensing-based
 * SPS, KPI collection into SQLite -- is the upstream machinery.
 *
 * In addition to the v2x-kpi tables, every run of the Mode 2 sensing algorithm
 * is logged to <outputDir><simTag>-sensing.csv (see SensingTraceSink).
 *
 * \code{.unparsed}
$ ./ns3 run "sidewalk-robots --help"
    \endcode
 */

#include "v2x-kpi.h"

#include "ns3/external-mobility-model.h"
#include "ns3/nr-sl-ue-mac-scheduler-earliest.h"
#include "ns3/robot-gateway.h"
#include "ns3/triggered-send-helper.h"
#include "ns3/sensing-trace-sink.h"

#include "ns3/antenna-module.h"
#include "ns3/buildings-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"

#include <iomanip>
#include <memory>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SidewalkRobots");

/*
 * Global methods to hook trace sources from different layers of
 * the protocol stack.
 */

/**
 * \brief Method to listen the trace SlPscchScheduling of NrUeMac, which gets
 *        triggered upon the transmission of SCI format 1-A from UE MAC.
 *
 * \param pscchStats Pointer to the UeMacPscchTxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param pscchStatsParams Parameters of the trace source.
 */
void
NotifySlPscchScheduling(UeMacPscchTxOutputStats* pscchStats,
                        const SlPscchUeMacStatParameters pscchStatsParams)
{
    pscchStats->Save(pscchStatsParams);
}

/**
 * \brief Method to listen the trace SlPsschScheduling of NrUeMac, which gets
 *        triggered upon the transmission of SCI format 2-A and data from UE MAC.
 *
 * \param psschStats Pointer to the UeMacPsschTxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param psschStatsParams Parameters of the trace source.
 */
void
NotifySlPsschScheduling(UeMacPsschTxOutputStats* psschStats,
                        const SlPsschUeMacStatParameters psschStatsParams)
{
    psschStats->Save(psschStatsParams);
}

/**
 * \brief Method to listen the trace RxPscchTraceUe of NrSpectrumPhy, which gets
 *        triggered upon the reception of SCI format 1-A.
 *
 * \param pscchStats Pointer to the UePhyPscchRxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param pscchStatsParams Parameters of the trace source.
 */
void
NotifySlPscchRx(UePhyPscchRxOutputStats* pscchStats,
                const SlRxCtrlPacketTraceParams pscchStatsParams)
{
    pscchStats->Save(pscchStatsParams);
}

/**
 * \brief Method to listen the trace RxPsschTraceUe of NrSpectrumPhy, which gets
 *        triggered upon the reception of SCI format 2-A and data.
 *
 * \param psschStats Pointer to the UePhyPsschRxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param psschStatsParams Parameters of the trace source.
 */
void
NotifySlPsschRx(UePhyPsschRxOutputStats* psschStats,
                const SlRxDataPacketTraceParams psschStatsParams)
{
    psschStats->Save(psschStatsParams);
}

/**
 * \brief Method to listen the application level traces of type TxWithAddresses
 *        and RxWithAddresses.
 * \param stats Pointer to the UeToUePktTxRxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param node The pointer to the TX or RX node
 * \param localAddrs The local IPV4 address of the node
 * \param txRx The string indicating the type of node, i.e., TX or RX
 * \param p The packet
 * \param srcAddrs The source address from the trace
 * \param dstAddrs The destination address from the trace
 * \param seqTsSizeHeader The SeqTsSizeHeader
 */
void
UePacketTraceDb(UeToUePktTxRxOutputStats* stats,
                Ptr<Node> node,
                const Address& localAddrs,
                std::string txRx,
                Ptr<const Packet> p,
                const Address& srcAddrs,
                const Address& dstAddrs,
                const SeqTsSizeHeader& seqTsSizeHeader)
{
    uint32_t nodeId = node->GetId();
    uint64_t imsi = node->GetDevice(0)->GetObject<NrUeNetDevice>()->GetImsi();
    uint32_t seq = seqTsSizeHeader.GetSeq();
    uint32_t pktSize = p->GetSize() + seqTsSizeHeader.GetSerializedSize();

    stats->Save(txRx, localAddrs, nodeId, imsi, pktSize, srcAddrs, dstAddrs, seq);
}

/**
 * \brief Trace sink for RxRlcPduWithTxRnti trace of NrUeMac
 * \param stats Pointer to UeRlcRxOutputStats API responsible to write the
 *        information communicated by this trace into a database.
 * \param imsi The IMSI of the UE
 * \param rnti The RNTI of the UE
 * \param txRnti The RNTI of the TX UE
 * \param lcid The logical channel id
 * \param rxPduSize The received PDU size
 * \param delay The end-to-end, i.e., from TX RLC entity to RX
 *        RLC entity, delay in Seconds.
 */
void
NotifySlRlcPduRx(UeRlcRxOutputStats* stats,
                 uint64_t imsi,
                 uint16_t rnti,
                 uint16_t txRnti,
                 uint8_t lcid,
                 uint32_t rxPduSize,
                 double delay)
{
    stats->Save(imsi, rnti, txRnti, lcid, rxPduSize, delay);
}

/**
 * \brief Create numRobots nodes moving by random waypoint inside a
 *        areaSize x areaSize square at constant speed (no pause).
 */
NodeContainer
InstallRobotMobility(uint16_t numRobots,
                     double areaSize,
                     double speed,
                     double antennaHeight,
                     bool external)
{
    NodeContainer ueNodes;
    ueNodes.Create(numRobots);
    std::cout << "Total UEs = " << ueNodes.GetN() << std::endl;

    std::ostringstream xy;
    xy << "ns3::UniformRandomVariable[Min=0.0|Max=" << areaSize << "]";
    Ptr<RandomRectanglePositionAllocator> posAlloc =
        CreateObject<RandomRectanglePositionAllocator>();
    posAlloc->SetAttribute("X", StringValue(xy.str()));
    posAlloc->SetAttribute("Y", StringValue(xy.str()));
    posAlloc->SetAttribute("Z", DoubleValue(antennaHeight));

    std::ostringstream spd;
    spd << "ns3::ConstantRandomVariable[Constant=" << speed << "]";
    MobilityHelper mobility;
    mobility.SetPositionAllocator(posAlloc);
    if (external)
    {
        // positions come from the co-simulation gateway (ROS 2); the allocator only
        // provides a starting point until the first update arrives
        mobility.SetMobilityModel("ns3::ExternalMobilityModel");
        mobility.Install(ueNodes);
        return ueNodes;
    }
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                              "Speed", StringValue(spd.str()),
                              "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"),
                              "PositionAllocator", PointerValue(posAlloc));
    mobility.Install(ueNodes);
    return ueNodes;
}

/**
 * \brief Get sidelink bitmap from string
 * \param slBitMapString The sidelink bitmap string
 * \param slBitMapVector The vector passed to store the converted sidelink bitmap
 */
void
GetSlBitmapFromString(std::string slBitMapString, std::vector<std::bitset<1>>& slBitMapVector)
{
    static std::unordered_map<std::string, uint8_t> lookupTable = {
        {"0", 0},
        {"1", 1},
    };

    std::stringstream ss(slBitMapString);
    std::string token;
    std::vector<std::string> extracted;

    while (std::getline(ss, token, '|'))
    {
        extracted.push_back(token);
    }

    for (const auto& v : extracted)
    {
        if (lookupTable.find(v) == lookupTable.end())
        {
            NS_FATAL_ERROR("Bit type " << v << " not valid. Valid values are: 0 and 1");
        }
        slBitMapVector.emplace_back(lookupTable[v] & 0x01);
    }
}

/**
 * \brief Save position of the UE as per its IP address
 * \param v2xKpi pointer to the V2xKpi API storing the IP of an UE and its position.
 */
void
SavePositionPerIP(V2xKpi* v2xKpi)
{
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<NrUeNetDevice> uedev = node->GetDevice(j)->GetObject<NrUeNetDevice>();
            if (uedev)
            {
                Ptr<Ipv4L3Protocol> ipv4Protocol = node->GetObject<Ipv4L3Protocol>();
                Ipv4InterfaceAddress addresses = ipv4Protocol->GetAddress(1, 0);
                std::ostringstream ueIpv4Addr;
                ueIpv4Addr.str("");
                ueIpv4Addr << addresses.GetLocal();
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                v2xKpi->FillPosPerIpMap(ueIpv4Addr.str(), pos);
            }
        }
    }
}

int
main(int argc, char* argv[])
{
    /*
     * Variables that represent the parameters we will accept as input by the
     * command line. Each of them is initialized with a default value.
     */
    uint16_t numRobots = 10;
    double areaSize = 50;        // meters, square side
    double speed = 1.5;          // meters per second
    double antennaHeight = 0.5;  // meters
    std::string scenario = "V2V_Urban"; // channel scenario, see BandwidthPartInfo::Scenario
    uint16_t msgPeriod = 100;    // ms between state messages per robot
    bool logging = false;
    bool harqEnabled = true;
    uint16_t pdb = 0;      // packet delay budget in ms; 0 = derive from T2
    bool dynamic = false;  // dynamic (per-packet) grants instead of SPS
    double slotFraction = 1.0; // <1: NrSlUeMacSchedulerEarliest restricted to the earliest fraction of candidate slots
    uint16_t cosimPort = 0;    // >0: co-simulate with a ROS 2 bridge listening on this TCP port (localhost)

    // Traffic parameters (that we will use inside this script:)
    bool useIPv6 = false; // default IPV4
    uint32_t udpPacketSizeBe = 200;

    // Simulation parameters.
    Time simTime = Seconds(10);
    // Sidelink bearers activation time
    Time slBearersActivationTime = Seconds(2.0);

    // NR parameters. We will take the input from the command line, and then we
    // will pass them inside the NR module.
    double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
    uint16_t bandwidthBandSl = 400;         // Multiple of 100 KHz; 400 = 40 MHz
    double txPower = 23;                    // dBm
    std::string tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";
    std::string slBitMap = "1|1|1|1|1|1|0|0|0|1|1|1";
    uint16_t numerologyBwpSl = 0;
    uint16_t slSensingWindow = 100; // T0 in ms
    uint16_t slSelectionWindow = 5; // T2min
    uint16_t slSubchannelSize = 50;
    uint16_t slMaxNumPerReserve = 3;
    double slProbResourceKeep = 0.0;
    uint16_t slMaxTxTransNumPssch = 5;
    uint16_t reservationPeriod = 0; // in ms; 0 = msgPeriod
    bool enableSensing = false;
    uint16_t t1 = 2;
    uint16_t t2 = 33;
    int slThresPsschRsrp = -128;
    bool enableChannelRandomness = false;
    uint16_t channelUpdatePeriod = 500; // ms
    uint8_t mcs = 14;

    // flags to generate gnuplot plotting scripts

    // Where we will store the output files.
    std::string simTag = "default";
    std::string outputDir = "./";

    /*
     * From here, we instruct the ns3::CommandLine class of all the input parameters
     * that we may accept as input, as well as their description, and the storage
     * variable.
     */
    CommandLine cmd(__FILE__);

    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("numRobots", "Number of robots (all transmit and receive)", numRobots);
    cmd.AddValue("areaSize", "Side of the square area in meters", areaSize);
    cmd.AddValue("speed", "Robot speed in m/s", speed);
    cmd.AddValue("antennaHeight", "Antenna height in meters", antennaHeight);
    cmd.AddValue("scenario", "Channel scenario: V2V_Urban | V2V_Highway | InH_OfficeOpen | UMi_StreetCanyon", scenario);
    cmd.AddValue("msgPeriod", "State message period per robot in ms", msgPeriod);
    cmd.AddValue("pdb", "Packet delay budget in ms (0 = from T2)", pdb);
    cmd.AddValue("dynamic", "Use dynamic grants instead of semi-persistent scheduling", dynamic);
    cmd.AddValue("slotFraction", "Earliest fraction of candidate slots to select from (1 = stock scheduler)", slotFraction);
    cmd.AddValue("cosimPort", "TCP port of the ROS 2 co-simulation bridge on localhost (0 = standalone run)", cosimPort);
    cmd.AddValue("useIPv6", "Use IPv6 instead of IPv4", useIPv6);
    cmd.AddValue("packetSizeBe",
                 "packet size in bytes to be used by best effort traffic",
                 udpPacketSizeBe);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("slBearerActivationTime",
                 "Sidelik bearer activation time in seconds",
                 slBearersActivationTime);
    cmd.AddValue("centralFrequencyBandSl",
                 "The central frequency to be used for Sidelink band/channel",
                 centralFrequencyBandSl);
    cmd.AddValue("bandwidthBandSl",
                 "The system bandwidth to be used for Sidelink",
                 bandwidthBandSl);
    cmd.AddValue("txPower", "total tx power in dBm", txPower);
    cmd.AddValue("tddPattern", "The TDD pattern string", tddPattern);
    cmd.AddValue("slBitMap", "The Sidelink bitmap string", slBitMap);
    cmd.AddValue("numerologyBwpSl",
                 "The numerology to be used in Sidelink bandwidth part",
                 numerologyBwpSl);
    cmd.AddValue("slSensingWindow", "The Sidelink sensing window length in ms", slSensingWindow);
    cmd.AddValue("slSelectionWindow",
                 "The parameter which decides the minimum Sidelink selection "
                 "window length in physical slots. T2min = slSelectionWindow * 2^numerology",
                 slSelectionWindow);
    cmd.AddValue("slSubchannelSize", "The Sidelink subchannel size in RBs", slSubchannelSize);
    cmd.AddValue("slMaxNumPerReserve",
                 "The parameter which indicates the maximum number of reserved "
                 "PSCCH/PSSCH resources that can be indicated by an SCI.",
                 slMaxNumPerReserve);
    cmd.AddValue("slProbResourceKeep",
                 "The parameter which indicates the probability with which the "
                 "UE keeps the current resource when the resource reselection"
                 "counter reaches zero.",
                 slProbResourceKeep);
    cmd.AddValue("slMaxTxTransNumPssch",
                 "The parameter which indicates the maximum transmission number "
                 "(including new transmission and retransmission) for PSSCH.",
                 slMaxTxTransNumPssch);
    cmd.AddValue("enableSensing",
                 "If true, it enables the sensing based resource selection for "
                 "SL, otherwise, no sensing is applied",
                 enableSensing);
    cmd.AddValue("t1",
                 "The start of the selection window in physical slots, "
                 "accounting for physical layer processing delay",
                 t1);
    cmd.AddValue("t2", "The end of the selection window in physical slots", t2);
    cmd.AddValue("slThresPsschRsrp",
                 "A threshold in dBm used for sensing based UE autonomous resource selection",
                 slThresPsschRsrp);
    cmd.AddValue("enableChannelRandomness",
                 "Enable shadowing and channel updates",
                 enableChannelRandomness);
    cmd.AddValue("channelUpdatePeriod", "The channel update period in ms", channelUpdatePeriod);
    cmd.AddValue("mcs", "The MCS to used for sidelink", mcs);
    cmd.AddValue("outputDir", "directory where to store simulation results", outputDir);
    cmd.AddValue("simTag", "tag identifying the simulation campaigns", simTag);
    cmd.AddValue("reservationPeriod", "SPS resource reservation interval in ms (0 = msgPeriod)", reservationPeriod);
    cmd.Parse(argc, argv);

    if (reservationPeriod == 0)
    {
        reservationPeriod = msgPeriod;
    }
    // TS 38.214 §8.1.4: the selection window [n+T1, n+T2] must lie inside one
    // reservation period (the MAC aborts otherwise). Clamp T2 when the period is short.
    uint16_t maxT2 = reservationPeriod * (1 << numerologyBwpSl) - t1 - 1;
    if (t2 > maxT2)
    {
        std::cout << "Clamping T2 from " << t2 << " to " << maxT2 << " slots to fit the "
                  << reservationPeriod << " ms reservation period" << std::endl;
        t2 = maxT2;
    }
    // one packet per msgPeriod -> CBR rate in kb/s
    double dataRateBe = udpPacketSizeBe * 8.0 / msgPeriod;
    Time delayBudget = MilliSeconds(pdb); // 0 => use T2 configuration (upstream semantics)

    std::map<std::string, BandwidthPartInfo::Scenario> scenarios = {
        {"V2V_Urban", BandwidthPartInfo::V2V_Urban},
        {"V2V_Highway", BandwidthPartInfo::V2V_Highway},
        {"InH_OfficeOpen", BandwidthPartInfo::InH_OfficeOpen},
        {"UMi_StreetCanyon", BandwidthPartInfo::UMi_StreetCanyon},
    };
    NS_ABORT_MSG_IF(scenarios.count(scenario) == 0, "Unknown scenario " << scenario);

    /*
     * Check if the frequency is in the allowed range.
     * If you need to add other checks, here is the best position to put them.
     */
    NS_ABORT_IF(centralFrequencyBandSl > 6e9);

    /*
     * If the logging variable is set to true, enable the log of some components
     * through the code. The same effect can be obtained through the use
     * of the NS_LOG environment variable:
     *
     * export NS_LOG="OnOffApplication=level_all|prefix_time|prefix_func|prefix_node:PacketSink=..."
     *
     * Usually, the environment variable way is preferred, as it is more customizable,
     * and more expressive.
     */
    if (logging)
    {
        LogLevel logLevel =
            (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL);
        LogComponentEnable("OnOffApplication", logLevel);
        LogComponentEnable("PacketSink", logLevel);
        LogComponentEnable("LtePdcp", logLevel);
        LogComponentEnable("NrSlHelper", logLevel);
        LogComponentEnable("NrSlUeRrc", logLevel);
        LogComponentEnable("NrUePhy", logLevel);
        LogComponentEnable("NrSpectrumPhy", logLevel);
    }

    /*
     * Default values for the simulation.
     */
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    /*
     * Create a NodeContainer for all the UEs
     */
    NodeContainer allSlUesContainer;

    /*
     * Assign mobility to the UEs.
     *  1. Set mobility model type.
     *  2. Assign position to the UEs
     *  3. Install mobility model
     */
    const bool cosim = cosimPort > 0;
    allSlUesContainer =
        InstallRobotMobility(numRobots, areaSize, speed, antennaHeight, cosim);
    if (scenario == "V2V_Urban")
    {
        // The V2V urban channel-condition model looks up MobilityBuildingInfo on each
        // node (it segfaults without it). No buildings are defined, so this is an open
        // area with TR 37.885 LOS / NLOSv (blocked-by-other-robots) statistics.
        BuildingsHelper::Install(allSlUesContainer);
    }

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    // Put the pointers inside nrHelper
    nrHelper->SetEpcHelper(epcHelper);

    /*
     * Spectrum division. We create one operational band, containing
     * one component carrier, and a single bandwidth part
     * centered at the frequency specified by the input parameters.
     * We will use the StreetCanyon channel modeling.
     */
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    /* Create the configuration for the CcBwpHelper. SimpleOperationBandConf
     * creates a single BWP per CC
     */
    CcBwpCreator::SimpleOperationBandConf bandConfSl(centralFrequencyBandSl,
                                                     bandwidthBandSl,
                                                     numCcPerBand,
                                                     scenarios.at(scenario));

    // By using the configuration created, it is time to make the operation bands
    OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc(bandConfSl);

    /*
     * The configured spectrum division is:
     * ------------Band1--------------
     * ------------CC1----------------
     * ------------BwpSl--------------
     */
    if (enableChannelRandomness)
    {
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                           TimeValue(MilliSeconds(channelUpdatePeriod)));
        nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                    TimeValue(MilliSeconds(channelUpdatePeriod)));
        nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    }
    else
    {
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
        nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
        nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    }

    /*
     * Initialize channel and pathloss, plus other things inside bandSl. If needed,
     * the band configuration can be done manually, but we leave it for more
     * sophisticated examples. For the moment, this method will take care
     * of all the spectrum initialization needs.
     */
    nrHelper->InitializeOperationBand(&bandSl);
    allBwps = CcBwpCreator::GetAllBwps({bandSl});

    /*
     * Now, we can setup the attributes. We can have three kind of attributes:
     */

    /*
     * Antennas for all the UEs
     * We are not using beamforming in SL, rather we are using
     * quasi-omnidirectional transmission and reception, which is the default
     * configuration of the beams.
     *
     * Following attribute would be common for all the UEs
     */
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPower));

    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(enableSensing));
    nrHelper->SetUeMacAttribute("T1", UintegerValue(static_cast<uint8_t>(t1)));
    nrHelper->SetUeMacAttribute("T2", UintegerValue(t2));
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));
    nrHelper->SetUeMacAttribute("SlThresPsschRsrp", IntegerValue(slThresPsschRsrp));

    uint8_t bwpIdForGbrMcptt = 0;

    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    // following parameter has no impact at the moment because:
    // 1. No support for PQI based mapping between the application and the LCs
    // 2. No scheduler to consider PQI
    // However, till such time all the NR SL examples should use GBR_MC_PUSH_TO_TALK
    // because we hard coded the PQI 65 in UE RRC.
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdForGbrMcptt));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdForGbrMcptt);

    /*
     * We have configured the attributes we needed. Now, install and get the pointers
     * to the NetDevices, which contains all the NR stack:
     */
    NetDeviceContainer allSlUesNetDeviceContainer =
        nrHelper->InstallUeDevice(allSlUesContainer, allBwps);

    // When all the configuration is done, explicitly call UpdateConfig ()
    for (auto it = allSlUesNetDeviceContainer.Begin(); it != allSlUesNetDeviceContainer.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    /*
     * Configure Sidelink. We create the following helpers needed for the
     * NR Sidelink, i.e., V2X simulation:
     * - NrSlHelper, which will configure the UEs protocol stack to be ready to
     *   perform Sidelink related procedures.
     * - EpcHelper, which takes care of triggering the call to EpcUeNas class
     *   to establish the NR Sidelink bearer(s). We note that, at this stage
     *   just communicate the pointer of already instantiated EpcHelper object,
     *   which is the same pointer communicated to the NrHelper above.
     */
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    // Put the pointers inside NrSlHelper
    nrSlHelper->SetEpcHelper(epcHelper);

    /*
     * Set the SL error model and AMC
     * Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1,
     *                   ns3::NrEesmIrT2, ns3::NrLteMiErrorModel
     * AMC type: NrAmc::ShannonModel or NrAmc::ErrorModel
     */
    std::string errorModel = "ns3::NrEesmIrT1";
    nrSlHelper->SetSlErrorModel(errorModel);
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    /*
     * Set the SL scheduler attributes
     * In this example we use NrSlUeMacSchedulerSimple scheduler, which uses
     * a fixed MCS value
     */
    if (slotFraction < 1.0)
    {
        nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerEarliest::GetTypeId());
        nrSlHelper->SetUeSlSchedulerAttribute("SlotFraction", DoubleValue(slotFraction));
    }
    else
    {
        nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    }
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(mcs));

    /*
     * Very important method to configure UE protocol stack, i.e., it would
     * configure all the SAPs among the layers, setup callbacks, configure
     * error model, configure AMC, and configure ChunkProcessor in Interference
     * API.
     */
    nrSlHelper->PrepareUeForSidelink(allSlUesNetDeviceContainer, bwpIdContainer);

    /*
     * Start preparing for all the sub Structs/RRC Information Element (IEs)
     * of LteRrcSap::SidelinkPreconfigNr. This is the main structure, which would
     * hold all the pre-configuration related to Sidelink.
     */

    // SlResourcePoolNr IE
    LteRrcSap::SlResourcePoolNr slResourcePoolNr;
    // get it from pool factory
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();
    /*
     * Above pool factory is created to help the users of the simulator to create
     * a pool with valid default configuration. Please have a look at the
     * constructor of NrSlCommResourcePoolFactory class.
     *
     * In the following, we show how one could change those default pool parameter
     * values as per the need.
     */
    std::vector<std::bitset<1>> slBitMapVector;
    GetSlBitmapFromString(slBitMap, slBitMapVector);
    NS_ABORT_MSG_IF(slBitMapVector.empty(), "GetSlBitmapFromString failed to generate SL bitmap");
    ptrFactory->SetSlTimeResources(slBitMapVector);
    ptrFactory->SetSlSensingWindow(slSensingWindow); // T0 in ms
    ptrFactory->SetSlSelectionWindow(slSelectionWindow);
    ptrFactory->SetSlFreqResourcePscch(10); // PSCCH RBs
    ptrFactory->SetSlSubchannelSize(slSubchannelSize);
    ptrFactory->SetSlMaxNumPerReserve(slMaxNumPerReserve);
    std::list<uint16_t> resourceReservePeriodList = {0, reservationPeriod}; // in ms
    ptrFactory->SetSlResourceReservePeriodList(resourceReservePeriodList);
    // Once parameters are configured, we can create the pool
    LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool();
    slResourcePoolNr = pool;

    // Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
    LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
    slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
    // Pool id, ranges from 0 to 15
    uint16_t poolId = 0;
    LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
    slResourcePoolIdNr.id = poolId;
    slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
    slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

    // Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
    // Array for pools, we insert the pool in the array as per its poolId
    slBwpPoolConfigCommonNr.slTxPoolSelectedNormal[slResourcePoolIdNr.id] = slresoPoolConfigNr;

    // Configure the BWP IE
    LteRrcSap::Bwp bwp;
    bwp.numerology = numerologyBwpSl;
    bwp.symbolsPerSlots = 14;
    bwp.rbPerRbg = 1;
    bwp.bandwidth = bandwidthBandSl;

    // Configure the SlBwpGeneric IE
    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp = bwp;
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);

    // Configure the SlBwpConfigCommonNr IE
    LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
    slBwpConfigCommonNr.haveSlBwpGeneric = true;
    slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
    slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
    slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

    // Configure the SlFreqConfigCommonNr IE, which hold the array to store
    // the configuration of all Sidelink BWP (s).
    LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
    // Array for BWPs. Here we will iterate over the BWPs, which
    // we want to use for SL.
    for (const auto& it : bwpIdContainer)
    {
        // it is the BWP id
        slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
    }

    // Configure the TddUlDlConfigCommon IE
    LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
    tddUlDlConfigCommon.tddPattern = tddPattern;

    // Configure the SlPreconfigGeneralNr IE
    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    // Configure the SlUeSelectedConfig IE
    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    NS_ABORT_MSG_UNLESS(slProbResourceKeep <= 1.0,
                        "slProbResourceKeep value must be between 0 and 1");
    slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
    // Configure the SlPsschTxParameters IE
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = static_cast<uint8_t>(slMaxTxTransNumPssch);
    // Configure the SlPsschTxConfigList IE
    LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
    pscchTxConfigList.slPsschTxParameters[0] = psschParams;
    slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

    /*
     * Finally, configure the SidelinkPreconfigNr. This is the main structure
     * that needs to be communicated to NrSlUeRrc class
     */
    LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
    slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
    slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
    slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;

    // Communicate the above pre-configuration to the NrSlHelper
    nrSlHelper->InstallNrSlPreConfiguration(allSlUesNetDeviceContainer, slPreConfigNr);

    /****************************** End SL Configuration ***********************/

    /*
     * Fix the random streams
     */
    int64_t stream = 1;
    stream += nrHelper->AssignStreams(allSlUesNetDeviceContainer, stream);
    stream += nrSlHelper->AssignStreams(allSlUesNetDeviceContainer, stream);

    NodeContainer txSlUes;
    NodeContainer rxSlUes;
    NetDeviceContainer txSlUesNetDevice;
    NetDeviceContainer rxSlUesNetDevice;
    // every robot both transmits its own state and receives everyone else's
    txSlUes.Add(allSlUesContainer);
    rxSlUes.Add(allSlUesContainer);
    txSlUesNetDevice.Add(allSlUesNetDeviceContainer);
    rxSlUesNetDevice.Add(allSlUesNetDeviceContainer);

    /*
     * Configure the IP stack, and activate NR Sidelink bearer (s) as per the
     * configured time.
     *
     * This example supports IPV4 and IPV6
     */

    InternetStackHelper internet;
    internet.Install(allSlUesContainer);
    stream += internet.AssignStreams(allSlUesContainer, stream);
    uint32_t dstL2Id = 255;
    Ipv4Address groupAddress4("225.0.0.0"); // use multicast address as destination
    Ipv6Address groupAddress6("ff0e::1");   // use multicast address as destination
    Address remoteAddress;
    Address localAddress;
    uint16_t port = 8000;
    Ptr<LteSlTft> tft;
    SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id = dstL2Id;
    slInfo.m_rri = MilliSeconds(reservationPeriod);
    slInfo.m_dynamic = dynamic;
    slInfo.m_pdb = delayBudget;
    slInfo.m_harqEnabled = harqEnabled;
    if (!useIPv6)
    {
        Ipv4InterfaceContainer ueIpIface;
        ueIpIface = epcHelper->AssignUeIpv4Address(allSlUesNetDeviceContainer);

        // set the default gateway for the UE
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        for (uint32_t u = 0; u < allSlUesContainer.GetN(); ++u)
        {
            Ptr<Node> ueNode = allSlUesContainer.Get(u);
            // Set the default gateway for the UE
            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
        }
        remoteAddress = InetSocketAddress(groupAddress4, port);
        localAddress = InetSocketAddress(Ipv4Address::GetAny(), port);

        tft = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, groupAddress4, slInfo);
        // Set Sidelink bearers
        nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, txSlUesNetDevice, tft);

        tft = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, groupAddress4, slInfo);
        // Set Sidelink bearers
        nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, rxSlUesNetDevice, tft);
    }
    else
    {
        Ipv6InterfaceContainer ueIpIface;
        ueIpIface = epcHelper->AssignUeIpv6Address(allSlUesNetDeviceContainer);

        // set the default gateway for the UE
        Ipv6StaticRoutingHelper ipv6RoutingHelper;
        for (uint32_t u = 0; u < allSlUesContainer.GetN(); ++u)
        {
            Ptr<Node> ueNode = allSlUesContainer.Get(u);
            // Set the default gateway for the UE
            Ptr<Ipv6StaticRouting> ueStaticRouting =
                ipv6RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv6>());
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress6(), 1);
        }
        remoteAddress = Inet6SocketAddress(groupAddress6, port);
        localAddress = Inet6SocketAddress(Ipv6Address::GetAny(), port);

        tft = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, groupAddress4, slInfo);
        // Set Sidelink bearers for transmitting UEs
        nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, txSlUesNetDevice, tft);

        tft = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, groupAddress4, slInfo);
        nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, rxSlUesNetDevice, tft);
    }

    /*
     * Configure the applications:
     * Client app: OnOff application configure to generate CBR traffic
     * Server app: PacketSink application.
     */
    // Random variable to randomize a bit start times of the client applications
    // to avoid simulation artifacts of all the TX UEs transmitting at the same time.
    Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable>();
    startTimeSeconds->SetStream(stream);
    startTimeSeconds->SetAttribute("Min", DoubleValue(0));
    startTimeSeconds->SetAttribute("Max", DoubleValue(0.10));

    // Set Application in the UEs
    OnOffHelper sidelinkClient("ns3::UdpSocketFactory", remoteAddress);
    sidelinkClient.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    std::string dataRateBeString = std::to_string(dataRateBe) + "kb/s";
    std::cout << "Data rate " << DataRate(dataRateBeString) << std::endl;
    sidelinkClient.SetConstantRate(DataRate(dataRateBeString), udpPacketSizeBe);

    ApplicationContainer clientApps;
    double realAppStart = 0.0;
    double realAppStopTime = 0.0;
    double txAppDuration = 0.0;

    // Co-simulation: the ROS 2 side decides when each robot emits a state message,
    // so install a TriggeredSendApplication (fired by RobotGateway) instead of OnOff.
    TriggeredSendHelper triggeredClient("ns3::UdpSocketFactory", remoteAddress);
    triggeredClient.SetAttribute("PacketSize", UintegerValue(udpPacketSizeBe));

    for (uint32_t i = 0; i < txSlUes.GetN(); i++)
    {
        if (cosim)
        {
            clientApps.Add(triggeredClient.Install(txSlUes.Get(i)));
            clientApps.Get(i)->SetStartTime(slBearersActivationTime);
            realAppStart = slBearersActivationTime.GetSeconds();
            continue;
        }
        clientApps.Add(sidelinkClient.Install(txSlUes.Get(i)));
        double jitter = startTimeSeconds->GetValue();
        Time appStart = slBearersActivationTime + Seconds(jitter);
        clientApps.Get(i)->SetStartTime(appStart);
        // onoff application will send the first packet at :
        // slBearersActivationTime + random jitter + ((Pkt size in bits) / (Data rate in bits per
        // sec))
        realAppStart = slBearersActivationTime.GetSeconds() + jitter +
                       ((double)udpPacketSizeBe * 8.0 / (DataRate(dataRateBeString).GetBitRate()));
        realAppStopTime = realAppStart + simTime.GetSeconds();
        clientApps.Get(i)->SetStopTime(Seconds(realAppStopTime));
        txAppDuration = realAppStopTime - realAppStart;

        // Output app start, stop and duration
        std::cout << "Tx App " << i + 1 << " start time " << std::fixed << std::setprecision(5)
                  << realAppStart << " sec" << std::endl;
        std::cout << "Tx App " << i + 1 << " stop time " << realAppStopTime << " sec" << std::endl;
        std::cout << "Tx App duration " << std::defaultfloat << txAppDuration << " sec"
                  << std::endl;
    }

    ApplicationContainer serverApps;
    PacketSinkHelper sidelinkSink("ns3::UdpSocketFactory", localAddress);
    sidelinkSink.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    for (uint32_t i = 0; i < rxSlUes.GetN(); i++)
    {
        serverApps.Add(sidelinkSink.Install(rxSlUes.Get(i)));
        serverApps.Start(Seconds(0.0));
    }

    /*
     * Hook the traces, for trace data to be stored in a database
     */
    std::string exampleName = simTag + "-" + "sidewalk-robots";
    // Datebase setup
    SQLiteOutput db(outputDir + exampleName + ".db");

    UeMacPscchTxOutputStats pscchStats;
    pscchStats.SetDb(&db, "pscchTxUeMac");
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                  "ComponentCarrierMapUe/*/NrUeMac/SlPscchScheduling",
                                  MakeBoundCallback(&NotifySlPscchScheduling, &pscchStats));

    UeMacPsschTxOutputStats psschStats;
    psschStats.SetDb(&db, "psschTxUeMac");
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                  "ComponentCarrierMapUe/*/NrUeMac/SlPsschScheduling",
                                  MakeBoundCallback(&NotifySlPsschScheduling, &psschStats));

    UePhyPscchRxOutputStats pscchPhyStats;
    pscchPhyStats.SetDb(&db, "pscchRxUePhy");
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
        "SpectrumPhy/RxPscchTraceUe",
        MakeBoundCallback(&NotifySlPscchRx, &pscchPhyStats));

    UePhyPsschRxOutputStats psschPhyStats;
    psschPhyStats.SetDb(&db, "psschRxUePhy");
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
        "SpectrumPhy/RxPsschTraceUe",
        MakeBoundCallback(&NotifySlPsschRx, &psschPhyStats));

    UeRlcRxOutputStats ueRlcRxStats;
    ueRlcRxStats.SetDb(&db, "rlcRx");
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                  "ComponentCarrierMapUe/*/NrUeMac/RxRlcPduWithTxRnti",
                                  MakeBoundCallback(&NotifySlRlcPduRx, &ueRlcRxStats));

    UeToUePktTxRxOutputStats pktStats;
    pktStats.SetDb(&db, "pktTxRx");

    if (cosim)
    {
        // no SeqTsSize header on triggered packets; deliveries are reported to ROS 2
        // by RobotGateway (matched on packet UID) instead of the pktTxRx table
    }
    else if (!useIPv6)
    {
        // Set Tx traces
        for (uint32_t ac = 0; ac < clientApps.GetN(); ac++)
        {
            Ipv4Address localAddrs = clientApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv4L3Protocol>()
                                         ->GetAddress(1, 0)
                                         .GetLocal();
            std::cout << "Tx address: " << localAddrs << std::endl;
            clientApps.Get(ac)->TraceConnect("TxWithSeqTsSize",
                                             "tx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               clientApps.Get(ac)->GetNode(),
                                                               localAddrs));
        }

        // Set Rx traces
        for (uint32_t ac = 0; ac < serverApps.GetN(); ac++)
        {
            Ipv4Address localAddrs = serverApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv4L3Protocol>()
                                         ->GetAddress(1, 0)
                                         .GetLocal();
            std::cout << "Rx address: " << localAddrs << std::endl;
            serverApps.Get(ac)->TraceConnect("RxWithSeqTsSize",
                                             "rx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               serverApps.Get(ac)->GetNode(),
                                                               localAddrs));
        }
    }
    else
    {
        // Set Tx traces
        for (uint32_t ac = 0; ac < clientApps.GetN(); ac++)
        {
            clientApps.Get(ac)->GetNode()->GetObject<Ipv6L3Protocol>()->AddMulticastAddress(
                groupAddress6);
            Ipv6Address localAddrs = clientApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv6L3Protocol>()
                                         ->GetAddress(1, 1)
                                         .GetAddress();
            std::cout << "Tx address: " << localAddrs << std::endl;
            clientApps.Get(ac)->TraceConnect("TxWithSeqTsSize",
                                             "tx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               clientApps.Get(ac)->GetNode(),
                                                               localAddrs));
        }

        // Set Rx traces
        for (uint32_t ac = 0; ac < serverApps.GetN(); ac++)
        {
            serverApps.Get(ac)->GetNode()->GetObject<Ipv6L3Protocol>()->AddMulticastAddress(
                groupAddress6);
            Ipv6Address localAddrs = serverApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv6L3Protocol>()
                                         ->GetAddress(1, 1)
                                         .GetAddress();
            std::cout << "Rx address: " << localAddrs << std::endl;
            serverApps.Get(ac)->TraceConnect("RxWithSeqTsSize",
                                             "rx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               serverApps.Get(ac)->GetNode(),
                                                               localAddrs));
        }
    }

    V2xKpi v2xKpi;
    v2xKpi.SetDbPath(outputDir + exampleName);
    v2xKpi.SetTxAppDuration(txAppDuration);
    SavePositionPerIP(&v2xKpi);
    v2xKpi.SetRangeForV2xKpis(areaSize * 1.5); // whole area: every robot is a neighbour

    SensingTraceSink sensingTrace(outputDir + exampleName + "-sensing.csv");
    sensingTrace.ConnectAll();

    Time simStopTime = simTime + slBearersActivationTime + Seconds(realAppStart);

    // Co-simulation: the ROS 2 bridge owns the clock (lock-step) and ends the run.
    // The gateway must outlive Simulator::Run and be connected after apps exist.
    std::unique_ptr<RobotGateway> gateway;
    if (cosim)
    {
        gateway = std::make_unique<RobotGateway>(allSlUesContainer,
                                                 slBearersActivationTime + MilliSeconds(100));
        gateway->Attach();
        std::cout << "Connecting to ROS 2 bridge on 127.0.0.1:" << cosimPort << std::endl;
        gateway->Connect("127.0.0.1", cosimPort);
    }
    else
    {
        Simulator::Stop(simStopTime);
    }
    Simulator::Run();

    /*
     * VERY IMPORTANT: Do not forget to empty the database cache, which would
     * dump the data store towards the end of the simulation in to a database.
     */
    pktStats.EmptyCache();
    pscchStats.EmptyCache();
    psschStats.EmptyCache();
    pscchPhyStats.EmptyCache();
    psschPhyStats.EmptyCache();
    ueRlcRxStats.EmptyCache();
    if (!cosim)
    {
        v2xKpi.WriteKpis(); // needs the pktTxRx table
    }

    // GtkConfigStore config;
    //  config.ConfigureAttributes ();

    Simulator::Destroy();
    return 0;
}
