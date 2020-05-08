#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <numeric>
#include <typeinfo>
#include <cmath>

#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/nstime.h"
#include "ns3/traffic-control-module.h"

using namespace ns3;

static std::list<int> RxTimes;
static std::list<int> bytesTotal;

static bool firstCwnd = true;
static bool firstSshThr = true;
static bool firstRtt = true;
static bool firstGoodput = true;
static Ptr<OutputStreamWrapper> cWndStream;
static Ptr<OutputStreamWrapper> rttStream;
static Ptr<OutputStreamWrapper> goodputStream;
static uint32_t cWndValue;
static uint32_t ssThreshValue;

static void CwndTracer (uint32_t oldval, uint32_t newval)
{
  if (firstCwnd)
    {
      *cWndStream->GetStream () << "0.0," << oldval << std::endl;
      firstCwnd = false;
    }
  *cWndStream->GetStream () << Simulator::Now ().GetSeconds () << "," << newval << std::endl;
  cWndValue = newval;

  if (!firstSshThr)
    {
    }
}

static void SsThreshTracer (uint32_t oldval, uint32_t newval)
{
  if (firstSshThr)
    {
      firstSshThr = false;
    }
  ssThreshValue = newval;

  if (!firstCwnd)
    {
      *cWndStream->GetStream () << Simulator::Now ().GetSeconds () << "," << cWndValue << std::endl;
    }
}

static void RttTracer (Time oldval, Time newval)
{
  if (firstRtt)
    {
      *rttStream->GetStream () << "0.0," << oldval.GetMilliSeconds () << std::endl;
      firstRtt = false;
    }
  *rttStream->GetStream () << Simulator::Now ().GetSeconds () << "," << newval.GetMilliSeconds () << std::endl;
}

static void SinkRx(Ptr<const Packet> p, const Address &ad)
{
  if(firstGoodput)
  {
    //std::cout<< "Throughput: " << "0.0," << "0.0" << std::endl;
    *goodputStream->GetStream () << "0.0," << "0.0" << std::endl;
    firstGoodput = false;
  }

  RxTimes.push_front(Simulator::Now().GetSeconds());
  bytesTotal.push_front(p->GetSize());

  int max = RxTimes.size();
  for (int i = 0; i < max; i++)
  {
    if((RxTimes.back() < RxTimes.front() - 10))
    {
      RxTimes.pop_back();
      bytesTotal.pop_back();
    }
    else
    {
      break;
    }
  }

  float localGoodput = std::accumulate(std::begin(bytesTotal), std::end(bytesTotal), 0.0)*8/(RxTimes.front() - RxTimes.back())/1024/1024;
  if(std::isfinite(localGoodput))
  {
    std::cout<< "Goodput: " << Simulator::Now ().GetSeconds () << "," << localGoodput << std::endl;
    *goodputStream->GetStream () << Simulator::Now ().GetSeconds () << "," << localGoodput << std::endl;
  }
}

static void TraceGoodput (std::string throughput_name)
{
  AsciiTraceHelper ascii;
  goodputStream = ascii.CreateFileStream (throughput_name.c_str ());
  Config::ConnectWithoutContext("/NodeList/63/ApplicationList/0/$ns3::PacketSink/Rx", MakeCallback (&SinkRx));
}

static void TraceCwnd (std::string cwnd_tr_file_name)
{

  AsciiTraceHelper ascii;
  cWndStream = ascii.CreateFileStream (cwnd_tr_file_name.c_str ());
  Config::ConnectWithoutContext ("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeCallback (&CwndTracer));
}

static void TraceRtt (std::string rtt_tr_file_name)
{
  AsciiTraceHelper ascii;
  rttStream = ascii.CreateFileStream (rtt_tr_file_name.c_str ());
  Config::ConnectWithoutContext ("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/RTT", MakeCallback (&RttTracer));
}

static void TraceSsThresh ()
{
  Config::ConnectWithoutContext ("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/SlowStartThreshold", MakeCallback (&SsThreshTracer));
}

NS_LOG_COMPONENT_DEFINE ("VideoLTE");

int main (int argc, char *argv[])
{

  std::string traceFile;
  std::string transport_prot = "None";
  std::string tcp_algorithm = "TcpWestwood";
  std::string simulation = "loaded";
  uint32_t    ueNodeNum = 0;
  uint32_t    enbNodeNum = 9;
  uint32_t    packet_size = 1100;
  double      distance = 473.75;
  double      duration;
  double      interPacketInterval = 3.5211;
  bool sack = true;
  uint32_t run = 1;
  std::string recovery = "ns3::TcpClassicRecovery";
  DataRate internet_bandwidth = DataRate("260Mbps");
  Time internet_delay = Seconds (0.010);

  // Parse command line attribute
  CommandLine cmd;
  cmd.AddValue ("traceFile", "Ns2 movement trace file", traceFile);
  cmd.AddValue ("ueNodeNum", "Number of ue nodes", ueNodeNum);
  cmd.AddValue ("duration", "Duration of Simulation", duration);
  cmd.AddValue ("tcp_algorithm", "Transport protocol to use: TcpNewReno, "
                "TcpHybla, TcpHighSpeed, TcpHtcp, TcpVegas, TcpScalable, TcpVeno, "
                "TcpBic, TcpYeah, TcpIllinois, TcpWestwood, TcpWestwoodPlus, TcpLedbat, "
                "TcpLp, TcpBbr", tcp_algorithm);
  cmd.AddValue ("run", "Run index (for setting repeatable seeds)", run);
  cmd.Parse (argc,argv);

  SeedManager::SetSeed (1);
  SeedManager::SetRun (run);

  Config::SetDefault("ns3::LteEnbMac::NumberOfRaPreambles", UintegerValue(10));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (80));
  Config::SetDefault ("ns3::LteEnbRrc::EpsBearerToRlcMapping", StringValue ("RlcAmAlways"));
  Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::LteHelper::UsePdschForCqiGeneration", BooleanValue (false));
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (46.0));
  Config::SetDefault ("ns3::LteEnbPhy::NoiseFigure", DoubleValue (5.0));
  Config::SetDefault ("ns3::LteUePhy::NoiseFigure", DoubleValue (7.0));
  Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (24.0));
  Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::MiErrorModel));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(100));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(100));

  Config::SetDefault ("ns3::LteUeNetDevice::DlEarfcn", UintegerValue (3100));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlEarfcn", UintegerValue (3100));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", UintegerValue (20750));


  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (packet_size));
  Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (6291456));
  Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (4194304));
  Config::SetDefault ("ns3::TcpSocketBase::Sack", BooleanValue (sack));

  //Set queues for the sources to be very large to avoid lost packet sin sources, later on I set enb queues to be realistic values
  Config::SetDefault ("ns3::QueueBase::MaxSize", QueueSizeValue (QueueSize ("20000p")));

  Config::SetDefault ("ns3::TcpL4Protocol::RecoveryType", TypeIdValue (TypeId::LookupByName (recovery)));

  Config::SetDefault ("ns3::LogNormalRandomVariable::Mu", DoubleValue(0.4026));
  Config::SetDefault ("ns3::LogNormalRandomVariable::Sigma", DoubleValue(0.0352));

  Config::SetDefault ("ns3::WeibullRandomVariable::Scale", DoubleValue(6.49702));
  Config::SetDefault ("ns3::WeibullRandomVariable::Shape", DoubleValue(10.2063));

  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  transport_prot = std::string ("ns3::") + tcp_algorithm;
  // Select TCP variant
  if (transport_prot.compare ("ns3::TcpWestwoodPlus") == 0)
    {
      // TcpWestwoodPlus is not an actual TypeId name; we need TcpWestwood here
      Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpWestwood::GetTypeId ()));
      // the default protocol type in ns3::TcpWestwood is WESTWOOD
      Config::SetDefault ("ns3::TcpWestwood::ProtocolType", EnumValue (TcpWestwood::WESTWOODPLUS));
    }
  else
    {
      TypeId tcpTid;
      NS_ABORT_MSG_UNLESS (TypeId::LookupByNameFailSafe (transport_prot, &tcpTid), "TypeId " << transport_prot << " not found");
      Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TypeId::LookupByName (transport_prot)));
    }

  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
  uv->SetStream (50);
  RateErrorModel error_model;
  error_model.SetRandomVariable (uv);
  error_model.SetUnit (RateErrorModel::ERROR_UNIT_PACKET);
  error_model.SetRate (0.01);

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  std::cout << "Id of remote host node: " << remoteHost->GetId() << std::endl;
  InternetStackHelper internet;
  internet.Install(remoteHost);

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  lteHelper->SetSchedulerType ("ns3::PfFfMacScheduler");    //
  lteHelper->SetHandoverAlgorithmType ("ns3::A2A4RsrqHandoverAlgorithm"); // set handover algorithm
  lteHelper->SetHandoverAlgorithmAttribute ("ServingCellThreshold",
                                             UintegerValue (30));
  lteHelper->SetHandoverAlgorithmAttribute ("NeighbourCellOffset",
                                             UintegerValue (1));


  lteHelper->SetAttribute ("FadingModel", StringValue ("ns3::TraceFadingLossModel"));
  lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue ("/home/paddyxvy/fading_trace_EVA_120kmph.fad"));

  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  std::cout << "Id of pgw node: " << pgw->GetId() << std::endl;

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (internet_bandwidth));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (internet_delay));
  p2ph.SetDeviceAttribute ("ReceiveErrorModel", PointerValue (&error_model));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  std::cout << "Ip address of remote host node: " << remoteHostAddr << std::endl;
  std::cout << "Ip address of pgw node        : " << internetIpIfaces.GetAddress (0) << std::endl;

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  //MOBILITY START
  // Give EPC nodes a mobility model
  Ptr<ListPositionAllocator> positionAllocEPC = CreateObject<ListPositionAllocator> ();
  for (uint16_t i = 0; i < ns3::NodeList::GetNNodes(); i++)
    {
      positionAllocEPC->Add (Vector (distance * i, 0, 0));
    }
  MobilityHelper mobility0;
  mobility0.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility0.SetPositionAllocator(positionAllocEPC);
  mobility0.InstallAll();

  // Create enbNodes and install const mobility on all existing nodes so far
  NodeContainer enbNodes;
  enbNodes.Create (enbNodeNum);

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (6996.18,3675.11, 0));
  //positionAlloc->Add (Vector (6996.18,3675.11, 0));
  positionAlloc->Add (Vector (6214.69,6988.19, 0));
  //positionAlloc->Add (Vector (6214.69,6988.19, 0));
  positionAlloc->Add (Vector (7540.65,7035.29, 0));
  //positionAlloc->Add (Vector (7580.32,5923.10, 0));
  positionAlloc->Add (Vector (7580.32,5923.10, 0));
  //positionAlloc->Add (Vector (2236.78,6849.12, 0));
  positionAlloc->Add (Vector (2236.78,6849.12, 0));
  //positionAlloc->Add (Vector (5083.86,1380.46, 0));
  positionAlloc->Add (Vector (5083.86,1380.46, 0));
  positionAlloc->Add (Vector (-3619.18,3310.10, 0));
  positionAlloc->Add (Vector (3756.33,1334.08, 0));
  positionAlloc->Add (Vector (4329.76,3952.77, 0));
  //positionAlloc->Add (Vector (4329.76,3952.77, 0));

  MobilityHelper mobility1;
  mobility1.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility1.SetPositionAllocator(positionAlloc);
  mobility1.Install(enbNodes);

  //Create ueNodes and install trace mobility on them
  NodeContainer ueNodes;
  ueNodes.Create (ueNodeNum);
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
  ns2.Install (ueNodes.Begin (), ueNodes.End ()); // configure movements for each node, while reading trace file
  Ptr<Node> testNode = ueNodes.Get (ueNodeNum - 1);
  std::cout << "Id of test node: " << testNode->GetId() << std::endl;
  //MOBILITY END

  NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueDevs = lteHelper->InstallUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));
  std::cout << "Ip address of test UE node: " << ueIpIface.GetAddress(ueNodeNum - 1) << std::endl;

  // Assign IP address to UEs
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
  {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
  }

  lteHelper->Attach(ueDevs);

  // Install and start applications on UEs and remote host
  uint16_t dlPort = 1100;

  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (interPacketInterval/1000.0));


  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      ApplicationContainer clientApps;
      ApplicationContainer serverApps;

      if ( u == (ueNodeNum - 1) )
        {
          OnOffHelper onoff ("ns3::TcpSocketFactory", Address (InetSocketAddress (Ipv4Address::GetAny (), dlPort)));
          onoff.SetAttribute ("Remote", AddressValue (InetSocketAddress(Ipv4Address (ueIpIface.GetAddress(u)), dlPort)));
          onoff.SetAttribute ("DataRate", DataRateValue (DataRate ("5000000bps")));
          onoff.SetAttribute ("PacketSize", UintegerValue (packet_size));
          onoff.SetAttribute ("OffTime", StringValue ("ns3::LogNormalRandomVariable"));
          onoff.SetAttribute ("OnTime", StringValue ("ns3::WeibullRandomVariable"));
          onoff.SetAttribute ("MaxBytes", UintegerValue (0));
          serverApps.Add ( onoff.Install (remoteHost));

          PacketSinkHelper sink ("ns3::TcpSocketFactory", Address (InetSocketAddress (Ipv4Address::GetAny (), dlPort)));
          sink.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
          sink.SetAttribute ("Local", AddressValue (InetSocketAddress(Ipv4Address (ueIpIface.GetAddress(u)), dlPort)));
          clientApps.Add ( sink.Install (ueNodes.Get(u)));
        }
      else
        {
          UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort);
          dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketInterval)));
          dlClient.SetAttribute ("MaxPackets", UintegerValue (85200));
          dlClient.SetAttribute ("RemoteAddress", AddressValue(InetSocketAddress(Ipv4Address (ueIpIface.GetAddress(u)), dlPort)));
          dlClient.SetAttribute ("PacketSize", UintegerValue(packet_size));
          serverApps.Add (dlClient.Install (remoteHost));

          PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
          clientApps.Add (dlPacketSinkHelper.Install (ueNodes.Get (u)));
        }
      serverApps.Start (Seconds (startTimeSeconds->GetValue ()));
      serverApps.Stop  (Seconds (duration));
      clientApps.Start (Seconds (0));
      clientApps.Stop  (Seconds (duration));
    }

  lteHelper->AddX2Interface (enbNodes);

  Config::Set ("NodeList/4/$ns3::Node/DeviceList/*/$ns3::PointToPointNetDevice/TxQueue/$ns3::DropTailQueue<Packet>/MaxSize",    QueueSizeValue (QueueSize ("750p")));
  Config::Set ("NodeList/5/$ns3::Node/DeviceList/*/$ns3::PointToPointNetDevice/TxQueue/$ns3::DropTailQueue<Packet>/MaxSize",    QueueSizeValue (QueueSize ("750p")));
  Config::Set ("NodeList/6/$ns3::Node/DeviceList/*/$ns3::PointToPointNetDevice/TxQueue/$ns3::DropTailQueue<Packet>/MaxSize",    QueueSizeValue (QueueSize ("750p")));
  Config::Set ("NodeList/7/$ns3::Node/DeviceList/*/$ns3::PointToPointNetDevice/TxQueue/$ns3::DropTailQueue<Packet>/MaxSize",    QueueSizeValue (QueueSize ("750p")));
  Config::Set ("NodeList/8/$ns3::Node/DeviceList/*/$ns3::PointToPointNetDevice/TxQueue/$ns3::DropTailQueue<Packet>/MaxSize",    QueueSizeValue (QueueSize ("750p")));
  Config::Set ("NodeList/9/$ns3::Node/DeviceList/*/$ns3::PointToPointNetDevice/TxQueue/$ns3::DropTailQueue<Packet>/MaxSize",    QueueSizeValue (QueueSize ("750p")));
  Config::Set ("NodeList/10/$ns3::Node/DeviceList/*/$ns3::PointToPointNetDevice/TxQueue/$ns3::DropTailQueue<Packet>/MaxSize",    QueueSizeValue (QueueSize ("750p")));
  Config::Set ("NodeList/11/$ns3::Node/DeviceList/*/$ns3::PointToPointNetDevice/TxQueue/$ns3::DropTailQueue<Packet>/MaxSize",    QueueSizeValue (QueueSize ("750p")));
  Config::Set ("NodeList/12/$ns3::Node/DeviceList/*/$ns3::PointToPointNetDevice/TxQueue/$ns3::DropTailQueue<Packet>/MaxSize",    QueueSizeValue (QueueSize ("750p")));

  std::cout<< "Total number of nodes: " << ns3::NodeList::GetNNodes() << std::endl;
  std::string dir = "/home/paddyxvy/garbage/" + simulation + "/" + tcp_algorithm + "/";
  std::string dirToSave = "mkdir -p " + dir;
  if( system (dirToSave.c_str ()) == 0)
  {
    std::cout << "Save directory created for " << tcp_algorithm << std::endl;
  }
  std::cout << "Run number: " << run << std::endl;
  std::cout << "Simulation: " << simulation << std::endl;
  std::cout << "Duration: " << duration << " seconds" << std::endl;



  Simulator::Schedule (Seconds (0.004), &TraceCwnd,  dir + std::to_string(run) + "_cwnd.csv");
  Simulator::Schedule (Seconds (0.004), &TraceGoodput, dir + std::to_string(run) + "_goodput.csv");
  Simulator::Schedule (Seconds (0.004), &TraceRtt, dir + std::to_string(run) + "_rtt.csv");
  Simulator::Schedule (Seconds (0.004), &TraceSsThresh);

  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();

  Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
  Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
  Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
  ConfigStore outputConfig2;
  outputConfig2.ConfigureDefaults ();
  outputConfig2.ConfigureAttributes ();

  //AnimationInterface anim("vehicularmobility.xml");
  Simulator::Stop (Seconds (duration));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
