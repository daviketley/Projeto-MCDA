#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;

int main(int argc, char* argv[]) {
  // Configuração dos parâmetros da simulação
  int numUAVs = 15;
  int numUsers = 4000;
  int simTime = 10; // Tempo de simulação em segundos

  // Criação dos nós da rede
  NodeContainer uavNodes;
  uavNodes.Create(numUAVs);

  NodeContainer userNodes;
  userNodes.Create(numUsers);

  // Configuração da mobilidade dos UAVs
  MobilityHelper mobilityUAVs;
  mobilityUAVs.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                    "X", StringValue("500.0"),
                                    "Y", StringValue("500.0"),
                                    "RhoMin", StringValue("0.0"),
                                    "RhoMax", StringValue("300.0"));
  mobilityUAVs.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityUAVs.Install(uavNodes);

  // Configuração da mobilidade dos usuários
  MobilityHelper mobilityUsers;
  mobilityUsers.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                     "X", StringValue("500.0"),
                                     "Y", StringValue("500.0"),
                                     "RhoMin", StringValue("0.0"),
                                     "RhoMax", StringValue("400.0"));
  mobilityUsers.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityUsers.Install(userNodes);

  // Configuração da pilha de protocolos da internet
  InternetStackHelper internet;
  internet.Install(uavNodes);
  internet.Install(userNodes);

  // Criação do canal de comunicação
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
  p2p.SetChannelAttribute("Delay", StringValue("2ms"));

  NetDeviceContainer devices;
  devices = p2p.Install(uavNodes);

  // Atribuição de endereços IP
  Ipv4AddressHelper address;
  address.SetBase("10.1.1.0", "255.255.255.0");

  Ipv4InterfaceContainer interfaces;
  interfaces = address.Assign(devices);

  // Criação dos aplicativos de tráfego
  UdpEchoServerHelper echoServer(9);

  ApplicationContainer serverApps = echoServer.Install(uavNodes.Get(numUAVs - 1));
  serverApps.Start(Seconds(1.0));
  serverApps.Stop(Seconds(simTime));

  UdpEchoClientHelper echoClient(interfaces.GetAddress(numUAVs - 1), 9);
  echoClient.SetAttribute("MaxPackets", UintegerValue(100));
  echoClient.SetAttribute("Interval", TimeValue(Seconds(0.1)));
  echoClient.SetAttribute("PacketSize", UintegerValue(1024));

ApplicationContainer clientApps = echoClient.Install(userNodes.Get(numUsers - 1));
clientApps.Start(Seconds(2.0));
clientApps.Stop(Seconds(simTime));

// Configuração do rastreamento de pacotes
AsciiTraceHelper ascii;
p2p.EnableAsciiAll(ascii.CreateFileStream("trace.txt"));

// Configuração da animação da simulação
AnimationInterface anim("animation.xml");
anim.SetConstantPosition(uavNodes.Get(numUAVs - 1), 500.0, 500.0);
anim.EnablePacketMetadata(); // Exibe informações dos pacotes na animação

// Configuração do monitor de fluxo
FlowMonitorHelper flowmon;
Ptr<FlowMonitor> monitor = flowmon.InstallAll();

// Execução da simulação
Simulator::Stop(Seconds(simTime));
Simulator::Run();
Simulator::Destroy();

// Cálculo da vazão dos nós UAVs
for (int i = 0; i < numUAVs; i++) {
Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
double totalThroughput = 0.0;
for (auto it = stats.begin(); it != stats.end(); ++it) {
  Ipv4FlowClassifier::FiveTuple tuple = classifier->FindFlow(it->first);
  if (tuple.sourceAddress == uavNodes.Get(i)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal()) {
    totalThroughput += it->second.rxBytes * 8.0 / (simTime * 1000000.0); // Mbps
  }
}

std::cout << "Vazão do UAV " << i << ": " << totalThroughput << " Mbps" << std::endl;

}

return 0;
}