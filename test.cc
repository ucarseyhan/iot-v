#include <ns3/netanim-module.h>
#include "ns3/mobility-module.h"
#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>
#include <ns3/node-container.h>
#include "ns3/network-module.h"
#include <vector>
#include <unordered_map>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "ns3/random-variable-stream.h"

using namespace ns3;
using namespace std;



/**
 * Constant Values
 */
int TOPOLOGYFILESEQUENCENUMBER = 1;


static const int TOPOLOGY_LEARN = 100;
static const int TOPOLOGY_RESPONSE = 101;
static const string MESSAGE_SEPERATOR = "~";

const string TOPOLOGYFILEHELPER = "/Users/seyhanucar/Desktop/ns3workspace/ns-3.26/files/topology%i.txt";
static const string TOPOLOGYFILENAME = "/Users/seyhanucar/Desktop/ns3workspace/ns-3.26/files/topology.txt";
//static const string MATLABSCHFILE	 = "/Users/seyhanucar/Desktop/ns3workspace/ns-3.26/scratch/matlab.sh";

const string NODEPACKETSTATSFILE = "/Users/seyhanucar/Desktop/ns3workspace/ns-3.26/files/wnode%i.txt";
const string CONTROLLERPACKETSTATSFILE = "/Users/seyhanucar/Desktop/ns3workspace/ns-3.26/files/controller%i.txt";




class TopologyInformation
{
public:
	int nodeId;
	int controllerId;
	double RSS;
	double SNR;
	/**
	 * Fill the TopologyInformation object
	 */
	void topologyInformationFromString(vector<string> parsedData)
	{
		//PacketFlag~NodeId~ControllerId~RSS~SNR
		if(parsedData.size() > 0)
		{
			this->nodeId = atoi(parsedData.at(1).c_str());
			this->controllerId = atoi(parsedData.at(2).c_str());
			this->RSS =  atof(parsedData.at(3).c_str());
			this->SNR =  atof(parsedData.at(4).c_str());
		}
	}
};



class WNode{
public:
	//Device related attributes
	Ptr<Node> node;
	Ptr<LrWpanNetDevice> device;
	Ptr<SingleModelSpectrumChannel> channel;


	//Helper attributes
	double signalToNoise;
	double RSS;
	string nodeId;
	int id;
};

class WController  {
public:
	//Device related attributes
	Ptr<Node> node;
	Ptr<LrWpanNetDevice> device;
	Ptr<SingleModelSpectrumChannel> channel;

	//Helper attributes
	string controllerId;
	int id;

};
///////////////////////////////////////////////////////////////////////////////////////////////
vector<string> split(string str,string sep)
{
	char* cstr=const_cast<char*>(str.c_str());
	char* current;
	vector<string> arr;
	current=strtok(cstr,sep.c_str());
	while(current!=NULL)
	{
		arr.push_back(current);
		current=strtok(NULL,sep.c_str());
	}
	return arr;
}




/////////////////////////////////////////////////////////////////////////////////////////////////
struct BasicPacketData 
{
public:
	BasicPacketData (Ptr<const Packet>& p) : p(p)
{

		//get packets 802.15.4-Header
		LrWpanMacHeader h;
		p->PeekHeader(h);

		size = p->GetSize();
		src = h.GetShortSrcAddr();
		dst = h.GetShortDstAddr();
		pan_id = h.GetSrcPanId();
		seq_nr = h.GetSeqNum();
}

	uint32_t size;
	Mac16Address src;
	Mac16Address dst;
	uint16_t pan_id;
	uint8_t seq_nr;


private:
	Ptr<const Packet> p;
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * PUblic variables
 */
map<int, TopologyInformation*> networkTopologyInformation;

////////////////////////////////////////////////////////////////////////////////////////////////////////////

static string generateTopologyLearnPacket(int controllerId)
{
	string topologyLearn = ("");
	char myData[200] = "";

	//PacketFlag flag
	sprintf(myData, "%d", TOPOLOGY_LEARN);
	topologyLearn += myData;
	topologyLearn += "~";

	//COntroller Id
	sprintf(myData, "%d", controllerId);
	topologyLearn += myData;
	topologyLearn += "~";


	return topologyLearn;
}

static string generateTopologyResponsePacket(int nodeId, int controllerId, double RSS, double snr)
{
	//PacketFlag~NodeId~ControllerId~RSS~SNR

	string topologyResponse = ("");
	char myData[200] = "";

	//PacketFlag flag
	sprintf(myData, "%d", TOPOLOGY_RESPONSE);
	topologyResponse += myData;
	topologyResponse += "~";

	//Node Id
	sprintf(myData, "%d", nodeId);
	topologyResponse += myData;
	topologyResponse += "~";

	//Controller Id
	sprintf(myData, "%d", controllerId);
	topologyResponse += myData;
	topologyResponse += "~";

	//RSS
	sprintf(myData, "%f", RSS);
	topologyResponse += myData;
	topologyResponse += "~";

	//SNR
	sprintf(myData, "%f", snr);
	topologyResponse += myData;
	topologyResponse += "~";


	return topologyResponse;
}

static void writeNodePacketStatsToFile(int wnodeId, int packetType, int controllerId,  double time)
{

	string fileData = ("");
	char myData[200];

	char filename[100];
	sprintf(filename,NODEPACKETSTATSFILE.c_str(),wnodeId);
	ofstream out(filename, ios::app);
	if (!out) cout << "Cannot open file.\n";

	//Node unique id
	sprintf(myData, "%d", wnodeId);
	fileData += myData;
	fileData += "~";
	//Node packetType
	sprintf(myData, "%d", packetType);
	fileData += myData;
	fileData += "~";
	//Controller ID
	sprintf(myData, "%d", controllerId);
	fileData += myData;
	fileData += "~";


	//Simulation time
	sprintf(myData, "%f", time);
	fileData += myData;
	fileData += "~";

	//Write into file
	out <<fileData<< "\n";
	//Clear file data
	out.close();

}

static void writeControllerPacketStatsToFile(int contollerId, int packetType, int wnodeId,  double time)
{

	string fileData = ("");
	char myData[200];

	char filename[100];
	sprintf(filename,CONTROLLERPACKETSTATSFILE.c_str(),contollerId);
	ofstream out(filename, ios::app);
	if (!out) cout << "Cannot open file.\n";

	//Controller unique id
	sprintf(myData, "%d", contollerId);
	fileData += myData;
	fileData += "~";
	//Node packetType
	sprintf(myData, "%d", packetType);
	fileData += myData;
	fileData += "~";
	//Node ID
	sprintf(myData, "%d", wnodeId);
	fileData += myData;
	fileData += "~";

	//Simulation time
	sprintf(myData, "%f", time);
	fileData += myData;
	fileData += "~";

	//Write into file
	out <<fileData<< "\n";
	//Clear file data
	out.close();

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



static void PacketEnqued (Ptr<LrWpanNetDevice> dev, Ptr<const Packet> p)
{
	BasicPacketData bpd (p);

	NS_LOG_UNCOND ("----------MSDU enqued----------\n"
			<< "Packet of size " << bpd.size << " bytes enqued\n"
			<< "on device with address "<<dev->GetMac()->GetShortAddress()<<"\n"
			<< "to be sent to address " << bpd.dst << "\n"
			<< "sequence number is " << int (bpd.seq_nr) << "\n"
			<< "Source PAN ID is " << bpd.pan_id << "\n"
			<< "----------MSDU enqued------------\n\n"
	);
}

static void wNodePacketReceived (WNode* wnode, Ptr<const Packet> p)
{
	LrWpanMacHeader h;
	p->PeekHeader(h);
	BasicPacketData bpd (p);

	Ptr<Packet> packet = p->Copy();
	packet->RemoveHeader(h);



	if(!h.IsAcknowledgment())
	{
		NS_LOG_UNCOND ("----------Packet received----------\n"
				<< "Packet of size " << bpd.size << " bytes received\n"
				<< "on device with address "<< wnode->device->GetMac()->GetShortAddress()<<"\n"

				<< "was sent from address " << bpd.src << "\n"
				<< "sequence number is " << (int) bpd.seq_nr << "\n"
				<< "PAN ID is " << bpd.pan_id << "\n"
				<< "----------Packet received------------\n\n"
		);

		uint8_t *buffer = new uint8_t[packet->GetSize()];
		packet->CopyData (buffer, packet->GetSize());
		string receivedData(buffer, buffer+packet->GetSize ());
		/**
		 * Parse the received data
		 */
		vector<string> parsedData = split(receivedData, MESSAGE_SEPERATOR);
		if(parsedData.size() > 0)
		{
			int packetType = atoi(parsedData.at(0).c_str());

			if(packetType == TOPOLOGY_LEARN)
			{

				int controllerId =  atoi(parsedData.at(1).c_str());

				//Write packet stats
				writeNodePacketStatsToFile(wnode->id, TOPOLOGY_LEARN, controllerId, Simulator::Now().GetSeconds());

				//PacketType~ControllerId
				Ptr<UniformRandomVariable> v = CreateObject<UniformRandomVariable> ();
				v->SetAttribute ("Min", DoubleValue (0));
				v->SetAttribute ("Max", DoubleValue (1));


				//Create the Topology Response Packet
				//Create the response
				McpsDataRequestParams params;
				params.m_srcAddrMode = SHORT_ADDR;
				params.m_dstAddrMode = SHORT_ADDR;
				params.m_dstPanId = 0;
				//!!Node unicasts the TopologyResponse
				params.m_dstAddr = bpd.src;
				params.m_msduHandle = 0;
				params.m_txOptions = TX_OPTION_ACK;

				string packetInformation = ("");
				packetInformation = generateTopologyResponsePacket(wnode->id, controllerId, wnode->RSS, wnode->signalToNoise);
				stringstream msgx;
				msgx << packetInformation;
				uint16_t packetSize = msgx.str().length()+1;
				Ptr<Packet> packet = Create<Packet>((uint8_t*) msgx.str().c_str(), packetSize);

				//Schedule CSMA send
				Simulator::ScheduleWithContext (1, Seconds (v->GetValue()), &LrWpanMac::McpsDataRequest, wnode->device->GetMac (), params, packet);
			}
		}
	}
	else
	{
		NS_LOG_UNCOND ("----------ACK Frame received---------\n"
				<< "ACK frame of size " << bpd.size << " bytes \n"
				<<  "received on device with address " << wnode->device->GetMac()->GetShortAddress() << "\n"
				<< "for the frame with sequence number " << (int) bpd.seq_nr << "\n"
				<< "----------ACK Frame received---------\n\n"
		);

	}

}

static void wControllerPacketReceived (WController* wcontroller, Ptr<const Packet> p)
{

	LrWpanMacHeader h;
	p->PeekHeader(h);
	BasicPacketData bpd (p);

	Ptr<Packet> packet = p->Copy();
	packet->RemoveHeader(h);



	if(!h.IsAcknowledgment())
	{
		uint8_t *buffer = new uint8_t[packet->GetSize()];
		packet->CopyData (buffer, packet->GetSize());
		string receivedData(buffer, buffer+packet->GetSize ());

		/**
		 * Parse the received data
		 */
		vector<string> parsedData = split(receivedData, MESSAGE_SEPERATOR);
		if(parsedData.size() > 0)
		{
			int packetType = atoi(parsedData.at(0).c_str());

			if(packetType == TOPOLOGY_RESPONSE)
			{
				//PacketFlag~NodeId~ControllerId~RSS~SNR
				int nodeId = atoi(parsedData.at(1).c_str());
				//Write packet stats
				writeControllerPacketStatsToFile(wcontroller->id,TOPOLOGY_RESPONSE,nodeId,Simulator::Now().GetSeconds());

				TopologyInformation* nodeTopologyInformation = new TopologyInformation();
				nodeTopologyInformation->topologyInformationFromString(parsedData);


				/**
				 * Get the topologyInformation hash table
				 * If this hash table contains the node id then
				 * 	 Get the RSS value from TopologyInformation
				 * 	 If RSS value is larger than the value in hash table
				 * 	 Update it
				 */
				TopologyInformation* existingTopologyInformation = networkTopologyInformation[nodeId];
				if(existingTopologyInformation)
				{
					//If larger RSS then update it
					if(nodeTopologyInformation->RSS > existingTopologyInformation->RSS)
					{
						networkTopologyInformation[nodeId] = nodeTopologyInformation;
					}
				}
				else
				{
					networkTopologyInformation[nodeId] = nodeTopologyInformation;
					cout<<"Node:"<<nodeId<<" RSS:"<<nodeTopologyInformation->RSS<<endl;
				}
			}
		}



	}
	else
	{
		NS_LOG_UNCOND ("----------ACK Frame received---------\n"
				<< "ACK frame of size " << bpd.size << " bytes \n"
				<<  "received on device with address " << wcontroller->device->GetMac()->GetShortAddress() << "\n"
				<< "for the frame with sequence number " << (int) bpd.seq_nr << "\n"
				<< "----------ACK Frame received---------\n\n"
		);

	}
}

static void PacketSendInfo (Ptr<LrWpanNetDevice> dev, Ptr<const Packet> p, uint8_t retries, uint8_t csmaca_backoffs)
{
	BasicPacketData bpd (p);
	NS_LOG_UNCOND ("----------MSDU SEND INFO----------\n"
			<< "Packet with sequence number "<< int(bpd.seq_nr)<<"\n"
			<< "was sent or given up on device with address " << dev->GetMac()->GetShortAddress() << "\n"
			<< "Number of sending retries " << int(retries)<<"\n"
			<< "Number of CSMA/CA backoffs " << int(csmaca_backoffs) << "\n"
			<< "----------MSDU SEND INFO------------\n"
			<< "---------------------------------------------------\n"
			<< "---------------------------------------------------\n\n\n\n");
}

static void McpsDataConfirm(Ptr<LrWpanNetDevice> dev, McpsDataConfirmParams params)
{
	std::string status;
	switch(params.m_status)
	{
	case   IEEE_802_15_4_SUCCESS : status = "IEEE_802_15_4_SUCCESS"; break;
	case   IEEE_802_15_4_TRANSACTION_OVERFLOW : status =  "IEEE_802_15_4_TRANSACTION_OVERFLOW"; break;
	case   IEEE_802_15_4_TRANSACTION_EXPIRED  : status = "IEEE_802_15_4_TRANSACTION_EXPIRED"; break;
	case   IEEE_802_15_4_CHANNEL_ACCESS_FAILURE  : status = "IEEE_802_15_4_CHANNEL_ACCESS_FAILURE"; break;
	case   IEEE_802_15_4_INVALID_ADDRESS  : status = "IEEE_802_15_4_INVALID_ADDRESS"; break;
	case   IEEE_802_15_4_INVALID_GTS   : status = "IEEE_802_15_4_INVALID_GTS"; break;
	case   IEEE_802_15_4_NO_ACK        : status = "IEEE_802_15_4_NO_ACK"; break;
	case   IEEE_802_15_4_COUNTER_ERROR    : status = "IEEE_802_15_4_COUNTER_ERROR"; break;
	case   IEEE_802_15_4_FRAME_TOO_LONG   : status = "IEEE_802_15_4_FRAME_TOO_LONG"; break;
	case   IEEE_802_15_4_UNAVAILABLE_KEY    : status = "IEEE_802_15_4_UNAVAILABLE_KEY"; break;
	case   IEEE_802_15_4_UNSUPPORTED_SECURITY   : status = "IEEE_802_15_4_UNSUPPORTED_SECURITY"; break;
	case   IEEE_802_15_4_INVALID_PARAMETER   : status = "IEEE_802_15_4_INVALID_PARAMETER"; break;
	default : status = "UNKNOWN VALUE";
	}

	NS_LOG_UNCOND ("--------McpsDataConfirmStatus--------\n"
			<< "On device with address "<< dev->GetMac()->GetShortAddress()<<"\n"
			<< status << "\n"
			<<"--------McpsDataConfirmStatus-------- " << "\n\n"
	);
}


static void PacketSentSuccessfully (Ptr<const Packet> p)
{

	BasicPacketData bpd (p);
	NS_LOG_UNCOND ("----------MSDU sent successfully----------\n"
			<< "Packet of size " << bpd.size << " bytes sent\n"
			<< "was successfully sent from address " << bpd.src << " to address " << bpd.dst << "\n"
			<< "sequence number is " << (int) bpd.seq_nr << "\n"
			<< "PAN ID is " << bpd.pan_id << "\n"
			<< "----------MSDU sent successfully------------\n\n"
	);
}



//static void PacketDropped (Ptr<LrWpanNetDevice> dev, Ptr<const Packet> p)
//{
//
//	BasicPacketData bpd (p);
//	NS_LOG_UNCOND ("----------Packet dropped----------\n"
//			<< "Packet of size " << bpd.size << " was dropped during MAC filtering\n"
//			<< "on device with address "<<dev->GetMac()->GetShortAddress()<<"\n"
//			<< "coming from address " << bpd.src << " to address " << bpd.dst << "\n"
//			<< "sequence number is " << (int) bpd.seq_nr << "\n"
//			<< "PAN ID is " << bpd.pan_id << "\n"
//			<< "----------Packet dropped------------\n\n"
//	);
//}

static void phyTrace (WNode* wnode, Ptr< const Packet > p, double sinr)
{

	LrWpanMacHeader h;
	p->PeekHeader(h);
	BasicPacketData bpd (p);

	//If packet is not acknowledgment
	if(!h.IsAcknowledgment())
	{
		double signalToNoise = 10 * log10 (sinr);
		wnode->RSS = wnode->device->GetPhy()->RSS;
		wnode->signalToNoise = signalToNoise;
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void MakeCallbacks(vector<WNode*> wnodeList)
{
	for (std::vector<WNode* >::const_iterator i = wnodeList.begin(); i!=wnodeList.end(); ++i)
	{
		WNode* wnode = *i;
		wnode->device->GetMac()->TraceConnectWithoutContext ("MacTxEnqueue", MakeBoundCallback(&PacketEnqued,wnode->device));
		wnode->device->GetMac()->TraceConnectWithoutContext ("MacTxOk", MakeCallback(&PacketSentSuccessfully));
		wnode->device->GetMac()->TraceConnectWithoutContext ("MacRx", MakeBoundCallback(&wNodePacketReceived, wnode));
		//wnode->device->GetMac()->TraceConnectWithoutContext ("MacRxDrop", MakeBoundCallback(&PacketDropped,wnode->device));
		wnode->device->GetMac()->TraceConnectWithoutContext ("MacSentPkt", MakeBoundCallback(&PacketSendInfo,wnode->device));

		wnode->device->GetPhy()->TraceConnectWithoutContext ("PhyRxEnd", MakeBoundCallback(&phyTrace,wnode));


		McpsDataConfirmCallback cb0 = MakeBoundCallback (&McpsDataConfirm,wnode->device);
		wnode->device->GetMac ()->SetMcpsDataConfirmCallback (cb0);
	}
}

static void MakeCallbacks(vector<WController*> wcontrollerList)
{


	for (vector<WController*>::const_iterator i = wcontrollerList.begin(); i != wcontrollerList.end(); ++i)
	{
		WController* wcontroller = *i;
		wcontroller->device->GetMac()->TraceConnectWithoutContext ("MacTxEnqueue", MakeBoundCallback(&PacketEnqued,wcontroller->device));
		wcontroller->device->GetMac()->TraceConnectWithoutContext ("MacTxOk", MakeCallback(&PacketSentSuccessfully));
		wcontroller->device->GetMac()->TraceConnectWithoutContext ("MacRx", MakeBoundCallback(&wControllerPacketReceived, wcontroller));
		//wcontroller->device->GetMac()->TraceConnectWithoutContext ("MacRxDrop", MakeBoundCallback(&PacketDropped,wcontroller->device));
		wcontroller->device->GetMac()->TraceConnectWithoutContext ("MacSentPkt", MakeBoundCallback(&PacketSendInfo,wcontroller->device));

		//wcontroller->device->GetPhy()->TraceConnectWithoutContext ("PhyRxEnd", MakeCallback(&phyTrace));

		McpsDataConfirmCallback cb0 = MakeBoundCallback (&McpsDataConfirm,wcontroller->device);
		wcontroller->device->GetMac ()->SetMcpsDataConfirmCallback (cb0);
	}

}

double  learnTopology(vector<WController*> wcontrollerList)
{
	//Sending Packets
	McpsDataRequestParams params;
	params.m_srcAddrMode = SHORT_ADDR;
	params.m_dstAddrMode = SHORT_ADDR;
	params.m_dstPanId = 0;
	params.m_dstAddr = Mac16Address ("FF:FF");
	params.m_msduHandle = 0;
	params.m_txOptions = TX_OPTION_ACK;
	/**
	 * Topology Learn
	 */

	double second = 0.0;

	for (vector<WController*>::const_iterator i = wcontrollerList.begin(); i != wcontrollerList.end(); ++i)
	{
		WController* controller = *i;
		string packetInformation = ("");
		packetInformation = generateTopologyLearnPacket(controller->id);
		stringstream msgx;
		msgx << packetInformation;
		uint16_t packetSize = msgx.str().length()+1;
		Ptr<Packet> packet = Create<Packet>((uint8_t*) msgx.str().c_str(), packetSize);

		Simulator::ScheduleWithContext (1, Seconds (second),
				&LrWpanMac::McpsDataRequest,
				controller->device->GetMac (), params, packet);

		second += 1.0;
	}

	return second;

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void writeTopologyInformationToFile()
{
	string fileData = ("");
	char myData[200];
	//If file exist, delete it
	ifstream fin(TOPOLOGYFILENAME);
	if (fin)
	{
		/**
		 * Copy the topology learning file
		 */
		char filename[100];
		sprintf(filename,TOPOLOGYFILEHELPER.c_str(),TOPOLOGYFILESEQUENCENUMBER);

		ifstream infile(TOPOLOGYFILENAME.c_str());
		ofstream outfile(filename);

		string content = "";
		int i;
		for(i=0 ; infile.eof()!=true ; i++)
		{
			// get content of infile
			content += infile.get();
		}
		// erase last character
		i--;
		content.erase(content.end()-1);

		cout << i << " characters read...\n";
		infile.close();

		// output
		outfile << content;
		outfile.close();
		//increase the
		TOPOLOGYFILESEQUENCENUMBER += 1;
		/////////////////////////////////////////////////////////////////////////////


		fin.close();
		remove(TOPOLOGYFILENAME.c_str());
	}

	ofstream out(TOPOLOGYFILENAME, ios::app);
	if (!out) cout << "Cannot open file.\n";

	cout<<"Size:"<<networkTopologyInformation.size()<<endl;
	for ( const auto &myPair : networkTopologyInformation )
	{
		int key = myPair.first;
		TopologyInformation* info = networkTopologyInformation[key];
		cout<<"Key:"<< myPair.first<<" RSS:"<<info->RSS<<" Controller:"<<info->controllerId<<endl;

		//Node unique id
		sprintf(myData, "%d", myPair.first);
		fileData += myData;
		fileData += "~";
		//Node sensed RSS
		sprintf(myData, "%f", info->RSS);
		fileData += myData;
		fileData += "~";
		//Node assigned Controller
		sprintf(myData, "%d", info->controllerId);
		fileData += myData;
		fileData += "~";

		//Write into file
		out <<fileData<< "\n";
		//Clear file data
		fileData = ("");
	}

	out.close();
}


/**
static void runScheduling()
{

	system(MATLABSCHFILE.c_str());

}
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




/**
 * Nodes related list
 */
vector<WNode*> wnodeList;
/**
 * Controller Related list
 */
vector<WController*> wcontorllerNodesList;



int main (int argc, char *argv[])
{
	//LogComponentEnable ("LrWpanPhy", LOG_LEVEL_ALL);

	srand(time(NULL));
	map<string, Vector> nodePositionHashTable;


	bool verbose = false;

	CommandLine cmd;
	cmd.AddValue ("verbose", "turn on all log components", verbose);
	cmd.Parse (argc, argv);

	LrWpanHelper lrWpanHelper;
	if (verbose)
	{
		lrWpanHelper.EnableLogComponents ();
	}
	ns3::PacketMetadata::Enable ();
	GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));




	int numberOfNodes = 100;
	int numberOfController = 5;
	//Create the node list
	for(int i = 0; i < numberOfNodes; i++)
	{
		WNode* wnode = new WNode();
		wnode->node = CreateObject <Node> ();
		wnodeList.push_back(wnode);
	}
	//Create the controller
	for(int i = 0; i < numberOfController; i++)
	{
		WController* wcontroller = new WController();
		wcontroller->node = CreateObject <Node> ();
		wcontorllerNodesList.push_back(wcontroller);
	}

	/**
	 * To implement the Rayleigh, Nakagami (m values set to 1) and LogDistance
	 * propagation channel models are combined.
	 */
	// Each device must be attached to the same channel
	Ptr<SingleModelSpectrumChannel> channel
	= CreateObject<SingleModelSpectrumChannel> ();

	Ptr<NakagamiPropagationLossModel> propModel
	= CreateObject<NakagamiPropagationLossModel> ();
	propModel->SetAttribute("m0",DoubleValue(1.0));
	propModel->SetAttribute("m1",DoubleValue(1.0));
	propModel->SetAttribute("m2",DoubleValue(1.0));
	//Add Channel Properties
	channel->AddPropagationLossModel(propModel);

	Ptr<LogDistancePropagationLossModel> lognormalPropagationModel
	= CreateObject<LogDistancePropagationLossModel> ();
	//Add Channel Properties
	channel->AddPropagationLossModel(lognormalPropagationModel);

	Ptr<ConstantSpeedPropagationDelayModel> delayModel
	= CreateObject<ConstantSpeedPropagationDelayModel> ();
	//Add Channel Properties
	channel->SetPropagationDelayModel (delayModel);


	MobilityHelper mobility;
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

	int start = 0;
	int end = 20;
	/**
	 * Nodes position adjustment
	 */
	for(uint i = 0; i < wnodeList.size();i++)
	{
		WNode* wnode = wnodeList[i];
		wnode->device = CreateObject<LrWpanNetDevice> ();

		stringstream sstm;
		if((i+1) < 10)	sstm << "00:0" << (i+1);
		else sstm << "00:" << (i+1);
		wnode->device->SetAddress (Mac16Address (sstm.str().c_str()));
		wnode->device->SetChannel(channel);
		string nodeId = sstm.str().c_str();
		wnode->nodeId = nodeId;
		wnode->id = (i+1);
		wnode->node->AddDevice(wnode->device);

		int locationY = rand()%((end - 0) + 1) + 0;
		int locationX = rand()%((end - start) + 1) + start;

		Ptr<ConstantPositionMobilityModel>  position = CreateObject<ConstantPositionMobilityModel> ();
		position->SetPosition (Vector (locationX,locationY,0));
		wnode->device->GetPhy ()->SetMobility (position);
		nodePositionHashTable [sstm.str().c_str()] =  Vector (locationX,locationY,0);
		mobility.Install(wnode->node);
	}

	/**
	 * Controller position adjustment
	 */
	for(uint i = 0; i < wcontorllerNodesList.size(); i++)
	{
		WController* wcontroller = wcontorllerNodesList[i];
		wcontroller->device = CreateObject<LrWpanNetDevice> ();
		stringstream sstm;
		sstm << "00:0" << (numberOfNodes+i+1);
		wcontroller->device->SetAddress (Mac16Address (sstm.str().c_str()));
		wcontroller->device->SetChannel(channel);
		string controllerId = sstm.str().c_str();
		wcontroller->controllerId = controllerId;
		wcontroller->id =  (numberOfNodes+i+1);

		wcontroller->node->AddDevice(wcontroller->device);
		int controllerX = rand()%((end - 0) + 1) + 0;
		int controllerY = rand()%((end - start) + 1) + start;

		Ptr<ConstantPositionMobilityModel>  position = CreateObject<ConstantPositionMobilityModel> ();
		position->SetPosition (Vector (controllerX,controllerY,0));
		wcontroller->device->GetPhy ()->SetMobility (position);

		nodePositionHashTable [sstm.str().c_str()] =  Vector (controllerX,controllerY,0);
		mobility.Install(wcontroller->node);
	}

	MakeCallbacks(wnodeList);
	MakeCallbacks(wcontorllerNodesList);

	//Learn the topology
	double second = learnTopology(wcontorllerNodesList);
	second += 1.0;
	//Get the topology write into file
	Simulator::Schedule(Seconds(second), writeTopologyInformationToFile);
	second += 1.0;
	//Run the scheduling
	//Simulator::Schedule(Seconds(second), runScheduling);


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 * Animator Configuration
	 */

	AnimationInterface anim ("wncs.xml");
	anim.EnablePacketMetadata (); // Optional
	anim.EnableWifiMacCounters (Seconds (0), Seconds (100)); //Optional
	anim.EnableWifiPhyCounters (Seconds (0), Seconds (100)); //Optional
	/**
	 * Nodes placement
	 */
	for(uint i = 0; i < wnodeList.size();i++)
	{
		WNode* currentNode = wnodeList[i];
		stringstream sstm;
		if((i+1) < 10)	sstm << "00:0" << (i+1);
		else sstm << "00:" << (i+1);
		string address = sstm.str().c_str();
		Vector location = nodePositionHashTable[address];
		anim.SetConstantPosition(currentNode->node, location.x, location.y, 0);
	}
	/**
	 * Controller placement
	 */
	for(uint i = 0; i < wcontorllerNodesList.size();i++)
	{
		WController* controller = wcontorllerNodesList[i];
		stringstream sstm;
		sstm << "00:0" << (numberOfNodes+i+1);
		string address = sstm.str().c_str();

		Vector location = nodePositionHashTable[address];
		anim.SetConstantPosition(controller->node, location.x, location.y, 0);
		anim.UpdateNodeColor (controller->node, 0, 0, 255); // Optional
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	Simulator::Stop (Seconds (100.0));
	Simulator::Run ();
	Simulator::Destroy ();




	return 0;
}
