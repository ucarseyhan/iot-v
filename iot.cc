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
//#include <math.h> //Ubuntu
#include <cmath> //Mac
#include "ns3/random-variable-stream.h"

using namespace ns3;
using namespace std;

//double calculatedPathloss = 0.0;


/**
 * Constant Values
 */
int TOPOLOGYFILESEQUENCENUMBER = 1;
static const int PERIOD[4] = { 1, 2, 4, 8};

static const int TOPOLOGY_LEARN = 100;
static const int TOPOLOGY_RESPONSE = 101;
//static const int TOPOLOGY_SCHEDULE = 102;




static const string MESSAGE_SEPERATOR = "~";

const string TOPOLOGYFILEHELPER = "/Users/seyhanucar/Desktop/ns3workspace/ns-3.26/files/topology%i.txt";
static const string TOPOLOGYFILENAME = "/Users/seyhanucar/Desktop/ns3workspace/ns-3.26/files/topology.txt";
//static const string MATLABSCHFILE	 = "/Users/seyhanucar/Desktop/ns3workspace/ns-3.26/scratch/matlab.sh";

const string NODEPACKETSTATSFILE = "/Users/seyhanucar/Desktop/ns3workspace/ns-3.26/files/wnode%i.txt";
const string CONTROLLERPACKETSTATSFILE = "/Users/seyhanucar/Desktop/ns3workspace/ns-3.26/files/controller%i.txt";
const string SCHEDULEINFORMATION = "/Users/seyhanucar/Desktop/ns3workspace/ns-3.26/files/output.txt";

class TopologyInformation
{
public:
	int nodeId;
	int controllerId;
	double RSS;
	double SNR;
	int period;
	/**
	 * Fill the TopologyInformation object
	 */
	void topologyInformationFromString(vector<string> parsedData)
	{
		//PacketFlag~NodeId~ControllerId~RSS~SNR~period
		if(parsedData.size() > 0)
		{
			this->nodeId = atoi(parsedData.at(1).c_str());
			this->controllerId = atoi(parsedData.at(2).c_str());
			this->RSS =  atof(parsedData.at(3).c_str());
			this->SNR =  atof(parsedData.at(4).c_str());
			this->period =  atoi(parsedData.at(5).c_str());
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
	double pathLoss;
	string nodeId;
	int id;
	int period;
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
map<int, vector<TopologyInformation*>> networkTopologyInformation;

////////////////////////////////////////////////////////////////////////////////////////////////////////////

static string generateTopologyLearnPacket(int controllerId)
{
	string topologyLearn = ("");
	char myData[200] = "";

	//PacketFlag flag
	sprintf(myData, "%d", TOPOLOGY_LEARN);
	topologyLearn += myData;
	topologyLearn += "~";

	//C0ntroller Id
	sprintf(myData, "%d", controllerId);
	topologyLearn += myData;
	topologyLearn += "~";


	return topologyLearn;
}

static string generateTopologyResponsePacket(int nodeId,
		int controllerId, double RSS, double snr, int period)
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

	//Period
	sprintf(myData, "%d", period);
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
				<< "on device with address "<<
				wnode->device->GetMac()->GetShortAddress()<<"\n"

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
				writeNodePacketStatsToFile(wnode->id, TOPOLOGY_LEARN,
						controllerId, Simulator::Now().GetSeconds());

				//PacketType~ControllerId

				Ptr<UniformRandomVariable> v
				= CreateObject<UniformRandomVariable> ();
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
				packetInformation = generateTopologyResponsePacket(wnode->id,
						controllerId, wnode->RSS, wnode->signalToNoise,wnode->period);
				stringstream msgx;
				msgx << packetInformation;
				uint16_t packetSize = msgx.str().length()+1;
				Ptr<Packet> packet = Create<Packet>((uint8_t*) msgx.str().c_str(), packetSize);

				//Schedule CSMA send
				Simulator::ScheduleWithContext (1,
						Seconds (v->GetValue()),
						&LrWpanMac::McpsDataRequest,
						wnode->device->GetMac (),
						params, packet);

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
static bool containTopologyInformation(vector<TopologyInformation*> existingTopologyInformation, TopologyInformation* target)
{
	for(vector<TopologyInformation*>::iterator it = existingTopologyInformation.begin(); it != existingTopologyInformation.end(); ++it)
	{
		TopologyInformation* current = *it;
		if(current->controllerId == target->controllerId) return true;
	}
	return false;
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
				//PacketFlag~NodeId~ControllerId~RSS~SNR~Node Period
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
				vector<TopologyInformation*> existingTopologyInformation = networkTopologyInformation[nodeId];
				//TopologyInformation* existingTopologyInformation = networkTopologyInformation[nodeId];
				//				if(existingTopologyInformation)
				//				{
				//					//If larger RSS then update it
				//					if(nodeTopologyInformation->RSS > existingTopologyInformation->RSS)
				//					{
				//						networkTopologyInformation[nodeId] = nodeTopologyInformation;
				//					}
				//				}
				//				else
				//				{
				//					networkTopologyInformation[nodeId] = nodeTopologyInformation;
				//					cout<<"Node:"<<nodeId<<" RSS:"<<nodeTopologyInformation->RSS<<endl;
				//				}

				if(!containTopologyInformation(existingTopologyInformation, nodeTopologyInformation))
				{
					existingTopologyInformation.push_back(nodeTopologyInformation);
				}
				networkTopologyInformation[nodeId] = existingTopologyInformation;



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

//static void pathLoss (Ptr< SpectrumPhy > txPhy, Ptr< SpectrumPhy > rxPhy, double lossDb)
//{
//	calculatedPathloss = lossDb;
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void MakeCallbacks(vector<WNode*> wnodeList)
{
	for (vector<WNode* >::const_iterator i = wnodeList.begin(); i!=wnodeList.end(); ++i)
	{
		WNode* wnode = *i;
		wnode->device->GetMac()->
				TraceConnectWithoutContext ("MacTxEnqueue", MakeBoundCallback(&PacketEnqued,wnode->device));
		wnode->device->GetMac()->
				TraceConnectWithoutContext ("MacTxOk", MakeCallback(&PacketSentSuccessfully));

		wnode->device->GetMac()->
				TraceConnectWithoutContext ("MacRx", MakeBoundCallback(&wNodePacketReceived, wnode));

		/*wnode->device->GetMac()->
		 * TraceConnectWithoutContext ("MacRxDrop", MakeBoundCallback(&PacketDropped,wnode->device));*/
		wnode->device->GetMac()->
				TraceConnectWithoutContext ("MacSentPkt", MakeBoundCallback(&PacketSendInfo,wnode->device));
		wnode->device->GetPhy()->
				TraceConnectWithoutContext ("PhyRxEnd", MakeBoundCallback(&phyTrace,wnode));
		/*wnode->device->GetPhy()->GetChannel()->
		 * TraceConnectWithoutContext("PathLoss", MakeBoundCallback(&pathLoss,wnode));*/

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

void  learnTopology(vector<WController*> wcontrollerList)
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

	//double second = 0.0;

	Ptr<UniformRandomVariable> v = CreateObject<UniformRandomVariable> ();
	v->SetAttribute ("Min", DoubleValue (0));
	v->SetAttribute ("Max", DoubleValue (1));

	for (vector<WController*>::const_iterator i = wcontrollerList.begin(); i != wcontrollerList.end(); ++i)
	{
		WController* controller = *i;
		string packetInformation = ("");
		packetInformation = generateTopologyLearnPacket(controller->id);
		stringstream msgx;
		msgx << packetInformation;
		uint16_t packetSize = msgx.str().length()+1;
		Ptr<Packet> packet = Create<Packet>((uint8_t*) msgx.str().c_str(), packetSize);

		Simulator::ScheduleWithContext (1, Seconds (v->GetValue()),
				&LrWpanMac::McpsDataRequest,
				controller->device->GetMac (), params, packet);
		//second += 1.0;
	}

	//return second;

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
		vector<TopologyInformation*> info = networkTopologyInformation[key];
		int size = info.size();
		/**
		 * Print the RSS to user
		 */
		string rssData = ("");
		char data[200];
		cout<<"Key:"<< myPair.first;

		double MIN_RSS = INT_MAX;
		int MIN_CONTROLLER = -1;
		int period = -1;
		for(int i= 0; i < size; i++)
		{
			//cout<<" C:"<<info[i]->controllerId<<" "<<abs(info[i]->RSS);
			if(abs(info[i]->RSS) < MIN_RSS)
			{
				MIN_RSS = abs(info[i]->RSS);
				MIN_CONTROLLER = info[i]->controllerId;
				period = info[i]->period;
			}
//			sprintf(data, "%d", info[i]->controllerId);
//			rssData += data;
//			rssData += ":";
//			//Node sensed RSS
//			sprintf(data, "%f", abs(info[i]->RSS));
//			rssData += data;
//			rssData += "~";
		}

		sprintf(data, "%d",MIN_CONTROLLER);
		rssData += data;
		rssData += ":";
		//Node sensed RSS
		sprintf(data, "%f", MIN_RSS);
		rssData += data;
		rssData += "~";
		cout<<endl;
		//Node unique id
		sprintf(myData, "%d", myPair.first);
		fileData += myData;
		fileData += "~";
		//Node sensed RSS
		sprintf(myData, "%s",rssData.c_str());
		fileData += myData;
		//RSS data has already contain ~

		//Node sensed RSS
		sprintf(myData, "%d",period);
		fileData += myData;
		fileData += "~";

		//Packet length
		int start = 50;
		int end  = 100;
		int range = end - start + 1;
		int packetLength = rand() % range + start;
		//cout<<"Packet Length:"<<packetLength<<endl;
		sprintf(myData, "%d",packetLength);
		fileData += myData;


		//Write into file
		out <<fileData<< "\n";
		//Clear file data
		fileData = ("");
	}

	out.close();
}
/*
static void readTheScheduling()
{
	ifstream scheduleFile;
	scheduleFile.open(SCHEDULEINFORMATION.c_str());
	string line;
	char schedule[200];
	string scheduleData = ("");
	int i = 0;
	if (scheduleFile.is_open()) {
		while (getline(scheduleFile,line))
		{
			i++;
			//cout<<line<<endl;

			//Time
			sprintf(schedule, "%d", i);
			scheduleData += schedule;
			scheduleData += ":";
			//NodeId
			sprintf(schedule, "%d", atoi(line.c_str()));
			scheduleData += schedule;
			scheduleData += "~";
		}
	}
	scheduleFile.close();
	cout<<scheduleData<<endl;

}

static void runScheduling()
{

	system(MATLABSCHFILE.c_str());
	//After generating the schedule read the schedule
	readTheScheduling();
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
	double txPower = 0;
	uint32_t channelNumber = 11;

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
	Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
	Ptr<NakagamiPropagationLossModel> propModel = CreateObject<NakagamiPropagationLossModel> ();
	propModel->SetAttribute("m0",DoubleValue(1.0));
	propModel->SetAttribute("m1",DoubleValue(1.0));
	propModel->SetAttribute("m2",DoubleValue(1.0));
	channel->AddPropagationLossModel(propModel);

	Ptr<LogDistancePropagationLossModel> lognormalPropagationModel = CreateObject<LogDistancePropagationLossModel> ();
	channel->AddPropagationLossModel(lognormalPropagationModel);

	Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
	channel->SetPropagationDelayModel (delayModel);


	//channel->TraceConnectWithoutContext("PathLoss", MakeCallback(&pathLoss));



	//Configure Power of Transmission
	LrWpanSpectrumValueHelper svh;
	Ptr<SpectrumValue> psd
	= svh.CreateTxPowerSpectralDensity (txPower, channelNumber);

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
		//Set the transmission power
		wnode->device->GetPhy()->SetTxPowerSpectralDensity (psd);
		//Node Address Arrangements
		stringstream sstm;
		if((i+1) < 10)	sstm << "00:0" << (i+1);
		else sstm << "00:" << (i+1);
		wnode->device->SetAddress (Mac16Address (sstm.str().c_str()));
		wnode->device->SetChannel(channel);
		//Random period configuration
		int period = rand() % 4;
		wnode->period = PERIOD[period];
		//Node Id Configuration
		string nodeId = sstm.str().c_str();
		wnode->nodeId = nodeId;
		wnode->id = (i+1);
		wnode->node->AddDevice(wnode->device);

		//Location arrangements
		int locationY = rand()%((end - 0) + 1) + 0;
		int locationX = rand()%((end - start) + 1) + start;
		Ptr<ConstantPositionMobilityModel>  position =
				CreateObject<ConstantPositionMobilityModel> ();
		position->SetPosition (Vector (locationX,locationY,0));
		wnode->device->GetPhy ()->SetMobility (position);
		nodePositionHashTable [sstm.str().c_str()]
							   =  Vector (locationX,locationY,0);
		//Install mobility
		mobility.Install(wnode->node);
	}

	/**
	 * Controller position adjustment
	 */
	for(uint i = 0; i < wcontorllerNodesList.size(); i++)
	{
		WController* wcontroller = wcontorllerNodesList[i];
		wcontroller->device = CreateObject<LrWpanNetDevice> ();
		//Controller Address Arrangements
		stringstream sstm;
		sstm << "00:0" << (numberOfNodes+i+1);
		wcontroller->device->SetAddress (Mac16Address (sstm.str().c_str()));
		wcontroller->device->SetChannel(channel);
		string controllerId = sstm.str().c_str();
		wcontroller->controllerId = controllerId;
		wcontroller->id =  (numberOfNodes+i+1);
		//Set the transmission power
		wcontroller->device->GetPhy()->SetTxPowerSpectralDensity (psd);
		wcontroller->node->AddDevice(wcontroller->device);

		//Location Arrangements
		int controllerX = rand()%((end - 0) + 1) + 0;
		int controllerY = rand()%((end - start) + 1) + start;

		Ptr<ConstantPositionMobilityModel>  position
		= CreateObject<ConstantPositionMobilityModel> ();
		position->SetPosition (Vector (controllerX,controllerY,0));
		wcontroller->device->GetPhy ()->SetMobility (position);
		nodePositionHashTable [sstm.str().c_str()]
							   =  Vector (controllerX,controllerY,0);
		//Install Mobility
		mobility.Install(wcontroller->node);
	}

	MakeCallbacks(wnodeList);
	MakeCallbacks(wcontorllerNodesList);

	//Learn the topology
	learnTopology(wcontorllerNodesList);
	double second = 2.0;
	//Get the topology write into file
	Simulator::Schedule(Seconds(second),
			writeTopologyInformationToFile);
	second += 1.0;
	//Run the scheduling
	//Simulator::Schedule(Seconds(second), runScheduling);




	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 * Net Animator Configuration
	 * before the Simulator::Run
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
