#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <fastrtps/xmlparser/XMLProfileManager.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
int main(int argc,char**argv) {
  // Conversion and XML parsing only. No participant creation, ROS initialization or socket opening.
  pcl::PointCloud<pcl::PointXYZINormal> cloud(2,1);pcl::PCLPointCloud2 msg;
  pcl::toPCLPointCloud2(cloud,msg);
  using namespace eprosima::fastrtps::xmlparser;
  if(argc!=2)return 2;
  XMLProfileManager::loadDefaultXMLFile();
  // Profile must come from FASTRTPS_DEFAULT_PROFILES_FILE even with SKIP_DEFAULT_XML=1.
  eprosima::fastrtps::ParticipantAttributes attr;
  if(XMLProfileManager::fillParticipantAttributes("audit_local_transports",attr)!=XMLP_ret::XML_OK)return 3;
  std::cout<<"{\"point_size\":"<<sizeof(pcl::PointXYZINormal)<<",\"point_step\":"<<msg.point_step<<",\"row_step\":"<<msg.row_step
    <<",\"data_size\":"<<msg.data.size()<<",\"builtin_transports\":"<<(attr.rtps.useBuiltinTransports?"true":"false")<<",\"metatraffic_unicast_count\":"<<attr.rtps.builtin.metatrafficUnicastLocatorList.size()<<",\"metatraffic_multicast_count\":"<<attr.rtps.builtin.metatrafficMulticastLocatorList.size()<<",\"initial_peers_count\":"<<attr.rtps.builtin.initialPeersList.size()<<",\"transports\":[";
  bool first=true;
  for(auto&descriptor:attr.rtps.userTransports) {
    if(!first)std::cout<<',';first=false;
    if(auto shm=std::dynamic_pointer_cast<eprosima::fastdds::rtps::SharedMemTransportDescriptor>(descriptor))
      std::cout<<"{\"type\":\"SHM\",\"segment_size\":"<<shm->segment_size()<<",\"max_message_size\":"<<shm->max_message_size()<<",\"port_queue_capacity\":"<<shm->port_queue_capacity()<<'}';
    else if(auto udp=std::dynamic_pointer_cast<eprosima::fastdds::rtps::UDPv4TransportDescriptor>(descriptor))
      std::cout<<"{\"type\":\"UDPv4\",\"max_message_size\":"<<udp->maxMessageSize<<",\"send_buffer\":"<<udp->sendBufferSize<<",\"receive_buffer\":"<<udp->receiveBufferSize<<",\"initial_peers_range\":"<<udp->maxInitialPeersRange<<",\"interface_whitelist\":[";
    if(auto udp=std::dynamic_pointer_cast<eprosima::fastdds::rtps::UDPv4TransportDescriptor>(descriptor)) {
      bool firstAddress=true;for(auto &address:udp->interfaceWhiteList){if(!firstAddress)std::cout<<',';firstAddress=false;std::cout<<'"'<<address<<'"';}std::cout<<"]}";
    }
    else if(!std::dynamic_pointer_cast<eprosima::fastdds::rtps::SharedMemTransportDescriptor>(descriptor))return 4;
  }
  std::cout<<"],\"participant_created\":false,\"ROS_initialized\":false}"<<std::endl;
}
