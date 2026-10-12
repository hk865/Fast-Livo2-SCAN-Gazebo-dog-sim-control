#include <fast_livo2_core/runner.hpp>

#include "LIVMapper.h"

#include <image_transport/image_transport.hpp>

namespace fast_livo2_core
{

void run_mapping(const std::shared_ptr<rclcpp::Node> &node)
{
  auto core_node = node;
  image_transport::ImageTransport image_transport(core_node);
  LIVMapper mapper(core_node, core_node->get_name());
  mapper.initializeSubscribersAndPublishers(core_node, image_transport);
  mapper.run(core_node);
}

}  // namespace fast_livo2_core
