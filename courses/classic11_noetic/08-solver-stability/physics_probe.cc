// Read-only native Classic transport request; no server plugin or control input.
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv)
{
  if (argc != 2) { std::cerr << "Expected world name\n"; return 2; }
  if (!gazebo::client::setup()) { std::cerr << "Client setup failed\n"; return 2; }
  int status = 1;
  try
  {
    // TransportIface takes seconds/nanoseconds, not an integer millisecond count.
    auto reply = gazebo::transport::request(argv[1], "physics_info", "",
        gazebo::common::Time(1, 500000000));
    gazebo::msgs::Physics physics;
    if (!reply || reply->response() != "success" ||
        reply->request() != "physics_info" || reply->type() != physics.GetTypeName() ||
        !physics.ParseFromString(reply->serialized_data()))
      throw std::runtime_error("Missing or invalid physics_info response");
    std::cout << physics.DebugString() << std::flush;
    status = 0;
  }
  catch (const std::exception &error) { std::cerr << error.what() << '\n'; }
  gazebo::client::shutdown();
  return status;
}
