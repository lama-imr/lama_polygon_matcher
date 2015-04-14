#include <place_matcher_mcc/jockey.h>

namespace place_matcher_mcc
{

Jockey::Jockey(const std::string& name) :
  lama_jockeys::LocalizingJockey(name),
  place_profile_interface_name_defined_(false)
{
  place_profile_interface_name_defined_ = private_nh_.getParam("place_profile_interface_name", place_profile_interface_name_);
  if (!place_profile_interface_name_defined_)
  {
    ROS_WARN_STREAM(private_nh_.getNamespace() << "/place_profile_interface_name parameter not defined");
  }

  initMapPlaceProfileInterface();
}

/** Create the getter proxies for PlaceProfile descriptors.
 */
void Jockey::initMapPlaceProfileInterface()
{
  ros::ServiceClient client = nh_.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  while (ros::ok() && !client.waitForExistence(ros::Duration(5)))
  {
    ROS_WARN("Waiting for service /interface_factory");
  }
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = place_profile_interface_name_;
  srv.request.interface_type = lama_interfaces::AddInterfaceRequest::SERIALIZED;
  srv.request.get_service_message = "lama_msgs/GetPlaceProfile";
  srv.request.set_service_message = "lama_msgs/SetPlaceProfile";
  if (!client.call(srv))
  {
    ROS_ERROR_STREAM("Failed to create the LaMa interface " << place_profile_interface_name_);
    return;
  }
  place_profile_getter_service_name_ = srv.response.get_service_name;
}

void Jockey::onGetDissimilarity()
{
  if (!place_profile_interface_name_defined_)
  {
    ROS_ERROR_STREAM(private_nh_.getNamespace() << "/place_profile_interface_name parameter not defined, aborting");
    server_.setAborted();
    return;
  }

  // Initialize the clients for the getter service (interface to map).
  ros::ServiceClient place_profile_getter;
  place_profile_getter = nh_.serviceClient<lama_msgs::GetPlaceProfile>(place_profile_getter_service_name_);
  while (ros::ok() && !place_profile_getter.waitForExistence(ros::Duration(5)))
  {
    ROS_WARN("Waiting for service /interface_factory");
  }

  getData();

  // Get all scans from database.
  lama_interfaces::ActOnMap srv;
  srv.request.action = lama_interfaces::ActOnMapRequest::GET_VERTEX_LIST;
  if (!map_agent_.call(srv))
  {
    ROS_ERROR_STREAM("Failed to call map agent \"" << map_agent_.getService() << "\"");
    server_.setAborted();
    return;
  }
  
  // Iterate over vertices and get the associated Polygon (from the PlaceProfile).
  std::vector<int32_t> vertices;
  vertices.reserve(srv.response.objects.size());
  std::vector<geometry_msgs::Polygon> polygons;
  polygons.reserve(srv.response.objects.size());
  for (size_t i = 0; i < srv.response.objects.size(); ++i)
  {
    // Get all PlaceProfile descriptors associated with the current vertex.
    lama_interfaces::ActOnMap desc_srv;
    desc_srv.request.action = lama_interfaces::ActOnMapRequest::GET_DESCRIPTOR_LINKS;
    desc_srv.request.object.id = srv.response.objects[i].id;
    desc_srv.request.interface_name = place_profile_interface_name_;
    if (!map_agent_.call(desc_srv))
    {
      ROS_ERROR_STREAM("Failed to call map agent \"" << map_agent_.getService() << "\"");
      server_.setAborted();
      return;
    }
    if (desc_srv.response.descriptor_links.empty())
    {
      continue;
    }
    if (desc_srv.response.descriptor_links.size() > 1)
    {
      ROS_WARN_STREAM("More than one descriptor with interface " <<
          place_profile_interface_name_ << " for vertex " <<
          desc_srv.request.object.id << ", taking the first one");
    }
    // Get the first linked PlaceProfile.
    lama_msgs::GetPlaceProfile profile_srv;
    profile_srv.request.id = desc_srv.response.descriptor_links[0].descriptor_id;
    if (!place_profile_getter.call(profile_srv))
    {
      ROS_ERROR_STREAM(jockey_name_ << ": failed to call service \"" <<
          place_profile_interface_name_ << "\"");
      server_.setAborted();
      return;
    }
    vertices.push_back(desc_srv.request.object.id);
    polygons.push_back(profile_srv.response.descriptor.polygon);
  }
  
  // Compare them to the current polygon.
  result_.idata.clear();
  result_.fdata.clear();
  result_.idata.reserve(vertices.size());
  result_.fdata.reserve(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    result_.idata.push_back(vertices[i]);
    result_.fdata.push_back(dissimilarity_getter_.getDissimilarity(current_polygon_, polygons[i]));
  }

  ROS_INFO_STREAM(jockey_name_ << ": computed " << result_.idata.size() << " dissimilarities");
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = getCompletionDuration();
  server_.setSucceeded(result_);
}

/** Start subscribers, wait for an appropriate message and exit upon completion.
 *
 * Start the subscribers, wait for a Polygon, a LaserScan, or a PlaceProfile
 * and exit upon reception of the first of those.
 */
void Jockey::getData()
{
  ros::Subscriber polygon_handler;
  polygon_handler = private_nh_.subscribe<geometry_msgs::Polygon>("polyoon", 1, &Jockey::handlePolygon, this);

  /* Wait a bit to avoid the first throttled message. */
  // TODO: simplify with ROS_INFO_STREAM_DELAYED_THROTTLE, when available.
  ros::Duration(0.2).sleep();
  ros::spinOnce();
  while (ros::ok())
  {
    ros::spinOnce();
    if (has_polygon_data_)
    {
      has_polygon_data_ = false;
      break;
    }
    ROS_INFO_STREAM_THROTTLE(5, "Did not received any polygon on " << polygon_handler.getTopic());
    ros::Duration(0.01).sleep();
  }
}

/** Callback for Polygon subscriber, receive a message and store it.
 */
void Jockey::handlePolygon(const geometry_msgs::PolygonConstPtr& msg)
{
  if (!has_polygon_data_)
  {
    current_polygon_ = *msg;
    has_polygon_data_ = true;
  }
}

} /* namespace place_matcher_mcc */

