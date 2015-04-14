/*
 * Short description.
 *
 * Long description:
 * - description of general role
 * - description of used actions with associated result (DONE when, FAIL if...)
 *
 * Interaction with the map (created by this jockey):
 * - [Getter][/][Setter], message type, interface default name
 * - Getter/Setter: VectorLaserScan, jockey_name + "_laser_descriptor"
 *
 * Interaction with the map (created by other jockeys):
 * - [Getter][/][Setter], message type, interface default name
 * - Getter: VectorLaserScan, "laser_descriptor"
 *
 * Subscribers (other than map-related):
 * - message type, topic default name, description
 * - sensor_msgs::LaserScan, "~/base_scan", 360-deg laser-scan.
 *
 * Publishers (other than map-related):
 * - message type, topic default name, description
 * - nav_msgs::Pose, "~/pose", robot pose
 *
 * Services used (other than map-related):
 * - service type, server default name, description
 * - place_matcher_msgs::PolygonDissimilarity, "~/dissimilarity_server", used to
 *    compare all known places with the current place
 *
 * Parameters:
 * - name, type, default value, description
 */

#ifndef PLACE_MATCHER_MCC_JOCKEY_H
#define PLACE_MATCHER_MCC_JOCKEY_H

#include <vector>

#include <geometry_msgs/Polygon.h>
#include <lama_interfaces/ActOnMap.h>
#include <lama_interfaces/AddInterface.h>
#include <lama_jockeys/localizing_jockey.h>
#include <lama_msgs/GetPlaceProfile.h>

#include <place_matcher_mcc/dissimilarity_getter.h>

namespace place_matcher_mcc
{

class Jockey : public lama_jockeys::LocalizingJockey
{
  public :

    Jockey(const std::string& name);

    void onGetDissimilarity();

    double getSampleCount() {return dissimilarity_getter_.sample_count;}
    void setSampleCount(double sample_count) {dissimilarity_getter_.sample_count = sample_count;}

    double getScaleCount() {return dissimilarity_getter_.scale_count;}
    void setScaleCount(double scale_count) {dissimilarity_getter_.scale_count = scale_count;}

    bool getRotationInvariance() {return dissimilarity_getter_.rotation_invariance;}
    void setRotationInvariance(bool rotation_invariance) {dissimilarity_getter_.rotation_invariance = rotation_invariance;}

  private :

    void initMapPlaceProfileInterface();
    void getData();
    void handlePolygon(const geometry_msgs::PolygonConstPtr& msg);

    // Map interface for PlaceProfile descriptors.
    std::string place_profile_interface_name_;
    bool place_profile_interface_name_defined_;
    std::string place_profile_getter_service_name_;

    DissimilarityGetter dissimilarity_getter_;
    bool has_polygon_data_;
    geometry_msgs::Polygon current_polygon_;

};

} /* namespace place_matcher_mcc */

#endif /* PLACE_MATCHER_MCC_JOCKEY_H */

