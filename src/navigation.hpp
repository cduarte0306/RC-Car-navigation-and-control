#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP


#include "peripheral_driver.hpp"

#include "gps_interface/gps_interface.hpp"
#include <thread>
#include <osmium/osm/types.hpp>
#include <vector>
#include <string>

#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/sparse_mem_array.hpp>
#include <osmium/geom/haversine.hpp>
#include <unordered_map>
#include <limits>


class Navigation {
public:
    Navigation(PeripheralCtrl* peripheralCtrl);;
    ~Navigation();

    void startNavigation();
    void stopNavigation();
    void updatePosition(double latitude, double longitude);
    void getCurrentPosition(double& latitude, double& longitude) const;
private:
    PeripheralCtrl* peripheralCtrl_;
    GPSInterface* gpsInterface_;

    double currentLatitude_;
    double currentLongitude_;

    bool isNavigating_;
    std::thread navigationThread_;

    std::thread navThread;
    std::unordered_map<osmium::object_id_type, osmium::Location> node_locations;
    osmium::object_id_type current_node_id_ = 0;
    GPSInterface* gps = nullptr;

private:
    struct OSMNode {
        osmium::object_id_type id;
        double lat;
        double lon;
    };

    struct Way {
        std::vector<osmium::object_id_type> node_ids;
        std::string highway_type;
    };

    void navigationLoop();
    void node(const osmium::Node& node);

protected:
    class SimpleHandler : public osmium::handler::Handler {
    public:
        std::unordered_map<osmium::object_id_type, osmium::Location> node_locations;

        void node(const osmium::Node& node) {
            if (node.location()) {
                node_locations[node.id()] = node.location();
            }
        }
    };
};

#endif // NAVIGATION_HPP