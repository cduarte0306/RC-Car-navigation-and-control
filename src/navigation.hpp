#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP


#include "peripheral_driver.hpp"

#include "gps_interface/gps_interface.hpp"
#include <thread>
#include <osmium/osm/types.hpp>
#include <vector>
#include <string>
#include <iostream>

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
protected:
    PeripheralCtrl* peripheralCtrl_;
    GPSInterface* gpsInterface_;

    double currentLatitude_;
    double currentLongitude_;

    double lastLatDiff = -1;
    double lastLonDiff = -1;

    bool isNavigating_;
    std::thread navigationThread_;

    char currentLocationBuff[256] = {0};

    std::thread navThread;
    std::unordered_map<osmium::object_id_type, osmium::Location> node_locations;
    osmium::object_id_type current_node_id_ = 0;
    GPSInterface* gps = nullptr;

protected:
    // std::vector<osmium::Way> ways; 

    struct OsmiumHandler : public osmium::handler::Handler {
        int numNodes, numWays = 0;

        std::unordered_map<osmium::object_id_type, osmium::Location> node_locations;
        std::vector<const osmium::Way*> ways; // Store pointers to all parsed ways

        void node(const osmium::Node& node) {
            this->numNodes++;

            if (node.location()) {
                node_locations[node.id()] = node.location();
            }
        }

        void way(const osmium::Way& way) {
            this->numWays ++;
            this->ways.push_back(&way); // Store pointer to the way
        }
    };

protected:
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
    void updateMyLocation(OsmiumHandler& handler, double latitude, double longitude);
};

#endif // NAVIGATION_HPP