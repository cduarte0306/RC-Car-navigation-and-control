#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP


#include "peripheral_driver.hpp"

#include "gps_interface/gps_interface.hpp"
#include <thread>
#include <osmium/osm/types.hpp>
#include <vector>


class Navigation {
public:
    Navigation();
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
};

#endif // NAVIGATION_HPP