#include "navigation.hpp"
#include <osmium/io/reader.hpp>
#include <osmium/osm/types.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/relation.hpp>


Navigation::Navigation() 
    : peripheralCtrl_(new PeripheralCtrl()), gpsInterface_(new GPSInterface()),
      currentLatitude_(0.0), currentLongitude_(0.0), isNavigating_(false) {

    this->navThread = std::thread(&Navigation::navigationLoop, this);
}


Navigation::~Navigation() {
}


void Navigation::navigationLoop(void) {
    osmium::object_id_type id;
    double lat;
    double lon;

    auto osmGraph = OSMGraph("map.osm");
    auto start = osmGraph.closestNode(current_lat, current_lon);
    auto goal  = osmGraph.closestNode(goal_lat, goal_lon);
    auto path = osmGraph.findShortestPath(start, goal);
    
    while(true) {
        this->gpsInterface_->getCoordinates(lat, lon);
        

    }
}
