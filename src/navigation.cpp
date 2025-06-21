#include "navigation.hpp"
#include <osmium/io/reader.hpp>
#include <osmium/osm/types.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/relation.hpp>


Navigation::Navigation(PeripheralCtrl* peripheralCtrl) 
    : peripheralCtrl_(peripheralCtrl), gpsInterface_(new GPSInterface("/dev/ttyUSB0", 9600)),
      currentLatitude_(0.0), currentLongitude_(0.0), isNavigating_(false) {

    this->navThread = std::thread(&Navigation::navigationLoop, this);
}


Navigation::~Navigation() {
}


void Navigation::node(const osmium::Node& node) {
    if (node.location()) {
        node_locations[node.id()] = node.location();
    }
}


void Navigation::navigationLoop(void) {
    osmium::io::Reader reader("/opt/map.osm");

    SimpleHandler handler;
    osmium::apply(reader, handler);
    reader.close();

    // Example: Get nearest node to current GPS location
    double lat, lon;

    while (true) {
      while (true) {
        try {
          this->gpsInterface_->gpsDoInterface(lat, lon); 
        } catch (const std::exception& e) {
          continue; // Retry on error
        }

        break;
      }

      osmium::Location current_loc{lon, lat};
      osmium::object_id_type closest_node_id = 0;
      double min_distance = std::numeric_limits<double>::max();

      for (const auto& [id, loc] : handler.node_locations) {
          double dist = osmium::geom::haversine::distance(current_loc, loc);
          if (dist < min_distance) {
              min_distance = dist;
              closest_node_id = id;
          }
      }

      // Save for pathfinding later
      this->current_node_id_ = closest_node_id;
    }

    
}
