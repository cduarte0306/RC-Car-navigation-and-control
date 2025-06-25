#include "navigation.hpp"
#include <osmium/io/reader.hpp>
#include <osmium/osm/types.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/relation.hpp>
#include <string>
#include <iterator>
#include <chrono>
#include <sstream>


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


void Navigation::updateMyLocation(OsmiumHandler& handler, double latitude, double longitude) {
    std::stringstream currentloc;
    const osmium::Way* wayRef = nullptr;  // Reference to closest way
    double min_distance = std::numeric_limits<double>::max();  // Initialize minimm distance as a max double

    std::memset(this->currentLocationBuff, 0, strlen(this->currentLocationBuff));

    for (const osmium::Way* way : handler.ways) {
        // Access way properties, e.g. way->id(), way->nodes(), etc.
        // Example: print way ID
        if (!way) 
            continue;

        // Find the nearest house I am at
        const char* house = way->tags()["addr:housenumber"];
        if (!house) {
            continue;
        }

        osmium::Location current_loc{longitude, latitude};

        for (const auto& [id, loc] : handler.node_locations) {
            double dist = osmium::geom::haversine::distance(current_loc, loc);
            if (dist < min_distance) {
                min_distance = dist;
                wayRef = way;
            }
        }
    }
    
    currentloc << wayRef->tags()["addr:housenumber"] << " "
            << wayRef->tags()["addr:street"] << ", " << wayRef->tags()["addr:city"] << " " 
            << wayRef->tags()["addr:state"] << ", " << wayRef->tags()["addr:postcode"];

    std::strncpy(this->currentLocationBuff, currentloc.str().c_str(), sizeof(this->currentLocationBuff));
}


void Navigation::navigationLoop(void) {
    osmium::io::Reader reader("/opt/map.osm");

    OsmiumHandler handler;
    osmium::apply(reader, handler);
    reader.close();
    
    std::cout << "Parsed " << handler.numNodes << " nodes and " 
              << handler.numWays << " ways." << std::endl;

    // Example: Get nearest node to current GPS location
    double lat, lon;

    // Read from the GPS device
    while (true) {
        bool read = false;
        do {
            try {
                this->gpsInterface_->gpsDoInterface(lat, lon); 
                read = true;
            } catch (const std::exception& e) {
                continue; // Retry on error
            }

            break;
        } while(!read && lat && lon);

        // Where am I?
        this->updateMyLocation(handler, lat, lon);

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

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    reader.close();
}
