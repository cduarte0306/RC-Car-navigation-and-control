#include "navigation.hpp"
#include <osmium/io/reader.hpp>
#include <osmium/osm/types.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/relation.hpp>
#include <string>
#include <iterator>
#include <chrono>
#include <sstream>


namespace GPS {
    Navigation::Navigation(PeripheralCtrl* peripheralCtrl) 
        : peripheralCtrl_(peripheralCtrl), gpsDev(new GPSInterface("/dev/ttyUSB0", 9600)),
        currentLatitude_(0.0), currentLongitude_(0.0), isNavigating_(false), handler(this->availableWays, this->poiMapper) {
        
        osmium::io::Reader reader("/opt/map.osm");

        osmium::apply(reader, handler);
        reader.close();
        this->navThread = std::thread(&Navigation::myLocationUpdtaeLoop, this);
    }


    Navigation::~Navigation() {
        this->threadRun.store(false);
        this->navThread.join();
    }


    /** @brief Calculate the path to the target location.
     * 
     * This function will use the OSM data to calculate the path to the target location.
     * 
     * @param targetLocation The target location to navigate to.
     */
    void Navigation::calculatePath(Poi& targetLocation) {

    }


    /** @brief Start navigation to the target location.
     * 
     * This function will start the navigation to the target location. It will first calculate the path
     * and then start the navigation.
     * 
     * @param targetLocation The target location to navigate to.
     */
    void Navigation::startNavigation(Poi& targetLocation) {
        // Calculate the path first
        
    }


    /**
     * @brief Read the node and store its location.
     * 
     * @param node The OSM node to read.
     */
    void Navigation::node(const osmium::Node& node) {
        if (node.location()) {
            node_locations[node.id()] = node.location();
        }
    }


    /** @brief Update the current location based on GPS coordinates.
     * 
     * This function will update the current location based on the GPS coordinates provided.
     * It will find the nearest way and update the current location buffer with the address.
     * 
     * @param latitude The latitude of the current location.
     * @param longitude The longitude of the current location.
     */
    void Navigation::updateMyLocation(double latitude, double longitude) {
        std::stringstream currentloc;
        const osmium::Way* wayRef = nullptr;  // Reference to closest way
        double min_distance = std::numeric_limits<double>::max();  // Initialize minimm distance as a max double

        std::memset(this->currentLocationBuff, 0, strlen(this->currentLocationBuff));

        for (const osmium::Way* way : this->handler.ways) {
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

            for (const auto& [id, loc] : this->handler.node_locations) {
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


    void Navigation::myLocationUpdtaeLoop(void) {

        // Example: Get nearest node to current GPS location
        double lat, lon;

        // Read from the GPS device
        while (this->threadRun.load()) {
            try {
                std::lock_guard<std::mutex> lock(this->devGpsMutex);
                this->gpsDev->gpsDoInterface(lat, lon); 
            } catch (const std::exception& e) {
                continue; // Retry on error
            }

            if ( ! lat || !lon )
                continue;

            // Where am I?
            this->updateMyLocation(lat, lon);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    }
}

