#include "navigation.hpp"
#include <osmium/io/reader.hpp>
#include <osmium/osm/types.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/relation.hpp>
#include <string>
#include <iterator>
#include <chrono>
#include <sstream>
#include <unordered_set>


namespace GPS {
    Navigation::Navigation(PeripheralCtrl* peripheralCtrl) 
        : peripheralCtrl_(peripheralCtrl), gpsDev(new GPSInterface("/dev/ttyUSB0", 9600)),
        latitude(0.0), longitude(0.0), isNavigating_(false), handler(this->availableWays, this->poiMapper) {
        
        // Parse the osm file
        this->parseMap();

        this->navThread = std::thread(&Navigation::myLocationUpdtaeLoop, this);
    }


    Navigation::~Navigation() {
        this->threadRun.store(false);
        this->navThread.join();
    }


    int Navigation::parseMap(void) {
        osmium::io::Reader reader("/opt/map.osm");

        osmium::apply(reader, handler);
        reader.close();

        for (const osmium::Way* way : this->handler.ways) {
            if(!way) continue;

            if (!way->tags().has_key("highway")) continue;

            GPS::Navigation::StreetInfo roadInfo;

            for (const auto& nodeRef : way->nodes()) {
                roadInfo.nodeIds.push_back(nodeRef.ref()); // Use ref() instead of id()
            }

            roadInfo.streetLength = this->getTransverseDistance(way);
            
            // Add roadInfo to streetInfoMap using the way's name as the key
            const char* wayName = way->tags()["name"];
            if (wayName) {
                this->streetInfoMap[std::string(wayName)] = roadInfo;
                this->streetInfoMap[std::string(wayName)].way = way;
                this->streetInfoMap[std::string(wayName)].streetName = std::string(wayName);
            }
        }

        // Parse the map
        for (const osmium::Way* way : this->handler.ways) {
            if(!way) continue;

            if (!way->tags().has_key("highway")) continue;
            GPS::Navigation::StreetInfo roadInfo;  // Road information object

            // Find the intersections
            for(const osmium::Way* way_ : this->handler.ways) {
                if (!way_->tags().has_key("highway") || way->id() == way_->id()) continue;

                for (const auto& nodeRef : way->nodes()) {
                    for(const auto& nodeRef2 : way_->nodes()) {
                        if (nodeRef == nodeRef2) {
                            // We've found the intersection, add it to the map
                            // Removed unused variable intSec
                            const char* streetName = way_->tags()["name"];
                            if (streetName) {
                                GPS::Navigation::IntersectionDescriptor inter;
                                inter.way = way_;
                                inter.intersectionStreetName = std::string(streetName);
                                inter.street = &this->streetInfoMap[way_->tags()["name"]];
                                inter.id     = nodeRef.ref();
                                roadInfo.intersections[nodeRef.ref()] = inter;  // Store the street name and reference
                                if(!way->tags().has_key("name"))
                                    continue;
                                
                                std::string wayName = way->tags()["name"];
                                this->streetInfoMap[wayName].intersections[nodeRef.ref()] = inter;
                            }
                        }
                    }
                }
            }    
        }

        return 0;
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


    /** @brief Calculate the path to the target location.
     * 
     * This function will use the OSM data to calculate the path to the target location.
     * 
     * @param targetLocation The target location to navigate to.
     */
    void Navigation::calculatePath(Poi& targetLocation) {
        // Find my current node
        const osmium::Way* targetWay = targetLocation.getWay();
        osmium::Location current_loc{this->longitude, this->latitude};

        // Generalized lambda: finds closest node to a given location, returns pair<node id, distance>
        auto findClosestNode = [this](const osmium::Location& loc) -> std::pair<const osmium::object_id_type*, double> {
            const osmium::object_id_type* closestNodeId = nullptr;
            double min_distance = std::numeric_limits<double>::max();
            for (const auto& [id, nodeloc] : this->handler.node_locations) {
                double dist = osmium::geom::haversine::distance(loc, nodeloc);
                if (dist < min_distance) {
                    min_distance = dist;
                    closestNodeId = &id;
                }
            }
            return {closestNodeId, min_distance};
        };
        
        // Find the closest node to the target way
        const osmium::object_id_type* closestNodeToTargetWay = nullptr;
        double minDistToTargetWay = std::numeric_limits<double>::max();
        if (targetWay && targetWay->nodes().size() > 0) {
            for (const auto& nodeRef : targetWay->nodes()) {
                auto it = this->handler.node_locations.find(nodeRef.ref());
                if (it != this->handler.node_locations.end()) {
                    auto [nodeId, dist] = findClosestNode(it->second);
                    if (dist < minDistToTargetWay) {
                        minDistToTargetWay = dist;
                        closestNodeToTargetWay = nodeId;
                    }
                }
            }
        }
        // closestNodeToTargetWay now points to the node id closest to the target way

        // Find my location's node and street address
        const osmium::object_id_type* closestNodeToCurrentLoc = nullptr;
        const osmium::Way* wayReference = nullptr; // <-- Add this to store the current way
        std::string wayName;

        // Find the closest node to the current location among all streets
        for (const auto& [streetName, streetInfo] : this->streetInfoMap) {
            auto maybeNodeIdPtr = streetInfo.getClosestNode(this->handler, current_loc);
            if (maybeNodeIdPtr) {
                closestNodeToCurrentLoc = maybeNodeIdPtr.value();
                wayName = streetName;
                break; // Found the closest node in a street, exit loop
            }
        }

        wayReference = this->streetInfoMap[wayName].way;        
        if(!wayReference) {
            return;
        }

        // Now we move on to iterating until we find our target location
        auto findElement = [](const std::unordered_set<std::string>& set, std::string& streetName) -> bool {
            auto iter = set.find(streetName);
            return iter == set.end();
        };

        StreetInfo* streetReference = &this->streetInfoMap[wayName];

        std::unordered_set<std::string> exploredPath;  // This stores the explored paths
        std::unordered_set<std::string> deadEnds;  // This stores the dead ends 
        
        std::vector<std::string> directions;
        std::vector<std::pair<double, std::vector<std::string>>> directionsBank;
        double distance = 0;

        // Search the map 
        while(true) {
            // First, we check if the node is in our current way reference
            if(streetReference->nodeExists(*closestNodeToTargetWay)) {
                // Make sure this path doesn't already exist
                for(auto it = directionsBank.begin(); it != directionsBank.end(); it++) {
                    
                }
                
                std::pair<double, std::vector<std::string>> entry = {distance, directions};
                directionsBank.push_back(entry);
                directions.clear();
                break;
            }

            const osmium::object_id_type* closestNodeId = nullptr;
            double minDistance = std::numeric_limits<double>::max();  // Initialize minimm distance as a max double
            
            // Node does not exist, now we search all intersections to see who's closest to the target 
            for (const auto& [interNode, intDescriptor] : streetReference->intersections) {
                StreetInfo* nextStreet = intDescriptor.street;

                // Make sure this isn't a dead end
                if(nextStreet->intersections.size() == 1) {
                    deadEnds.insert(streetReference->streetName);
                    continue;
                }

                // Avoid dead-ends
                if(findElement(deadEnds, nextStreet->streetName)) {
                    continue;
                }

                streetReference = nextStreet;
                distance += streetReference->streetLength;
                directions.push_back(streetReference->streetName);

                // osmium::Location nodeLoc    = this->handler.node_locations[interNode];
                // osmium::Location targetNode = this->handler.node_locations[*closestNodeToTargetWay];
                // double distance = osmium::geom::haversine::distance(nodeLoc, targetNode);
                // if(minDistance < distance ) {
                //     minDistance = distance;
                    
                //     // Point to the next street
                //     streetReference = *nextStreet;
                //     exploredPath.insert(streetReference.streetName);
                //     break;
                // }
            }
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
    void Navigation::updateMyLocation(void) {
        std::lock_guard<std::mutex> lock(this->devGpsMutex);

        std::stringstream currentloc;
        const osmium::Way* wayRef = nullptr;  // Reference to closest way
        double min_distance = std::numeric_limits<double>::max();  // Initialize minimm distance as a max double

        std::memset(this->currentLocationBuff, 0, sizeof(this->currentLocationBuff));

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

            osmium::Location current_loc{this->longitude, this->latitude};

            for (const auto& [id, loc] : this->handler.node_locations) {
                double dist = osmium::geom::haversine::distance(current_loc, loc);
                if (dist < min_distance) {
                    min_distance = dist;
                    wayRef = way;
                }
            }
        }
        
        currentloc << wayRef->tags()["addr:housenumber"] << " "
                   << wayRef->tags()["addr:street"]      << ", " << wayRef->tags()["addr:city"] << " " 
                   << wayRef->tags()["addr:state"]       << ", " << wayRef->tags()["addr:postcode"];

        std::strncpy(this->currentLocationBuff, currentloc.str().c_str(), sizeof(this->currentLocationBuff));
    }


    void Navigation::myLocationUpdtaeLoop(void) {

        // Example: Get nearest node to current GPS location
        double lat, lon;
        int ret;

        // Read from the GPS device
        while (this->threadRun.load()) {

            ret = this->gpsDev->gpsDoInterface(lat, lon);
            if (ret < 0) {
                continue;
            } 

            this->latitude  = lat;
            this->longitude = lon;

            // Where am I?
            this->updateMyLocation();

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    }
}

