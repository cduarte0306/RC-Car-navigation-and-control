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
#include <utility>


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
                roadInfo.nodeIds.insert(nodeRef.ref()); // Use ref() instead of id()
                this->nodeToWayMap[nodeRef.ref()] = way;
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

        // Find then 

        return 0;
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


    /**
     * @brief Stops all navigation
     * 
     */
    void Navigation::stopNavigation(void) {
        this->isNavigating_.store(false);
    }


    /**
     * @brief Get the directions based on the current position
     * 
     */
    void Navigation::getDirections(void) {
        osmium::Location current_loc{this->longitude, this->latitude};
    }


    /** @brief Calculate the path to the target location.
     * 
     * This function will use the OSM data to calculate the path to the target location.
     * 
     * @param targetLocation The target location to navigate to.
     */
    int Navigation::startNavigation(std::string& targetLocation) {
        this->isNavigating_.store(true);
        
        // Do not run the thread if already running
        if(this->navigationThread_.joinable()) return 0;
        
        // Starting the navigation thread
        this->navigationThread_ = std::thread([&]() {
            std::unordered_map<osmium::object_id_type, std::vector<Edge>> mapGraph;

            while(this->isNavigating_.load()) {
                // Map graph buffer
                decltype(mapGraph) _mapGraph;

                auto it = this->poiMapper.find(targetLocation);
                if(it == this->poiMapper.end()) {
                    return -1;
                }

                Poi& poi = this->poiMapper[targetLocation];
                const osmium::Way* targetWay = poi.getWay();
                osmium::Location current_loc{this->longitude, this->latitude};

                // Find the target node
                osmium::object_id_type targetNode;
                const osmium::Way* targetRoad = this->streetInfoMap[poi["street-addr"]].way;
                double minDist = std::numeric_limits<double>::max();

                for (const auto& houseNode : targetWay->nodes()) {
                    auto itHouse = this->handler.node_locations.find(houseNode.ref());
                    if (itHouse == this->handler.node_locations.end()) continue;

                    const osmium::Location& houseLoc = itHouse->second;
                    if (!houseLoc.valid()) continue;

                    for (const auto& roadNode : targetRoad->nodes()) {
                        auto itRoad = this->handler.node_locations.find(roadNode.ref());
                        if (itRoad == this->handler.node_locations.end()) continue;

                        const osmium::Location& roadLoc = itRoad->second;
                        if (!roadLoc.valid()) continue;

                        double dist = osmium::geom::haversine::distance(houseLoc, roadLoc);
                        if (dist < minDist) {
                            minDist = dist;
                            targetNode = roadNode.ref();
                        }
                    }
                }

                // Find my location's node and street address
                const osmium::object_id_type startingNode = this->getClosestNode(this->handler, current_loc); 

                // Generate a graph
                std::unordered_map<osmium::object_id_type, int> nodeOcurrenceCounter;

                for (const osmium::Way* way : this->handler.ways) {
                    if (!way->tags().has_key("highway")) continue;
                    for (const auto& node : way->nodes()) {
                        nodeOcurrenceCounter[node.ref()]++;
                    }
                }

                for (const auto& [streetName, street] : this->streetInfoMap) {
                    osmium::object_id_type lastNode = *street.nodeIds.begin();
                    const osmium::Way* way = street.way;

                    for(auto it = way->nodes().begin(); it != way->nodes().end(); ++it) {
                        const osmium::object_id_type nodeRef = (*it).ref();

                        if (nodeRef == lastNode) continue;

                        if ((nodeOcurrenceCounter[nodeRef] > 1) || 
                            (nodeRef == targetNode)             || 
                            (nodeRef == startingNode)           || 
                            it == way->nodes().end() - 1) {
                            const auto& loc1 = this->handler.node_locations[lastNode];
                            const auto& loc2 = this->handler.node_locations[nodeRef];
                            if (!loc1.valid() || !loc2.valid()) continue;

                            double distance = osmium::geom::haversine::distance(loc1, loc2);

                            // Forward edge
                            Edge edge{nodeRef, street.streetName, distance};
                            _mapGraph[lastNode].push_back(edge);

                            // Reverse edge
                            Edge reverse{lastNode, street.streetName, distance};
                            _mapGraph[nodeRef].push_back(reverse);

                            lastNode = nodeRef;
                        }
                    }
                }

                std::unordered_map<osmium::object_id_type, osmium::object_id_type> cameFrom;
                std::unordered_map<osmium::object_id_type, double> costSoFar;

                using QEntry = std::pair<double, osmium::object_id_type>;
                std::priority_queue<QEntry, std::vector<QEntry>, std::greater<>> frontier;

                frontier.emplace(0.0, startingNode);
                cameFrom[startingNode] = startingNode;
                costSoFar[startingNode] = 0.0;

                while (!frontier.empty()) {
                    auto [currentCost, currentNode] = frontier.top();
                    frontier.pop();

                    if (currentNode == targetNode) {
                        break; // found the target
                    }

                    for (const Edge& edge : _mapGraph[currentNode]) {
                        double newCost = costSoFar[currentNode] + edge.weight;
                        if (costSoFar.find(edge.id) == costSoFar.end() || newCost < costSoFar[edge.id]) {
                            costSoFar[edge.id] = newCost;
                            cameFrom[edge.id] = currentNode;
                            frontier.emplace(newCost, edge.id);
                        }
                    }
                }

                for (osmium::object_id_type node = targetNode; node != startingNode; node = cameFrom[node]) {
                    directions.push_back(node);
                }

                directions.push_back(startingNode);
                std::reverse(directions.begin(), directions.end());

                // Build the directions struct 
                mapGraph = _mapGraph;
            }
        });
        
        return 0;
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

