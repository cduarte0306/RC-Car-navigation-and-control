#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP


#include "peripheral_driver.hpp"

#include "gps_interface/gps_interface.hpp"
#include <thread>
#include <osmium/osm/types.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <mutex>
#include <atomic>
#include <optional>
#include <memory>
#include <unordered_set>
#include <set>

#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/sparse_mem_array.hpp>
#include <osmium/geom/haversine.hpp>
#include <unordered_map>
#include <limits>
#include <nlohmann/json.hpp>


namespace GPS {
    class Poi {
        public:
            Poi(const osmium::Way* way = nullptr) : way(way) { }

            std::string& operator[](const std::string& key) {
                return poiContainer[key];  // creates if not exists
            }

            std::vector<std::string> ways() const {
                std::vector<std::string> result;
                for (const auto& pair : poiContainer) {
                    result.push_back(pair.first);
                }

                return result;
            }

            const std::string& operator[](const std::string& key) const {
                auto it = poiContainer.find(key);
                if (it != poiContainer.end()) return it->second;
                throw std::out_of_range("Key not found");
            }
            
            const osmium::Way* getWay() const {
                return way;
            }

        private:
            std::unordered_map<std::string, std::string> poiContainer; 
            const osmium::Way* way = nullptr;
    };

    class Navigation {
    public:
        Navigation(PeripheralCtrl* peripheralCtrl);
        ~Navigation();

        std::vector<std::string> ways() const {
            std::vector<std::string> result;
            for (const auto& pair : this->poiMapper) {
                result.push_back(pair.first);
            }

            return result;
        }

        int calculatePath(std::string& targetLocation);
        void startNavigation(Poi& targetLocation);
        void stopNavigation();
        void getDirection();
        void updatePosition(double latitude, double longitude);
        void getCurrentPosition(double& latitude, double& longitude) const;
        void updateMyLocation(void);

        void getCurrentAddress(std::string& currentLoc) const {
            currentLoc = std::string(currentLocationBuff);
        }

        void getCurrentAddress(Poi& currentLoc) const {
            auto it = this->poiMapper.find(std::string(this->currentLocationBuff));
            if (it != this->poiMapper.end()) {
                currentLoc = it->second;
            } else {
                throw std::out_of_range("Current address not found in poiMapper");
            }
        }

        const std::vector<std::string>& getAvailableWays(void) const {
            return this->availableWays;
        }
        
    protected:

        struct OsmiumHandler : public osmium::handler::Handler {
            OsmiumHandler(std::vector<std::string>& waysContainer, std::unordered_map<std::string, Poi>& poisContainer)
            : waysContainer_(waysContainer), poisContainer_(poisContainer), numNodes(0), numWays(0) {}

            std::vector<std::string>& waysContainer_;
            std::unordered_map<std::string, Poi>& poisContainer_;
            int numNodes, numWays = 0;

            std::unordered_map<osmium::object_id_type, osmium::Location> node_locations;
            std::unordered_map<osmium::object_id_type, const osmium::Way*> waysMap;
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
                
                std::stringstream wayStream;

                if (way.tags().has_key("addr:housenumber")) {
                    wayStream << way.tags()["addr:housenumber"] << " "
                              << way.tags()["addr:street"]      << ", " << way.tags()["addr:city"] << " " 
                              << way.tags()["addr:state"]       << ", " << way.tags()["addr:postcode"];
                    
                    this->waysContainer_.push_back(wayStream.str());
                    
                    // Add the way and it's totag to the map
                    Poi thisPoi(&way);
                    thisPoi["housenumber"] = way.tags()["addr:housenumber"];
                    thisPoi["street-addr"] = way.tags()["addr:street"];
                    thisPoi["full-addr"]   = wayStream.str();
                    
                    this->poisContainer_[wayStream.str()] = thisPoi;
                }

                this->waysMap[way.id()] = &way;
            }
        };

        
        struct StreetInfo;
        struct MapNode;

        struct IntersectionDescriptor {
            MapNode* node = nullptr;
            std::string intersectionStreetName;
            osmium::object_id_type id;
            const osmium::Way* way = nullptr;
            StreetInfo* street = nullptr; // Remove 'struct' and use the correct scope
        };

        struct MapNode {
            MapNode(const osmium::object_id_type* id, osmium::Location& loc) : id(id), loc(loc) {
            }

            std::map<std::string, std::pair<std::unique_ptr<MapNode>, StreetInfo*>> edges;  // Maps the edge to it's street name
            const osmium::object_id_type* id = nullptr;  // ID of the vertice
            osmium::Location loc;
        };

        struct Edge {
            osmium::object_id_type id;
            std::string streetName;
            double weight;
        };

        struct StreetInfo {
            std::vector<osmium::Location> nodeLocations; 
            std::set<osmium::object_id_type> nodeIds;  // All nodes in the street
            const osmium::Way* way;
            std::unordered_map<osmium::object_id_type, IntersectionDescriptor> intersections;

            std::string streetName = "";
            std::string highway_type;
            double streetLength;

            // Returns true if the node exists in this street, false otherwise
            bool hasNode(const osmium::object_id_type node) const {
                return nodeIds.find(node) != nodeIds.end() ;
            }
        };

        struct Paths {
            const StreetInfo* way;
            struct Paths* prev;
            struct Paths* next;
        };

        // std::vector<osmium::Way> ways;
        struct PathNode {
            
            std::vector<GPS::Navigation::PathNode*> neighborsNodes;
        };

        OsmiumHandler handler;
        Paths* path = nullptr;
        void myLocationUpdtaeLoop();
        void node(const osmium::Node& node);

        PeripheralCtrl* peripheralCtrl_;
        GPSInterface* gpsDev = nullptr;

        double latitude;
        double longitude;

        double lastLatDiff = -1;
        double lastLonDiff = -1;

        bool isNavigating_;
        std::thread navigationThread_;
        std::thread wayfinder;

        char currentLocationBuff[256] = {0};

        std::thread navThread;
        
        std::unordered_map<osmium::object_id_type, std::vector<Edge>> mapGraph;
        std::unordered_map<osmium::object_id_type, osmium::Location> node_locations;
        std::unordered_map<std::string, GPS::Navigation::StreetInfo> streetInfoMap;
        std::unordered_map<osmium::object_id_type, const osmium::Way*> nodeToWayMap;
    
        std::vector<GPS::Navigation::StreetInfo> directions;

        osmium::object_id_type current_node_id_ = 0;
        std::vector<std::string> availableWays;
        std::unordered_map<std::string, Poi> poiMapper;

        std::mutex devGpsMutex;
        std::atomic<bool> threadRun = {true};

        
        /**
         * @brief Get the Transverse Distance of a road
         * 
         * @param way 
         * @return double 
         */
        inline double getTransverseDistance(const osmium::Way* way) const {
            if (!way || way->nodes().size() < 2) return 0.0;
            double total = 0.0;
            auto it = way->nodes().begin();
            auto prev = it++;
            for (; it != way->nodes().end(); ++it, ++prev) {
                auto loc1 = this->handler.node_locations.find(prev->ref());
                auto loc2 = this->handler.node_locations.find(it->ref());
                if (loc1 != this->handler.node_locations.end() && loc2 != this->handler.node_locations.end()) {
                    total += osmium::geom::haversine::distance(loc1->second, loc2->second);
                }
            }
            return total;
        }

        const osmium::object_id_type* getClosestNode(OsmiumHandler& handler, osmium::Location& loc) const {
                const osmium::object_id_type* closestNodeId = nullptr;
                double min_distance = std::numeric_limits<double>::max();
                for (const auto& [id, nodeloc] : handler.node_locations) {
                    if(this->nodeToWayMap.find(id) == this->nodeToWayMap.end()) continue;
                    double dist = osmium::geom::haversine::distance(loc, nodeloc);
                    if (dist < min_distance) {
                        min_distance = dist;
                        closestNodeId = &id;
                    }
                }

                return closestNodeId;
            }

        int parseMap(void);
    };

};

//set-waypoint '11 Arbor Lane, Bordentown NJ, 08505'
#endif // NAVIGATION_HPP