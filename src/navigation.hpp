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
            Poi()=default;

            std::string& operator[](const std::string& key) {
                return poiContainer[key];  // creates if not exists
            }

            const std::string& operator[](const std::string& key) const {
                auto it = poiContainer.find(key);
                if (it != poiContainer.end()) return it->second;
                throw std::out_of_range("Key not found");
            }
        private:
            std::unordered_map<std::string, std::string> poiContainer; 
    };

    class Navigation {
    public:
        Navigation(PeripheralCtrl* peripheralCtrl);
        ~Navigation();

        void startNavigation();
        void stopNavigation();
        void updatePosition(double latitude, double longitude);
        void getCurrentPosition(double& latitude, double& longitude) const;

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
        PeripheralCtrl* peripheralCtrl_;
        GPSInterface* gpsInterface_ = nullptr;

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
        std::vector<std::string> availableWays;
        std::unordered_map<std::string, Poi> poiMapper;

    protected:
        // std::vector<osmium::Way> ways; 

        struct OsmiumHandler : public osmium::handler::Handler {
            OsmiumHandler(std::vector<std::string>& waysContainer, std::unordered_map<std::string, Poi>& poisContainer)
            : waysContainer_(waysContainer), poisContainer_(poisContainer), numNodes(0), numWays(0) {}

            std::vector<std::string>& waysContainer_;
            std::unordered_map<std::string, Poi>& poisContainer_;
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
                
                std::stringstream wayStream;

                if (way.tags().has_key("addr:housenumber")) {
                    wayStream << way.tags()["addr:housenumber"] << " "
                            << way.tags()["addr:street"] << ", " << way.tags()["addr:city"] << " " 
                            << way.tags()["addr:state"] << ", "  << way.tags()["addr:postcode"];
                    
                    this->waysContainer_.push_back(wayStream.str());
                    
                    // Add the way and it's totag to the map
                    Poi thisPoi;
                    thisPoi["housenumber"] = way.tags()["addr:housenumber"];
                    this->poisContainer_[wayStream.str()] = thisPoi;
                }
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

        OsmiumHandler handler;
        void navigationLoop();
        void node(const osmium::Node& node);
        void updateMyLocation(OsmiumHandler& handler, double latitude, double longitude);
    };

};

#endif // NAVIGATION_HPP