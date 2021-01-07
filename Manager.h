#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <exception>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <functional>
#include <string>
#include <unordered_map>
#include <system_error>
#include <type_traits>
#include <map>
#include <iomanip>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include "graph.h"
#include "Json.h"
#include "router.h"
#include "svg.h"

using namespace std;

class TransportManager;
class Bus;
class Stop;

struct EdgeWeight {
    EdgeWeight() = default;
    EdgeWeight(int val) : time(val) {}
    string type;
    string text;
    double time = 0;
    size_t stop_count = 0;
};

namespace Map {
    class Map {
    public:
        Map() = delete;
        Map(const std::map<std::string, Json::Node>&, const TransportManager&);

        void RenderMap();

        string GetMap() const {
            return map;
        }

    private:
        enum class LayerType {
            BUS_LINES,
            BUS_LABELS,
            STOP_POINTS,
            STOP_LABELS
        };

        const unordered_map<string_view, LayerType> STR_TO_LAYER_TYPE = {
            {"bus_lines", LayerType::BUS_LINES},
            {"bus_labels", LayerType::BUS_LABELS},
            {"stop_points", LayerType::STOP_POINTS},
            {"stop_labels", LayerType::STOP_LABELS},
        };

        const TransportManager& manager;
        Svg::Document svg;
        string map = "";
        struct Properties {
            double width;
            double height;
            double padding;
            double stop_radius;
            double line_width;
            size_t stop_label_font_size;
            Svg::Point stop_label_offset;
            Svg::Color underlayer_color;
            double underlayer_width;
            vector<Svg::Color> color_palette;
            size_t bus_label_font_size;
            Svg::Point bus_label_offset;
            vector<LayerType> layers;
        } properties;

        struct Coeffitients {
            double min_lon, min_lat, max_lon, max_lat, zoom_coef;
        };

        void ComputeCoeff(Coeffitients& coeff);
        void AddRounds(const Coeffitients& coeff);
        void AddStops(const Coeffitients& coeff);
        void AddNames(const Coeffitients& coeff);
        void AddBusNames(const Coeffitients& coeff);

        
    };
}

struct Coordinate {
    double
        latitude = 0.0,
        longitude = 0.0;
};

double ComputeDistance(const Coordinate& lhs, const Coordinate& rhs);

class TransportManager {
public:
    TransportManager(size_t bus_wait_time, size_t bus_velocity) : 
        bus_wait_time_(bus_wait_time),
        bus_velocity_(bus_velocity)
    {}

    void AddStop(string stop_name, Coordinate coordinate);

    void AddBus(string bus_name, vector<string> stops, bool is_reversed);

    void AddDistance(const string& from, const string& to, int distance);

    const Bus* GetBus(const string& bus_name) const;

    vector<Graph::Edge<double>> GetRoute(const string& from, const string& to) const;

    const unordered_map<string, unique_ptr<Bus>>& GetBuses() const;

    const unordered_map<string, unique_ptr<Stop>>& GetStops() const;

    const Stop* GetStop(const string& stop_name) const;

    void BuildRouter();

    void BuildMap(std::map<std::string, Json::Node> properties) {
        map_ = make_unique<Map::Map>(properties, *this);
        map_.get()->RenderMap();
    }

    string GetMap() const {
        return string(map_.get()->GetMap());
    }

    void ReleaseRoute(uint64_t id) {
        router->ReleaseRoute(id);
    }

    const Coordinate& GetMinCoodinate() const {
        return min_coordinate;
    }

    const Coordinate& GetMaxCoodinate() const {
        return max_coordinate;
    }

private:
    Coordinate max_coordinate;
    Coordinate min_coordinate;

    size_t stops_coutner = 0;
    unordered_map<string, unique_ptr<Stop>> stops_;
    unordered_map<string, unique_ptr<Bus>> buses_;

    size_t bus_wait_time_ = 0;
    size_t bus_velocity_ = 0;

    unique_ptr<Graph::DirectedWeightedGraph<double>> graph_;
    unique_ptr<Graph::Router<double>> router;

    unique_ptr<Map::Map> map_;
};

class Stop {
public:
    Stop(const string& name, Coordinate coordinate, size_t indx) : name_(name), coordinate_(coordinate), indx_(indx) {}

    string_view GetName() { return string_view(name_); }

    void AddDistance(const string& to, int distance) { distances_[to] = distance; }

    int GetDistance(const string& to) const {
        return distances_.at(to);
    }

    const Coordinate& GetCoordinate() const { return coordinate_; }

    pair<size_t, size_t> GetIndx() const {
        return { indx_, indx_ + 1 };
    }

private:
    size_t indx_;
    string name_;
    Coordinate coordinate_;
    unordered_map<string, int> distances_;
};

class Bus{
public:
    Bus(const string& name, vector<string> stops, bool is_reversed) : name_(name), stops_(stops), is_reversed_(is_reversed) {}

    size_t GetStopsNum() const {
        if (is_reversed_) return stops_.size() * 2 - 1;
        return stops_.size();
    }

    size_t GetUniqueStopsNum() const {
        set<string> unique_stops(stops_.begin(), stops_.end());
        return unique_stops.size();
    }

    int GetLength(const TransportManager& manager) const;

    double GetGeographicDistance(const TransportManager& manager) const {
        double length = 0.0;
        for (size_t i = 0; i < stops_.size() - 1; ++i) {
            length += ComputeDistance(manager.GetStop(stops_[i])->GetCoordinate(), manager.GetStop(stops_[i + 1])->GetCoordinate());
        }
        if (is_reversed_) length *= 2;
        return length;
    }

    bool Find(const string& stop_name) const {
        return (find(stops_.begin(), stops_.end(), stop_name) != stops_.end());
    }

    const auto& GetStops() const {
        return stops_;
    }

    bool IsReversed() const { return is_reversed_; }

private:
    string name_;
    vector<string> stops_;
    bool is_reversed_;
};