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
#include <system_error>
#include <type_traits>
#include <iomanip>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include "graph.h"
#include "router.h"

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

    const Stop* GetStop(const string& stop_name) const;

    void BuildRouter();

    void ReleaseRoute(uint64_t id) {
        router->ReleaseRoute(id);
    }

private:
    size_t stops_coutner = 0;
    unordered_map<string, unique_ptr<Stop>> stops_;
    unordered_map<string, unique_ptr<Bus>> buses_;

    size_t bus_wait_time_ = 0;
    size_t bus_velocity_ = 0;

    unique_ptr<Graph::DirectedWeightedGraph<double>> graph_;
    unique_ptr<Graph::Router<double>> router;
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

    const auto& GetStops() {
        return stops_;
    }

    bool IsReversed() { return is_reversed_; }

private:
    string name_;
    vector<string> stops_;
    bool is_reversed_;
};