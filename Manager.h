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

using namespace std;

class TransportManager;
class Bus;
class Stop;

struct Coordinate {
    double
        latitude = 0.0,
        longitude = 0.0;
};

double ComputeDistance(const Coordinate& lhs, const Coordinate& rhs);

class TransportManager {
public:
    TransportManager() = default;

    void AddStop(string stop_name, Coordinate coordinate);

    void AddBus(string bus_name, vector<string> stops, bool is_reversed);

    void AddDistance(const string& from, const string& to, int distance);

    const Bus* GetBus(const string& bus_name) const;

    const unordered_map<string, unique_ptr<Bus>>& GetBuses() const;

    const Stop* GetStop(const string& stop_name) const;

private:
    unordered_map<string, unique_ptr<Stop>> stops_;
    unordered_map<string, unique_ptr<Bus>> buses_;
};

class Stop : public TransportManager {
public:
    Stop(const string& name, Coordinate coordinate) : name_(name), coordinate_(coordinate) {}

    string_view GetName() { return string_view(name_); }

    void AddDistance(const string& to, int distance) { distances_[to] = distance; }

    int GetDistance(const string& to) const {
        return distances_.at(to);
    }

    const Coordinate& GetCoordinate() const { return coordinate_; }

private:
    string name_;
    Coordinate coordinate_;
    unordered_map<string, int> distances_;
};

class Bus : public TransportManager {
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

private:
    string name_;
    vector<string> stops_;
    bool is_reversed_;
};