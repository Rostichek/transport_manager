#include "Manager.h"

using namespace std;

double ConvertDegToRad(double degree) {
    return (degree * M_PI) / 180.0;
}

double ComputeDistance(const Coordinate& lhs, const Coordinate& rhs) {
    return acos(sin(ConvertDegToRad(lhs.latitude)) * sin(ConvertDegToRad(rhs.latitude)) +
        cos(ConvertDegToRad(lhs.latitude)) * cos(ConvertDegToRad(rhs.latitude)) *
        cos(fabs(ConvertDegToRad(lhs.longitude) - ConvertDegToRad(rhs.longitude)))
    ) * 6371000;
}

void TransportManager::AddStop(string stop_name, Coordinate coordinate) {
    stops_[stop_name] = make_unique<Stop>(stop_name, coordinate);
}

void TransportManager::AddBus(string bus_name, vector<string> stops, bool is_reversed) {
    buses_[bus_name] = make_unique<Bus>(bus_name, move(stops), is_reversed);
}

void TransportManager::AddDistance(const string& from, const string& to, int distance) {
    stops_[from].get()->AddDistance(to, distance);
}

const Bus* TransportManager::TransportManager::GetBus(const string& bus_name) const {
    if (buses_.count(bus_name))
        return buses_.at(bus_name).get();
    return nullptr;
}

const unordered_map<string, unique_ptr<Bus>>& TransportManager::GetBuses() const {
    return buses_;
}

const Stop* TransportManager::GetStop(const string& stop_name) const {
    if (stops_.count(stop_name))
        return stops_.at(stop_name).get();
    return nullptr;
}

int Bus::GetLength(const TransportManager& manager) const {
    double length = 0.0;
    for (size_t i = 0; i < stops_.size() - 1; ++i) {
        int distance_between_stops = 0;
        try {
            distance_between_stops = manager.GetStop(stops_[i])->GetDistance(stops_[i + 1]);
        }
        catch (...) {
            distance_between_stops = manager.GetStop(stops_[i+1])->GetDistance(stops_[i]);
        }
        length += distance_between_stops;
    }
    if (is_reversed_) {
        for (size_t i = stops_.size() - 1; i > 0; --i) {
            int distance_between_stops = 0;
            try {
                distance_between_stops = manager.GetStop(stops_[i])->GetDistance(stops_[i - 1]);
            }
            catch (...) {
                distance_between_stops = manager.GetStop(stops_[i - 1])->GetDistance(stops_[i]);
            }
            length += distance_between_stops;
        }
    }
    return length;
}