#include "Manager.h"

using namespace std;

EdgeWeight operator+ (const EdgeWeight& lhs, const EdgeWeight& rhs) {
    EdgeWeight tmp = rhs;
    tmp.time = lhs.time + rhs.time;
    return tmp;
}

bool operator> (const EdgeWeight& lhs, const EdgeWeight& rhs) {
    return lhs.time > rhs.time;
}

bool operator>= (const EdgeWeight& lhs, int rhs) {
    return lhs.time >= rhs;
}

bool operator< (const EdgeWeight& lhs, const EdgeWeight& rhs) {
    return lhs.time < rhs.time;
}

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
    stops_[stop_name] = make_unique<Stop>(stop_name, coordinate, stops_coutner);
    stops_coutner += 2;
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
    int length = 0;
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

void TransportManager::BuildRouter() {
    graph_ = make_unique<Graph::DirectedWeightedGraph<double>>(stops_.size() * 2);
    auto& graph = *graph_.get();
    Graph::Edge<double> edge;
    for (const auto& stop : stops_) {
        auto vertexes = stop.second->GetIndx();
        edge.from = vertexes.first;
        edge.to = vertexes.second;
        edge.type = "Wait";
        edge.text = stop.first;
        edge.weight = bus_wait_time_;
        graph.AddEdge(edge);
    }
    for (const auto& bus : buses_) {
        edge.type = "Bus";
        edge.text = bus.first;
        const auto& stops = bus.second->GetStops();
        for (size_t j = 0; j < stops.size() - 1; ++j) {
            double distance = 0.0;
            edge.stop_count = 0;
            for (size_t i = j; i < stops.size() - 1; ++i) {
                int distance_between_stops = 0;
                try {
                    distance_between_stops = GetStop(stops[i])->GetDistance(stops[i + 1]);
                }
                catch (...) {
                    distance_between_stops = GetStop(stops[i + 1])->GetDistance(stops[i]);
                }
                distance += distance_between_stops;
                edge.stop_count++;
                edge.from = GetStop(stops[j])->GetIndx().second;
                edge.to = GetStop(stops[i + 1])->GetIndx().first;
                edge.weight = (distance / (bus_velocity_ * 1000.0)) * 60;
                graph.AddEdge(edge);
            }
        }
        if (bus.second->IsReversed()) {
            for (size_t j = stops.size() - 1; j > 0; --j) {
                double distance = 0.0;
                edge.stop_count = 0;
                for (size_t i = j; i > 0; --i) {
                    int distance_between_stops = 0;
                    try {
                        distance_between_stops = GetStop(stops[i])->GetDistance(stops[i - 1]);
                    }
                    catch (...) {
                        distance_between_stops = GetStop(stops[i - 1])->GetDistance(stops[i]);
                    }
                    distance += distance_between_stops;
                    edge.weight = (distance / (bus_velocity_ * 1000.0)) * 60;
                    edge.stop_count++;
                    edge.from = GetStop(stops[j])->GetIndx().second;
                    edge.to = GetStop(stops[i - 1])->GetIndx().first;
                    graph.AddEdge(edge);
                }
            }
        }
    }
    router = make_unique<Graph::Router<double>>(graph);
}

vector<Graph::Edge<double>> TransportManager::GetRoute(const string& from, const string& to) const {
    vector<Graph::Edge<double>> items;
    const auto& info = router->BuildRoute(stops_.at(from)->GetIndx().first, stops_.at(to)->GetIndx().first);
    if (!info.has_value())
        return {};
    for (int i = 0; i < info->edge_count; ++i)
        items.push_back(graph_->GetEdge(router->GetRouteEdge(info->id, i)));
    //router->ReleaseRoute(info->id);
    return items;
}