#include "Manager.h"
#include <algorithm>
#include <set>

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
    if (stops_coutner == 0) {
        min_coordinate = max_coordinate = coordinate;
    } else {
        min_coordinate.latitude = min(min_coordinate.latitude, coordinate.latitude);
        min_coordinate.longitude = min(min_coordinate.longitude, coordinate.longitude);
        max_coordinate.latitude = max(max_coordinate.latitude, coordinate.latitude);
        max_coordinate.longitude = max(max_coordinate.longitude, coordinate.longitude);
    }
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

const unordered_map<string, unique_ptr<Stop>>& TransportManager::GetStops() const {
    return stops_;
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


Svg::Color ReadColor(const Json::Node& json_color) {
    try {
        const auto& color = json_color.AsArray();
        Svg::Rgb rgb;
        rgb.red = color.at(0).AsNumber();
        rgb.green = color.at(1).AsNumber();
        rgb.blue = color.at(2).AsNumber();
        if (color.size() == 4) {
            rgb.alpha.emplace(color.at(3).AsNumber());
        }
        return Svg::Color(rgb);
    }
    catch (...) {
        return Svg::Color(json_color.AsString());
    }
   
}

Map::Map::Map(const std::map<std::string, Json::Node>& json_properties, const TransportManager& manager) : manager(manager) {
    properties.width = json_properties.at("width").AsNumber();
    properties.height = json_properties.at("height").AsNumber();
    properties.padding = json_properties.at("padding").AsNumber();
    properties.stop_radius = json_properties.at("stop_radius").AsNumber();
    properties.line_width = json_properties.at("line_width").AsNumber();
    properties.stop_label_font_size = json_properties.at("stop_label_font_size").AsNumber();
    properties.bus_label_font_size = json_properties.at("bus_label_font_size").AsNumber();
    properties.stop_label_offset = {
        json_properties.at("stop_label_offset").AsArray().front().AsNumber(),
        json_properties.at("stop_label_offset").AsArray().back().AsNumber()
    };
    properties.bus_label_offset = {
        json_properties.at("bus_label_offset").AsArray().front().AsNumber(),
        json_properties.at("bus_label_offset").AsArray().back().AsNumber()
    };
    properties.underlayer_color = ReadColor(json_properties.at("underlayer_color"));
    properties.underlayer_width = json_properties.at("underlayer_width").AsNumber();
    const auto& pallete = json_properties.at("color_palette").AsArray();
    properties.color_palette.reserve(pallete.size());
    for (const auto& color : pallete) {
        properties.color_palette.push_back(ReadColor(color));
    }
    const auto& layers = json_properties.at("layers").AsArray();
    properties.layers.reserve(layers.size());
    for (const auto& layer : layers) {
        properties.layers.push_back(STR_TO_LAYER_TYPE.at(layer.AsString()));
    }
    ComputeStopsCoordinates();
}

void Map::Map::AddRounds() {
    const auto& buses = manager.GetBuses();
    set<string_view> bus_names;
    for (const auto& bus : buses)
        bus_names.insert(bus.first);
    size_t bus_num = 0;
    for (const auto& bus_name : bus_names) {
        const auto* bus = manager.GetBus(string(bus_name));
        const auto& stops = bus->GetStops();
        Svg::Polyline round;
        round.SetStrokeColor(properties.color_palette.at(bus_num % (properties.color_palette.size())))
            .SetStrokeWidth(properties.line_width)
            .SetStrokeLineCap("round")
            .SetStrokeLineJoin("round");
        for (const auto& stop_name : stops) {
            const auto& stop_coord = stops_coodinates.at(stop_name);
            round.AddPoint({
                    stop_coord.longitude,
                    stop_coord.latitude
                });
        }
        if (bus->IsReversed()) {
            for (auto it = stops.rbegin() + 1; it != stops.rend(); it++) {
                const auto& stop_coord = stops_coodinates.at(*it);
                round.AddPoint({
                    stop_coord.longitude,
                    stop_coord.latitude
                    });
            }
        }
        svg.Add(round);
        bus_num++;
    }
}

void Map::Map::AddBusNames() {
    const auto& buses = manager.GetBuses();
    set<string_view> bus_names;
    for (const auto& bus : buses)
        bus_names.insert(bus.first);
    size_t bus_num = 0;
    for (const auto& bus_name : bus_names) {
        const auto* bus = manager.GetBus(string(bus_name));
        const auto& stops = bus->GetStops();
        const auto& stop_coord = stops_coodinates.at(stops.front());
        svg.Add(Svg::Text{}
            .SetPoint({
                    stop_coord.longitude,
                    stop_coord.latitude
                })
            .SetOffset(properties.bus_label_offset)
            .SetFontSize(properties.bus_label_font_size)
            .SetFontFamily("Verdana")
            .SetData(string(bus_name))
            .SetFillColor(properties.underlayer_color)
            .SetStrokeColor(properties.underlayer_color)
            .SetStrokeWidth(properties.underlayer_width)
            .SetStrokeLineCap("round")
            .SetFontWeight("bold")
            .SetStrokeLineJoin("round"));
        svg.Add(Svg::Text{}
            .SetPoint({
                    stop_coord.longitude,
                    stop_coord.latitude
                })
            .SetOffset(properties.bus_label_offset)
            .SetFontSize(properties.bus_label_font_size)
            .SetFontFamily("Verdana")
            .SetFontWeight("bold")
            .SetData(string(bus_name))
            .SetFillColor(properties.color_palette.at(bus_num % (properties.color_palette.size()))));
        if (bus->IsReversed() && (stops.front() != stops.back())) {
            const auto& stop_coord = stops_coodinates.at(stops.back());
            svg.Add(Svg::Text{}
                .SetPoint({
                    stop_coord.longitude,
                    stop_coord.latitude
                    })
                .SetOffset(properties.bus_label_offset)
                .SetFontSize(properties.bus_label_font_size)
                .SetFontFamily("Verdana")
                .SetData(string(bus_name))
                .SetFillColor(properties.underlayer_color)
                .SetStrokeColor(properties.underlayer_color)
                .SetStrokeWidth(properties.underlayer_width)
                .SetFontWeight("bold")
                .SetStrokeLineCap("round")
                .SetStrokeLineJoin("round"));
            svg.Add(Svg::Text{}
                .SetPoint({
                    stop_coord.longitude,
                    stop_coord.latitude
                    })
                .SetOffset(properties.bus_label_offset)
                .SetFontSize(properties.bus_label_font_size)
                .SetFontFamily("Verdana")
                .SetFontWeight("bold")
                .SetData(string(bus_name))
                .SetFillColor(properties.color_palette.at(bus_num % (properties.color_palette.size()))));
        }
        bus_num++;
    }
}

void Map::Map::AddStops() {
    const auto& stops = manager.GetStops();
    set<string_view> stop_names;
    for (const auto& stop : stops)
        stop_names.insert(stop.first);
    for (const auto& stop_name : stop_names) {
        const auto& stop_coord = stops_coodinates.at(stop_name);
        svg.Add(Svg::Circle{}
            .SetCenter({
                    stop_coord.longitude,
                    stop_coord.latitude
            })
            .SetRadius(properties.stop_radius)
            .SetFillColor("white"));
    }
}

void Map::Map::AddNames() {
    const auto& stops = manager.GetStops();
    set<string_view> stop_names;
    for (const auto& stop : stops)
        stop_names.insert(stop.first);
    for (const auto& stop_name : stop_names) {
        const auto& stop_coord = stops_coodinates.at(stop_name);
        svg.Add(Svg::Text{}
            .SetPoint({
                    stop_coord.longitude,
                    stop_coord.latitude
                })
            .SetOffset(properties.stop_label_offset)
            .SetFontSize(properties.stop_label_font_size)
            .SetFontFamily("Verdana")
            .SetData(string(stop_name))
            .SetFillColor(properties.underlayer_color)
            .SetStrokeColor(properties.underlayer_color)
            .SetStrokeWidth(properties.underlayer_width)
            .SetStrokeLineCap("round")
            .SetStrokeLineJoin("round"));
        svg.Add(Svg::Text{}
            .SetPoint({
                    stop_coord.longitude,
                    stop_coord.latitude
                })
            .SetOffset(properties.stop_label_offset)
            .SetFontSize(properties.stop_label_font_size)
            .SetFontFamily("Verdana")
            .SetData(string(stop_name))
            .SetFillColor("black"));
    }
}

template<typename T>
bool FindNearby(const vector<T>& container, const T& first, const T& second) {
    for (size_t i = 1; i < container.size(); ++i) {
        if (set<string>({ first, second }) == set<string>({ container[i], container[i - 1] }))
            return true;
    }
    return false;
}

bool Map::Map::IsNearby(const vector<StopPosition>& coordinates, const list<size_t>& idx_range) const {
    const auto& buses = manager.GetBuses();
    for (size_t cur : idx_range) {
        for (size_t cmp : idx_range) {
            if (cur == cmp) continue;
            string current =  string(coordinates[cur].name);
            string compare = string(coordinates[cmp].name);
            for (const auto& [bus_name, bus] : buses) {
                if (bus->Find(current) && bus->Find(compare)) {
                    const auto& route = bus->GetStops();
                    if (FindNearby(route, current, compare)) return true;
                }
            }
        }
    }
    return false;
}

vector<list<size_t>> Map::Map::Paginator(vector<StopPosition> coordinates) const {
    vector<list<size_t>> paginated_ranges;
    size_t idx = 0;
    list<size_t> idx_range(1, 0);
    for (size_t i = 1; i < coordinates.size(); i++) {
        idx_range.push_back(i);
        if (IsNearby(coordinates, idx_range)) {
            idx_range.pop_back();
            paginated_ranges.push_back(idx_range);
            idx_range.clear();
            idx_range.push_back(i);
            idx++;
        }
    }
    if(idx_range.size())
        paginated_ranges.push_back(idx_range);
    return paginated_ranges;
}

#define COMPRESS_COORDINATES(axis) {                                                                            \
    sort(coordinates.begin(), coordinates.end(), [](const StopPosition& lhs, const StopPosition& rhs) {         \
        return lhs.coordinate.axis < rhs.coordinate.axis;                                                       \
    });                                                                                                         \
    __ranges__ = Paginator(coordinates);                                                                        \
    for (size_t idx = 0; idx < __ranges__.size(); idx++)                                                        \
        for (size_t position : __ranges__[idx])                                                                 \
            coordinates[position].idx.axis = idx;                                                               \
}

void Map::Map::ComputeStopsCoordinates() {
    const auto& stops = manager.GetStops();
    vector<StopPosition> coordinates;
    coordinates.reserve(stops.size());
    for (const auto& stop : stops)
        coordinates.push_back({ stop.first, stop.second->GetCoordinate() });

    if (!coordinates.size()) return;
    if (coordinates.size() == 1) {
        stops_coodinates[coordinates.front().name].longitude = properties.padding;
        stops_coodinates[coordinates.front().name].latitude = properties.height - properties.padding;
        return;
    }

    vector<list<size_t>> __ranges__;
    COMPRESS_COORDINATES(longitude);
    double x_step = 0;
    if (__ranges__.size() == 1) x_step = 0;
    else x_step = (properties.width - 2 * properties.padding) / (__ranges__.size() - 1);
    for (auto& coordinate : coordinates)
        coordinate.coordinate.longitude = properties.padding + x_step * coordinate.idx.longitude;

    COMPRESS_COORDINATES(latitude);
    double y_step = 0;
    if (__ranges__.size() == 1) x_step = 0;
    else y_step = (properties.height - 2 * properties.padding) / (__ranges__.size() - 1);
    for (auto& coordinate : coordinates)
        coordinate.coordinate.latitude = properties.height - properties.padding - y_step * coordinate.idx.latitude;

    for (const auto& coordinate : coordinates)
        stops_coodinates[coordinate.name] = coordinate.coordinate;
}

void Map::Map::RenderMap() {
    for (auto layer : properties.layers) {
        switch (layer) {
        case LayerType::BUS_LABELS: {
            AddBusNames();
            break;
        }
        case LayerType::BUS_LINES: {
            AddRounds();
            break;
        }
        case LayerType::STOP_LABELS: {
            AddNames();
            break;
        }
        case LayerType::STOP_POINTS: {
            AddStops();
            break;
        }
        default: break;
        }
    }
    stringstream out;
    svg.Render(out);
    stringstream result;
    result << quoted(out.str());
    map = result.str();
}
