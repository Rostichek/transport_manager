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
    }
    else {
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

bool Bus::IsEnding(string_view stop) const {
    if (stop == stops_.front() || (is_reversed_ && stop == stops_.back())) return true;
    return false;
}


int Bus::GetLength(const TransportManager& manager) const {
    int length = 0;
    for (size_t i = 0; i < stops_.size() - 1; ++i) {
        int distance_between_stops = 0;
        try {
            distance_between_stops = manager.GetStop(stops_[i])->GetDistance(stops_[i + 1]);
        }
        catch (...) {
            distance_between_stops = manager.GetStop(stops_[i + 1])->GetDistance(stops_[i]);
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
            edge.stops_list.clear();
            edge.stop_count = 0;
            for (size_t i = j; i < stops.size() - 1; ++i) {
                int distance_between_stops = 0;
                try {
                    distance_between_stops = GetStop(stops[i])->GetDistance(stops[i + 1]);
                }
                catch (...) {
                    distance_between_stops = GetStop(stops[i + 1])->GetDistance(stops[i]);
                }
                edge.stops_list.push_back({ stops[i] , stops[i + 1] });
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
                edge.stops_list.clear();
                for (size_t i = j; i > 0; --i) {
                    int distance_between_stops = 0;
                    try {
                        distance_between_stops = GetStop(stops[i])->GetDistance(stops[i - 1]);
                    }
                    catch (...) {
                        distance_between_stops = GetStop(stops[i - 1])->GetDistance(stops[i]);
                    }
                    edge.stops_list.push_back({ stops[i] , stops[i - 1] });
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

pair<string, vector<Graph::Edge<double>>> TransportManager::GetRoute(const string& from, const string& to) const {
    vector<Graph::Edge<double>> items;
    const auto& info = router->BuildRoute(stops_.at(from)->GetIndx().first, stops_.at(to)->GetIndx().first);
    if (!info.has_value())
        return {};
    for (int i = 0; i < info->edge_count; ++i)
        items.push_back(graph_->GetEdge(router->GetRouteEdge(info->id, i)));
    //router->ReleaseRoute(info->id);
    return { reoute_renderer->RenderRoute(items), items };
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
    properties.outer_margin = json_properties.at("outer_margin").AsNumber();
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

const unordered_map<string, unique_ptr<Bus>>& Map::Map::GetBuses() const { return manager.GetBuses(); }

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
        bus_colors[bus_name] = bus_num;
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

void Map::Map::ComputeNearbyStops() {
    for (const auto& [bus_name, bus] : manager.GetBuses()) {
        const auto& stops = bus->GetStops();
        for (size_t i = 1; i < stops.size(); ++i) {
            nearby_stops[stops[i]].insert(stops[i - 1]);
            nearby_stops[stops[i - 1]].insert(stops[i]);
        }
    }
}

bool Map::Map::FindNearby(string_view first, string_view second) const {
    if (nearby_stops.count(first)) {
        return (nearby_stops.count(first) && nearby_stops.at(first).count(second)) ||
            (nearby_stops.count(second) && nearby_stops.at(second).count(first));
    }
    return false;
}

optional<size_t> Map::Map::IsNearby(const vector<StopPosition>& coordinates, const vector<size_t>& indeces, size_t coordinate_num) const {
    const auto& buses = manager.GetBuses();
    optional<size_t> max_idx;
    string_view stop_name = coordinates[coordinate_num].name;
    for (size_t i = 0; i < coordinate_num; ++i) {
        string_view cmp = coordinates[i].name;
        for (const auto& [bus_name, bus] : manager.GetBuses()) {
            if (bus->Find(cmp)
                && bus->Find(stop_name)) {
                if (FindNearby(stop_name, cmp)) {
                    if (max_idx.has_value()) {
                        max_idx = max(max_idx.value(), indeces[i]);
                    }
                    else max_idx = indeces[i];
                }
            }
        }
    }
    return max_idx;
}

vector<list<size_t>> Map::Map::Paginator(vector<StopPosition> coordinates) const {
    vector<size_t> indeces(coordinates.size());
    indeces.front() = 0;
    for (size_t i = 1; i < coordinates.size(); ++i) {
        auto is_nearby = IsNearby(coordinates,
            indeces, i);
        if (is_nearby.has_value())
            indeces[i] = is_nearby.value() + 1;
        else indeces[i] = 0;
    }
    vector<list<size_t>> paginated_ranges(indeces.size());
    for (size_t i = 0; i < indeces.size(); ++i)
        paginated_ranges[indeces[i]].push_back(i);
    return vector<list<size_t>>(paginated_ranges.begin(),
        find(paginated_ranges.begin(), paginated_ranges.end(), list<size_t>{}));
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

void Map::Map::FindBaseStops(vector<StopPosition>& coordinates) const {
    for (auto& coordinate : coordinates) {
        size_t buses_counter = 0;
        string_view stop_name = coordinate.name;
        for (const auto& [bus_name, bus] : manager.GetBuses()) {
            if (bus->Find(stop_name)) {
                size_t counter = 0;
                for (const auto& stop : bus->GetStops())
                    if (stop == stop_name) counter++;
                if (bus->IsReversed() && counter > 1) coordinate.is_base = true;
                if (!bus->IsReversed() && counter > 2) coordinate.is_base = true;
                buses_counter++;
                auto& route = bus->GetStops();
                if (route.front() == stop_name) coordinate.is_base = true;
                if (bus->IsReversed() && (route.front() != route.back()) && route.back() == stop_name) coordinate.is_base = true;
            }
        }
        if (buses_counter != 1) coordinate.is_base = true;
    }
}

void Map::Map::Interpolation(vector<StopPosition>& coordinates) const {
    FindBaseStops(coordinates);
    unordered_map<string_view, StopPosition*> coordinates_map;
    for (auto& coordinate : coordinates)
        coordinates_map[coordinate.name] = &coordinate;

    for (const auto& [bus_name, bus] : manager.GetBuses()) {
        auto& stops = bus->GetStops();
        size_t i = 0, j = 0;
        for (const auto& stop : stops) {
            if (coordinates_map[stop]->is_base && i != j) {
                double lon_step = (coordinates_map[stops[j]]->coordinate.longitude - coordinates_map[stops[i]]->coordinate.longitude) / (j - i);
                for (size_t k = i + 1; k < j; ++k) {
                    coordinates_map[stops[k]]->coordinate.longitude = coordinates_map[stops[i]]->coordinate.longitude + lon_step * (k - i);
                }
                double lat_step = (coordinates_map[stops[j]]->coordinate.latitude - coordinates_map[stops[i]]->coordinate.latitude) / (j - i);
                for (size_t k = i + 1; k < j; ++k) {
                    coordinates_map[stops[k]]->coordinate.latitude = coordinates_map[stops[i]]->coordinate.latitude + lat_step * (k - i);
                }
                i = j;
            }
            j++;
        }
    }
}

void Map::Map::ComputeStopsCoordinates() {
    ComputeNearbyStops();
    const auto& stops = manager.GetStops();
    vector<StopPosition> coordinates;
    coordinates.reserve(stops.size());
    for (const auto& stop : stops)
        coordinates.push_back({ stop.first, stop.second->GetCoordinate() });
    Interpolation(coordinates);
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
    if (is_rendered) return;
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
    is_rendered = true;
}

namespace Map {
    RouteRenderer::RouteRenderer(shared_ptr<Map> map) : map(map) {
        map->RenderMap();
        const auto& properties = map->GetProperties();
        base_svg = map->GetSvgMap();
        base_svg.Add(Svg::Rectangle{}
            .SetFirstPoint({
                -properties.outer_margin,
                -properties.outer_margin
                })
            .SetSecondPoint({
                properties.width + properties.outer_margin,
                properties.height + properties.outer_margin
                })
            .SetFillColor(properties.underlayer_color)
        );
        base_size = base_svg.Size();
    }

    string RouteRenderer::RenderRoute(const vector<Graph::Edge<double>>& items) {
        for (auto layer : map->GetProperties().layers) {
            switch (layer) {
            case LayerType::BUS_LABELS: {
                AddBusNames(items);
                break;
            }
            case LayerType::BUS_LINES: {
                AddRounds(items);
                break;
            }
            case LayerType::STOP_LABELS: {
                AddNames(items);
                break;
            }
            case LayerType::STOP_POINTS: {
                AddStops(items);
                break;
            }
            default: break;
            }
        }
        stringstream out;
        base_svg.Render(out);
        base_svg.Remove(base_size);
        stringstream result;
        result << quoted(out.str());
        return result.str();
    }

    void RouteRenderer::AddRounds(const vector<Graph::Edge<double>>& items) {
        const auto& properties = map->GetProperties();
        const auto& bus_colors = map->GetColors();
        const auto& stops_coordinates = map->GetCoordinates();
        for (const auto& item : items) {
            if (item.type == "Wait") continue;
            Svg::Polyline round;
            round.SetStrokeColor(properties.color_palette.at(bus_colors.at(item.text) % (properties.color_palette.size())))
                .SetStrokeWidth(properties.line_width)
                .SetStrokeLineCap("round")
                .SetStrokeLineJoin("round");
            for (const auto& line : item.stops_list) {
                const auto& stop_coord = stops_coordinates.at(line.first);
                round.AddPoint({
                        stop_coord.longitude,
                        stop_coord.latitude
                    });
            }
            const auto& stop_coord = stops_coordinates.at(item.stops_list.back().second);
            round.AddPoint({
                        stop_coord.longitude,
                        stop_coord.latitude
                });
            base_svg.Add(round);
        }
    }

    void RouteRenderer::AddBusNames(const vector<Graph::Edge<double>>& items) {
        const auto& properties = map->GetProperties();
        const auto& bus_colors = map->GetColors();
        const auto& stops_coordinates = map->GetCoordinates();
        for (const auto& item : items) {
            if (item.type == "Wait") continue;
            const auto& buses = map->GetBuses();
            vector<Coordinate> ending_stops;
            if (buses.at(item.text)->IsEnding(item.stops_list.front().first)) 
                ending_stops.push_back(stops_coordinates.at(item.stops_list.front().first));
            if (buses.at(item.text)->IsEnding(item.stops_list.back().second))
                ending_stops.push_back(stops_coordinates.at(item.stops_list.back().second));
            for (const auto& stop_coord : ending_stops) {
                base_svg.Add(Svg::Text{}
                    .SetPoint({
                            stop_coord.longitude,
                            stop_coord.latitude
                        })
                    .SetOffset(properties.bus_label_offset)
                    .SetFontSize(properties.bus_label_font_size)
                    .SetFontFamily("Verdana")
                    .SetData(item.text)
                    .SetFillColor(properties.underlayer_color)
                    .SetStrokeColor(properties.underlayer_color)
                    .SetStrokeWidth(properties.underlayer_width)
                    .SetStrokeLineCap("round")
                    .SetFontWeight("bold")
                    .SetStrokeLineJoin("round"));
                base_svg.Add(Svg::Text{}
                    .SetPoint({
                            stop_coord.longitude,
                            stop_coord.latitude
                        })
                    .SetOffset(properties.bus_label_offset)
                    .SetFontSize(properties.bus_label_font_size)
                    .SetFontFamily("Verdana")
                    .SetFontWeight("bold")
                    .SetData(item.text)
                    .SetFillColor(properties.color_palette.at(bus_colors.at(item.text) % (properties.color_palette.size()))));
            }
        }
    }

    void RouteRenderer::AddStops(const vector<Graph::Edge<double>>& items) {
        const auto& properties = map->GetProperties();
        const auto& bus_colors = map->GetColors();
        const auto& stops_coordinates = map->GetCoordinates();
        for (const auto& item : items) {
            if (item.type == "Wait") continue;
            for (const auto& line : item.stops_list) {
                const auto& stop_coord = stops_coordinates.at(line.first);
                base_svg.Add(Svg::Circle{}
                    .SetCenter({
                            stop_coord.longitude,
                            stop_coord.latitude
                        })
                    .SetRadius(properties.stop_radius)
                    .SetFillColor("white"));
            }
            const auto& stop_coord = stops_coordinates.at(item.stops_list.back().second);
            base_svg.Add(Svg::Circle{}
                .SetCenter({
                        stop_coord.longitude,
                        stop_coord.latitude
                    })
                .SetRadius(properties.stop_radius)
                .SetFillColor("white"));
        }
    }

    void RouteRenderer::AddNames(const vector<Graph::Edge<double>>& items) {
        size_t counter = 0;
        const auto& stops_coordinates = map->GetCoordinates();
        const auto& properties = map->GetProperties();
        for (const auto& item : items) {
            counter++;
            Coordinate stop_coord;
            string stop_name;
            if (item.type == "Wait") {
                stop_name = item.text;
                stop_coord = stops_coordinates.at(item.text);
            }
            else if (counter == items.size()) {
                stop_name = item.stops_list.back().second;
                stop_coord = stops_coordinates.at(item.stops_list.back().second);
            }
            else continue;
            base_svg.Add(Svg::Text{}
                .SetPoint({
                        stop_coord.longitude,
                        stop_coord.latitude
                    })
                .SetOffset(properties.stop_label_offset)
                .SetFontSize(properties.stop_label_font_size)
                .SetFontFamily("Verdana")
                .SetData(stop_name)
                .SetFillColor(properties.underlayer_color)
                .SetStrokeColor(properties.underlayer_color)
                .SetStrokeWidth(properties.underlayer_width)
                .SetStrokeLineCap("round")
                .SetStrokeLineJoin("round"));
            base_svg.Add(Svg::Text{}
                .SetPoint({
                        stop_coord.longitude,
                        stop_coord.latitude
                    })
                .SetOffset(properties.stop_label_offset)
                .SetFontSize(properties.stop_label_font_size)
                .SetFontFamily("Verdana")
                .SetData(stop_name)
                .SetFillColor("black"));
        }
    }
}