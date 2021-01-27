#include "Manager.h"
#include <iostream>
#include <fstream>
#include "Json.h"
#include "graph.h"
#include "router.h"
#include <numeric>
#include <fstream>

using namespace std;

pair<string_view, optional<string_view>> SplitTwoStrict(string_view s, string_view delimiter = " ") {
    const size_t pos = s.find(delimiter);
    if (pos == s.npos) {
        return { s, nullopt };
    }
    else {
        return { s.substr(0, pos), s.substr(pos + delimiter.length()) };
    }
}

pair<string_view, string_view> SplitTwo(string_view s, string_view delimiter = " ") {
    const auto [lhs, rhs_opt] = SplitTwoStrict(s, delimiter);
    return { lhs, rhs_opt.value_or("") };
}

string_view ReadToken(string_view & s, string_view delimiter = " ") {
    const auto [lhs, rhs] = SplitTwo(s, delimiter);
    s = rhs;
    return lhs;
}

double ConvertToDouble(string_view str) {
    // use std::from_chars when available to git rid of string copy
    size_t pos;
    const double result = stod(string(str), &pos);
    if (pos != str.length()) {
        std::stringstream error;
        error << "string " << str << " contains " << (str.length() - pos) << " trailing chars";
        throw invalid_argument(error.str());
    }
    return result;
}

int ConvertToInt(string_view str) {
    // use std::from_chars when available to git rid of string copy
    size_t pos;
    const int result = stoi(string(str), &pos);
    if (pos != str.length()) {
        std::stringstream error;
        error << "string " << str << " contains " << (str.length() - pos) << " trailing chars";
        throw invalid_argument(error.str());
    }
    return result;
}

template <typename Number>
void ValidateBounds(Number number_to_check, Number min_value, Number max_value) {
    if (number_to_check < min_value || number_to_check > max_value) {
        std::stringstream error;
        error << number_to_check << " is out of [" << min_value << ", " << max_value << "]";
        throw out_of_range(error.str());
    }
}

struct Request;
using RequestHolder = unique_ptr<Request>;

struct Request {
    enum class Type {
        ADD_STOP,
        ADD_BUS,
        BUS_INFO,
        STOP_INFO,
        ROUTE_INFO,
        MAP,
    };

    Request(Type type) : type(type) {}
    static RequestHolder Create(Type type);
    virtual void ParseFrom(Json::Node input) = 0;
    virtual ~Request() = default;

    const Type type;
};

const unordered_map<string_view, Request::Type> STR_TO_INPUT_REQUEST_TYPE = {
    {"Stop", Request::Type::ADD_STOP},
    {"Bus", Request::Type::ADD_BUS},
};

const unordered_map<string_view, Request::Type> STR_TO_OUTPUT_REQUEST_TYPE = {
    {"Bus", Request::Type::BUS_INFO},
    {"Stop", Request::Type::STOP_INFO},
    {"Route", Request::Type::ROUTE_INFO},
    {"Map", Request::Type::MAP},
};

struct Response {
    Response(const Request::Type type_) : type(type_) {}
    const Request::Type type;
    uint64_t respones_id;
    string error_message;
};

struct StopResponse : public Response {
    StopResponse() : Response(Request::Type::STOP_INFO) {}
    string name;
    set<string_view> buses_for_stop;
};

struct BusResponse : public Response {
    BusResponse() : Response(Request::Type::BUS_INFO) {}
    string name;
    size_t stops_num;
    size_t unique_stops_num;
    int real_route_length;
    double curvature;
};

struct RouteResponse : public Response {
    RouteResponse() : Response(Request::Type::ROUTE_INFO) {}
    struct Item {
        string type;
        string name;
        double time;
        size_t span_count;
    };
    vector<Item> items;
    double total_time;
};

struct MapResponse : public Response {
    MapResponse() : Response(Request::Type::MAP) {}
    string svg;
};

using ResponseHolder = unique_ptr<Response>;

template <typename ResultType>
struct ReadRequest : Request {
    using Request::Request;
    virtual ResultType Process(const TransportManager& manager) const = 0;
protected:
    uint64_t request_id;
};

struct ModifyRequest : Request {
    using Request::Request;
    virtual void Process(TransportManager& manager) const = 0;
};

struct AddStopRequest : ModifyRequest {
    AddStopRequest() : ModifyRequest(Type::ADD_STOP) {}
    
    void ParseFrom(Json::Node input) override {
        name = input.AsMap().at("name").AsString();
        coordinate = {
            input.AsMap().at("latitude").AsNumber(),
            input.AsMap().at("longitude").AsNumber()
        };
        for (const auto& stop : input.AsMap().at("road_distances").AsMap())
            distances[stop.first] = stop.second.AsNumber();
    }

    void Process(TransportManager& manager) const override {
        manager.AddStop(name, coordinate);
        for(const auto& distance_to_stop : distances)
            manager.AddDistance(name, distance_to_stop.first, distance_to_stop.second);
    }
private:
    string name;
    Coordinate coordinate;
    unordered_map<string, int> distances;
};

struct AddBusRequest : ModifyRequest {
    AddBusRequest() : ModifyRequest(Type::ADD_BUS) {}

    void ParseFrom(Json::Node input) override {
        name = input.AsMap().at("name").AsString();
        is_reversed = !input.AsMap().at("is_roundtrip").AsBool();
        for (const auto& stop : input.AsMap().at("stops").AsArray())
            stops.push_back(stop.AsString());
    }

    void Process(TransportManager& manager) const override {
        manager.AddBus(name, stops, is_reversed);
    }
private:
    string name;
    vector<string> stops;
    bool is_reversed = false;
};

struct BusInfoRequest : ReadRequest<unique_ptr<BusResponse>> {
    BusInfoRequest() : ReadRequest<unique_ptr<BusResponse>>(Type::BUS_INFO) {}

    void ParseFrom(Json::Node input) override {
        name = input.AsMap().at("name").AsString();
        request_id = input.AsMap().at("id").AsNumber();
    }

    unique_ptr<BusResponse> Process(const TransportManager& manager) const override {
        unique_ptr<BusResponse> response = make_unique<BusResponse>();
        response->name = name;
        response->respones_id = request_id;
        const auto& bus = manager.GetBus(name);
        if (bus == nullptr) {
            response->error_message = "not found";
            return move(response);
        }
        response->real_route_length = bus->GetLength(manager);
        response->curvature = response->real_route_length / bus->GetGeographicDistance(manager);
        response->stops_num = bus->GetStopsNum();
        response->unique_stops_num = bus->GetUniqueStopsNum();
        return move(response);
    }
private:
    string name;
};

struct StopInfoRequest : ReadRequest<unique_ptr<StopResponse>> {
    StopInfoRequest() : ReadRequest<unique_ptr<StopResponse>>(Type::STOP_INFO) {}

    void ParseFrom(Json::Node input) override {
        name = input.AsMap().at("name").AsString();
        request_id = input.AsMap().at("id").AsNumber();
    }

    unique_ptr<StopResponse> Process(const TransportManager& manager) const override {
        unique_ptr<StopResponse> response = make_unique<StopResponse>();
        const auto& stop = manager.GetStop(name);
        response->name = name;
        response->respones_id = request_id;
        if (stop == nullptr) {
            response->error_message = "not found";
            return move(response);
        }
        const auto& buses_from_manager = manager.GetBuses();
        set<string_view> buses_for_stop;
        for (const auto& bus : buses_from_manager)
            if (bus.second->Find(name))
                buses_for_stop.insert(bus.first);
        response->buses_for_stop = buses_for_stop;
        return move(response);
    }
private:
    string name;
};

struct RouteInfoRequest : ReadRequest<unique_ptr<RouteResponse>> {
    RouteInfoRequest() : ReadRequest<unique_ptr<RouteResponse>>(Type::ROUTE_INFO) {}

    void ParseFrom(Json::Node input) override {
        request_id = input.AsMap().at("id").AsNumber();
        from = input.AsMap().at("from").AsString();
        to = input.AsMap().at("to").AsString();;
    }

    unique_ptr<RouteResponse> Process(const TransportManager& manager) const override {
        unique_ptr<RouteResponse> response = make_unique<RouteResponse>();
        auto route_info_items = manager.GetRoute(from, to);
        response->total_time = 0;
        if (route_info_items.empty() && (from != to)) response->error_message = "not found";
        for (const auto& item : route_info_items) {
            response->total_time += item.weight;
            response->items.push_back({
                    item.type,
                    item.text,
                    item.weight,
                    item.stop_count,
                });
        }
        response->respones_id = request_id;
        return move(response);
    }
private:
    string from, to;

};

struct MapRequest : ReadRequest<unique_ptr<MapResponse>> {
    MapRequest() : ReadRequest<unique_ptr<MapResponse>>(Type::MAP) {}

    void ParseFrom(Json::Node input) override {
        request_id = input.AsMap().at("id").AsNumber();
    }

    unique_ptr<MapResponse> Process(const TransportManager& manager) const override {
        unique_ptr<MapResponse> response = make_unique<MapResponse>();
        response->svg = manager.GetMap();
        response->respones_id = request_id;
        return move(response);
        throw(out_of_range("Map did not rendered;"));
        return nullptr;
    }
private:
    string name;
};

RequestHolder Request::Create(Request::Type type) {
    switch (type) {
    case Request::Type::ADD_STOP:
        return make_unique<AddStopRequest>();
    case Request::Type::ADD_BUS:
        return make_unique<AddBusRequest>();
    case Request::Type::BUS_INFO:
        return make_unique<BusInfoRequest>();
    case Request::Type::STOP_INFO:
        return make_unique<StopInfoRequest>();
    case Request::Type::ROUTE_INFO:
        return make_unique<RouteInfoRequest>();
    case Request::Type::MAP:
        return make_unique<MapRequest>();
    default:
        return nullptr;
    }
}

template <typename Number>
Number ReadNumberOnLine(istream & stream) {
    Number number;
    stream >> number;
    string dummy;
    getline(stream, dummy);
    return number;
}

optional<Request::Type> ConvertRequestTypeFromString(string_view type_str, const unordered_map<string_view, Request::Type>& STR_TO_REQUEST_TYPE) {
    if (const auto it = STR_TO_REQUEST_TYPE.find(type_str);
        it != STR_TO_REQUEST_TYPE.end()) {
        return it->second;
    }
    else {
        return nullopt;
    }
}

RequestHolder ParseInputRequest(Json::Node request_node) {
    const auto request_type = ConvertRequestTypeFromString(request_node.AsMap().at("type").AsString(), STR_TO_INPUT_REQUEST_TYPE);
    if (!request_type) {
        return nullptr;
    }
    RequestHolder request = Request::Create(*request_type);
    if (request) {
        request->ParseFrom(request_node);
    };
    return request;
}

RequestHolder ParseOutputRequest(Json::Node request_node) {
    const auto request_type = ConvertRequestTypeFromString(request_node.AsMap().at("type").AsString(), STR_TO_OUTPUT_REQUEST_TYPE);
    if (!request_type) {
        return nullptr;
    }
    RequestHolder request = Request::Create(*request_type);
    if (request) {
        request->ParseFrom(request_node);
    };
    return request;
}

vector<RequestHolder> ReadRequests(const function<RequestHolder(Json::Node)>& ParseRequest, Json::Node in) {
    const size_t request_count = in.AsArray().size();

    vector<RequestHolder> requests;
    requests.reserve(request_count);

    for (size_t i = 0; i < request_count; ++i) {

        if (auto request = ParseRequest(in.AsArray()[i])) {
            requests.push_back(move(request));
        }
    }
    return requests;
}

vector<ResponseHolder> ProcessRequests(const vector<RequestHolder> & requests, TransportManager& manager) {
    vector<ResponseHolder> responses;
    for (const auto& request_holder : requests) {
        if (request_holder->type == Request::Type::ADD_STOP) {
            const auto& request = static_cast<const AddStopRequest&>(*request_holder);
            request.Process(manager);
        }
        else if (request_holder->type == Request::Type::ADD_BUS) {
            const auto& request = static_cast<const AddBusRequest&>(*request_holder);
            request.Process(manager);
        }
        else if (request_holder->type == Request::Type::BUS_INFO) {
            const auto& request = static_cast<const BusInfoRequest&>(*request_holder);
            responses.push_back(request.Process(manager));
        }
        else if (request_holder->type == Request::Type::STOP_INFO) {
            const auto& request = static_cast<const StopInfoRequest&>(*request_holder);
            responses.push_back(request.Process(manager));
        }
        else if (request_holder->type == Request::Type::ROUTE_INFO) {
            const auto& request = static_cast<const RouteInfoRequest&>(*request_holder);
            responses.push_back(request.Process(manager));
        }
        else if (request_holder->type == Request::Type::MAP) {
            const auto& request = static_cast<const MapRequest&>(*request_holder);
            responses.push_back(request.Process(manager));
        }
    }
    return responses;
}

void PrintResponses(const vector<ResponseHolder> & responses, ostream & stream = cout) {
    stream << "[" << endl;
    size_t response_counter = 0;
    for (const auto& response_holder : responses) {
        stream << "\t{" << endl;
        stream << "\t\t\"request_id\": " << response_holder->respones_id << "," << endl;
        if (!response_holder->error_message.empty()) {
            stream << "\t\t\"error_message\": \"" << response_holder->error_message << "\"" << endl;
        } 
        else if (response_holder->type == Request::Type::STOP_INFO) {
            const auto& response = static_cast<const StopResponse&>(*response_holder);
            stream << "\t\t\"buses\": [";
            size_t bus_counter = 0;
            for (const auto& bus : response.buses_for_stop) {
                stream << endl << "\t\t\t\"" << bus << "\"";
                bus_counter++;
                if (bus_counter != response.buses_for_stop.size())
                    stream << ",";
            }
            if (response.buses_for_stop.size()) stream << endl;
            stream << "\t\t]" << endl;
        } 
        else if (response_holder->type == Request::Type::BUS_INFO) {
            const auto& response = static_cast<const BusResponse&>(*response_holder);
            stream << "\t\t\"stop_count\": " << response.stops_num << "," << endl;
            stream << "\t\t\"unique_stop_count\": " << response.unique_stops_num << "," << endl;
            stream << "\t\t\"route_length\": " << response.real_route_length << "," << endl;
            stream << "\t\t\"curvature\": " << setprecision(16) << response.curvature << endl;
        }
        else if (response_holder->type == Request::Type::ROUTE_INFO) {
            const auto& response = static_cast<const RouteResponse&>(*response_holder);
            stream << "\t\t\"total_time\": "  << setprecision(16) << response.total_time << "," << endl;
            stream << "\t\t\"items\": [" << endl;
            int items_counter = 0;
            for (const auto& item : response.items) {
                stream << "\t\t\t{" << endl;
                stream << "\t\t\t\t\"type\": \"" << item.type << "\"," << endl;
                if (item.type == "Bus") {
                    stream << "\t\t\t\t\"bus\": \"" << item.name << "\"," << endl;
                    stream << "\t\t\t\t\"span_count\": " << item.span_count << "," << endl;
                }
                else {
                    stream << "\t\t\t\t\"stop_name\": \"" << item.name << "\"," << endl;
                }
                stream << "\t\t\t\t\"time\": " << setprecision(16) << item.time << endl;
                stream << "\t\t\t}";
                items_counter++;
                if (items_counter != response.items.size()) stream << ",";
                stream << endl;
            }
            stream << "\t\t]" << endl;
        }
        else if (response_holder->type == Request::Type::MAP) {
            const auto& response = static_cast<const MapResponse&>(*response_holder);
            stream << "\t\t\"map\": " /*<< "\""*/ << response.svg/* << "\""*/ << endl;
        }
        stream << "\t}";
        response_counter++;
        if (response_counter != responses.size())
            stream << ",";
        stream << endl;
    }
    stream << "]" << endl;
}

int main() {
    ofstream out("C:\\Users\\User\\Desktop\\Coursera\\out.txt");
    try {
        auto document = Json::Load(cin);
        const auto& settings = document.GetRoot().AsMap().at("routing_settings").AsMap();
        TransportManager manager(settings.at("bus_wait_time").AsNumber(), settings.at("bus_velocity").AsNumber());
        ProcessRequests(ReadRequests(ParseInputRequest, document.GetRoot().AsMap().at("base_requests")), manager);

        manager.BuildRouter();
        manager.BuildMap(document.GetRoot().AsMap().at("render_settings").AsMap());

        PrintResponses(ProcessRequests(ReadRequests(ParseOutputRequest, document.GetRoot().AsMap().at("stat_requests")), manager), cout);

        out.close();
    }
    catch (exception& ex) {
        cerr << ex.what() << endl;
        cerr << "Yout JSON is not have all information for build response or there are an format error;" << endl;
    }
    return 0;
}
