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
class Stop;
class Bus;

struct Coordinate {
    double 
        latitude = 0.0,
        longitude = 0.0;
};

double ConvertDegToRad(double degree) {
    return (degree * M_PI) / 180.0;
}

double ComputeDistance(const Coordinate& lhs, const Coordinate& rhs) {
    return acos(sin(ConvertDegToRad(lhs.latitude)) * sin(ConvertDegToRad(rhs.latitude)) +
        cos(ConvertDegToRad(lhs.latitude)) * cos(ConvertDegToRad(rhs.latitude)) *
        cos(fabs(ConvertDegToRad(lhs.longitude) - ConvertDegToRad(rhs.longitude)))
    ) * 6371000;
}

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

class TransportManager {
public:
    TransportManager() = default;

    void AddStop(string stop_name, Coordinate coordinate) {
        stops_[stop_name] = make_unique<Stop>(stop_name, coordinate);
    }

    void AddBus(string bus_name, vector<string> stops, bool is_reversed) {
        buses_[bus_name] = make_unique<Bus>(bus_name, move(stops), is_reversed);
    }

    const Bus* GetBus(const string& bus_name) const {
        if (buses_.count(bus_name))
            return buses_.at(bus_name).get();
        return nullptr;
    }

    const Stop* GetStop(const string& stop_name) const {
        if (stops_.count(stop_name))
            return stops_.at(stop_name).get();
        return nullptr;
    }

private:
    unordered_map<string, unique_ptr<Stop>> stops_;
    unordered_map<string, unique_ptr<Bus>> buses_;
};

class Stop : public TransportManager {
public:
    Stop(const string& name, Coordinate coordinate) : name_(name), coordinate_(coordinate) {}

    string_view GetName() {
        return string_view(name_);
    }

    const Coordinate& GetCoordinate() const {
        return coordinate_;
    }

private:
    string name_;
    Coordinate coordinate_;
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

    double GetLength(const TransportManager& manager) const {
        double length = 0.0;
        for (size_t i = 0; i < stops_.size() - 1; ++i) {
            length += ComputeDistance(manager.GetStop(stops_[i])->GetCoordinate(), manager.GetStop(stops_[i + 1])->GetCoordinate());
        }
            if (is_reversed_) length *= 2;
            return length;
    }

private:
    string name_;
    vector<string> stops_;
    bool is_reversed_;
};

struct Request;
using RequestHolder = unique_ptr<Request>;

struct Request {
    enum class Type {
        ADD_STOP,
        ADD_BUS,
        BUS_INFO,
    };

    Request(Type type) : type(type) {}
    static RequestHolder Create(Type type);
    virtual void ParseFrom(string_view input) = 0;
    virtual ~Request() = default;

    const Type type;
};

const unordered_map<string_view, Request::Type> STR_TO_INPUT_REQUEST_TYPE = {
    {"Stop", Request::Type::ADD_STOP},
    {"Bus", Request::Type::ADD_BUS},
};

const unordered_map<string_view, Request::Type> STR_TO_OUTPUT_REQUEST_TYPE = {
    {"Bus", Request::Type::BUS_INFO},
};

template <typename ResultType>
struct ReadRequest : Request {
    using Request::Request;
    virtual ResultType Process(const TransportManager& manager) const = 0;
};

struct ModifyRequest : Request {
    using Request::Request;
    virtual void Process(TransportManager& manager) const = 0;
};

struct AddStopRequest : ModifyRequest {
    AddStopRequest() : ModifyRequest(Type::ADD_STOP) {}
    void ParseFrom(string_view input) override {
        name = ReadToken(input, ":");
        coordinate.latitude = ConvertToDouble(ReadToken(input, ","));
        coordinate.longitude = ConvertToDouble(input);
    }
    
    void Process(TransportManager& manager) const override {
        manager.AddStop(name, coordinate);
    }
private:
    string name;
    Coordinate coordinate;
};

struct AddBusRequest : ModifyRequest {
    AddBusRequest() : ModifyRequest(Type::ADD_BUS) {}
    void ParseFrom(string_view input) override {
        name = ReadToken(input, ":");
        ReadToken(input, " ");
        auto description = input;
        while (!input.empty()) {
            stops.push_back(string(ReadToken(input, " > ")));
        }
        if (stops.size() == 1) {
            stops.clear();
            while (!description.empty())
                stops.push_back(string(ReadToken(description, " - ")));
            is_reversed = true;
        }
    }

    void Process(TransportManager& manager) const override {
        manager.AddBus(name, stops, is_reversed);
    }
private:
    string name;
    vector<string> stops;
    bool is_reversed = false;
};

struct BusInfoRequest : ReadRequest<string> {
    BusInfoRequest() : ReadRequest<string>(Type::BUS_INFO) {}
    void ParseFrom(string_view input) override {
        name = ReadToken(input, ":");
    }

    string Process(const TransportManager& manager) const override {
        const auto& bus = manager.GetBus(name);
        if (bus == nullptr) return "Bus " + name + ": not found";
        stringstream output; 
        output << "Bus " << name << ": " << bus->GetStopsNum() << " stops on route, " <<
            bus->GetUniqueStopsNum() << " unique stops, " << fixed << setprecision(6) << bus->GetLength(manager) << " route length";
        return  output.str();
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

RequestHolder ParseInputRequest(string_view request_str) {
    const auto request_type = ConvertRequestTypeFromString(ReadToken(request_str), STR_TO_INPUT_REQUEST_TYPE);
    if (!request_type) {
        return nullptr;
    }
    RequestHolder request = Request::Create(*request_type);
    if (request) {
        request->ParseFrom(request_str);
    };
    return request;
}

RequestHolder ParseOutputRequest(string_view request_str) {
    const auto request_type = ConvertRequestTypeFromString(ReadToken(request_str), STR_TO_OUTPUT_REQUEST_TYPE);
    if (!request_type) {
        return nullptr;
    }
    RequestHolder request = Request::Create(*request_type);
    if (request) {
        request->ParseFrom(request_str);
    };
    return request;
}

vector<RequestHolder> ReadRequests(const function<RequestHolder(string_view)>& ParseRequest, istream & in_stream = cin) {
    const size_t request_count = ReadNumberOnLine<size_t>(in_stream);

    vector<RequestHolder> requests;
    requests.reserve(request_count);

    for (size_t i = 0; i < request_count; ++i) {
        string request_str;
        getline(in_stream, request_str);
        if (auto request = ParseRequest(request_str)) {
            requests.push_back(move(request));
        }
    }
    return requests;
}

vector<string> ProcessRequests(const vector<RequestHolder> & requests, TransportManager& manager) {
    vector<string> responses;
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
    }
    return responses;
}

void PrintResponses(const vector<string> & responses, ostream & stream = cout) {
    for (const string& response : responses) {
        stream << response << endl;
    }
}


int main() {
    TransportManager manager;
    ProcessRequests(ReadRequests(ParseInputRequest), manager);
    PrintResponses(ProcessRequests(ReadRequests(ParseOutputRequest), manager));
    return 0;
}
