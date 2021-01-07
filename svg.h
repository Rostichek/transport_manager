#pragma once
#include <ostream>
#include <optional>
#include <vector>
#include <variant>
#include <memory>
#include <string>
#include <sstream>
#include <utility>
#include <iostream>

using namespace std;

namespace Svg {

	struct Point {
		double	x = 0,
			y = 0;

	public:
		Point() = default;
		Point(double x, double y) : x(x), y(y) {}
	};

	struct Rgb {
		size_t	red = 0,
			green = 0,
			blue = 0;
		optional<double> alpha = nullopt;

	public:
		Rgb() = default;
		Rgb(size_t red, size_t green, size_t blue) : red(red), green(green), blue(blue) {}
	};

	class Color {
	public:
		Color() : color("none") {}
		Color(const string& color) : color(color) {}
		Color(const char* color) : color(string(color)) {}
		Color(const Rgb& color) : color(color) {}

		Color* operator= (const Color& oth) {
			if (&oth != this) {
				color = oth.color;
			}

			return this;
		}

		const auto* GetColor() const { return &color; }
	private:
		variant<string, Rgb> color;
	};


	static const Color NoneColor = Color();

	enum class Type {
		CIRCLE,
		TEXT,
		POLYLINE,
	};

	template<typename D>
	class Object {
	public:
		const Type type;

		Object(Type type) : type(type) {}

		D& SetFillColor(const Color& color = NoneColor) {
			properties.fill = color;
			return*static_cast<D*>(this);
		}

		D& SetStrokeColor(const Color& color = NoneColor) {
			properties.stroke = color;
			return*static_cast<D*>(this);

		}

		D& SetStrokeWidth(double width = 1.0) {
			properties.stroke_width = width;
			return*static_cast<D*>(this);
		}

		D& SetStrokeLineCap(const string& linecap) {
			properties.stroke_linecap = linecap;
			return*static_cast<D*>(this);
		}

		D& SetStrokeLineJoin(const string& linejoin) {
			properties.stroke_linejoin = linejoin;
			return*static_cast<D*>(this);
		}

		string PrintObjectProperties() const {
			stringstream out;
			out << " fill=" << "\"" << properties.fill << "\"";
			out << " stroke=" << "\"" << properties.stroke << "\"";
			out << " stroke-width=" << "\"" << properties.stroke_width << "\"";
			if (properties.stroke_linecap.has_value())
				out << " stroke-linecap=" << "\"" << properties.stroke_linecap.value() << "\"";
			if (properties.stroke_linejoin.has_value())
				out << " stroke-linejoin=" << "\"" << properties.stroke_linejoin.value() << "\"";
			return out.str();
		}

	private:
		struct Properties {
			Color fill = NoneColor;
			Color stroke = NoneColor;
			double stroke_width = 1.0;
			optional<string> stroke_linecap;
			optional<string> stroke_linejoin;
		} properties;
	};

	class Circle : public Object<Circle> {
	public:
		Circle() : Object<Circle>(Type::CIRCLE) {}

		Circle& SetCenter(Point center) {
			center_ = center;
			return *this;
		}

		Circle& SetRadius(double radius) {
			radius_ = radius;
			return *this;
		}

		string PrintCircleProperties() const;

	private:
		Point center_ = { 0,0 };
		double radius_ = 1.0;
	};

	class Polyline : public Object<Polyline> {
	public:
		Polyline() : Object<Polyline>(Type::POLYLINE) {}

		string PrintPolylineProperties() const;

		Polyline& AddPoint(Point point) {
			points_.push_back(point);
			return *this;
		}
	private:
		vector<Point> points_;
	};

	class Text : public Object<Text> {
	public:
		Text() : Object<Text>(Type::TEXT) {}

		Text& SetPoint(Point coordinate) {
			coord_ = coordinate;
			return *this;
		}

		Text& SetOffset(Point offset) {
			offset_ = offset;
			return *this;
		}

		Text& SetFontSize(uint32_t size) {
			font_size_ = size;
			return *this;
		}

		Text& SetFontFamily(const string& font) {
			font_family_ = font;
			return *this;
		}

		Text& SetData(const string& text) {
			text_ = text;
			return *this;
		}

		Text& SetFontWeight(const string& weight) {
			font_weight_ = weight;
			return *this;
		}

		string PrintTextProperties() const;
	private:
		Point coord_ = { 0,0 };
		Point offset_ = { 0,0 };
		uint32_t font_size_ = 1;
		optional<string> font_family_;
		optional<string> font_weight_;
		string text_;
	};

	struct ObjectHolder {
		Type type;
		unique_ptr<variant<Circle, Polyline, Text>> object;
	};

	class Document {
	public:
		Document() = default;

		void Add(const Circle& object) {
			objects_pool.push_back({ object.type, make_unique<variant<Circle, Polyline, Text>>(object) });
		}

		void Add(const Text& object) {
			objects_pool.push_back({ object.type, make_unique<variant<Circle, Polyline, Text>>(object) });
		}

		void Add(const Polyline& object) {
			objects_pool.push_back({ object.type, make_unique<variant<Circle, Polyline, Text>>(object) });
		}

		void Render(std::ostream& out) const;
	private:
		vector<ObjectHolder> objects_pool;
	};
}
