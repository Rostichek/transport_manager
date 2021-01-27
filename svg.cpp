#include "svg.h"
#include <sstream>

namespace Svg {
	string ComposeProperty(const string& name, const string& value) {
		return  " " + name + "=\"" + value + "\"";
	}

	string ComposeProperty(const string& name, const double value) {
		stringstream out;
		out << " " << name << "=\"" << value << "\"";
		return out.str();
	}

	string ComposeProperty(const string& name, uint32_t value) {
		return  " " + name + "=\"" + to_string(value) + "\"";
	}

	string Circle::PrintCircleProperties() const {
		stringstream out;
		out << ComposeProperty("cx", center_.x)
			<< ComposeProperty("cy", center_.y)
			<< ComposeProperty("r", radius_);
		return out.str();
	}

	string Text::PrintTextProperties() const {
		stringstream out;
		out << ComposeProperty("x", coord_.x)
			<< ComposeProperty("y", coord_.y)
			<< ComposeProperty("dx", offset_.x)
			<< ComposeProperty("dy", offset_.y)
			<< ComposeProperty("font-size", font_size_);
		if (font_family_.has_value())
			out << ComposeProperty("font-family", font_family_.value());
		if (font_weight_.has_value())
			out << ComposeProperty("font-weight", font_weight_.value());
		out << " >";
		out << text_;
		return out.str();
	}

	string Polyline::PrintPolylineProperties() const {
		stringstream out;
		out << " points=" << "\"";
		for (const auto& point : points_) {
			out << point.x << "," << point.y << " ";
		}
		out << "\"";
		return out.str();
	}

	ostream& operator<<(ostream& out, const Color& color) {
		if (auto val = get_if<string>(color.GetColor())) {
			out << *val;
			return out;
		}
		if (auto val = get_if<Rgb>(color.GetColor())) {
			if (val->alpha.has_value())
				out << "rgba(" << val->red << "," << val->green << "," << val->blue << "," << val->alpha.value() << ")";
			else
				out << "rgb(" << val->red << "," << val->green << "," << val->blue << ")";
			return out;
		}
		return out;
	}

	ostream& operator<<(ostream& out, const Circle& circle) {
		out << "<circle ";
		out << circle.PrintObjectProperties();
		out << circle.PrintCircleProperties();
		out << "/>";
		return out;
	}

	ostream& operator<<(ostream& out, const Polyline& polyline) {
		out << "<polyline ";
		out << polyline.PrintObjectProperties();
		out << polyline.PrintPolylineProperties();
		out << "/>";
		return out;
	}

	ostream& operator<<(ostream& out, const Text& text) {
		out << "<text ";
		out << text.PrintObjectProperties();
		out << text.PrintTextProperties();
		out << "</text>";
		return out;

	}

	void Document::Render(std::ostream& out) const {
		out << "<?xml version=\"1.0\" encoding=\"UTF-8\" ?> ";
		out << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\"> ";
		for (const auto& object : objects_pool) {
			switch (object.type) {
			case Type::CIRCLE: {
				out << get<Circle>(*object.object);
				break;
			}
			case Type::POLYLINE: {
				out << get<Polyline>(*object.object);
				break;
			}
			case Type::TEXT: {
				out << get<Text>(*object.object);
				break;
			}
			default: {
				break;
			}
			}
		}
		out << " </svg>";
	}
}
