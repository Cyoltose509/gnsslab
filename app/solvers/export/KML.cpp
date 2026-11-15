#include <iomanip>
#include <ostream>
#include <string>
#include <vector>

#include "Core.h"
#include "solvers/common/Config.h"
#include "solvers/common/Task.h"
#include "i18n/I18n.h"
#include "core/TimeStruct.h"

namespace Solvers::Export {
    namespace {
        using F = Field;

        std::string isoUTC(const Stamp &s) {
            CommonTime ct = WeekSecond(s.week, s.tow).toCommonTime();
            ct.convertSystem(TimeSystem::UTC);
            return civilText(ct.toCivilTime(), '-', 'T', "Z");
        }

        std::string xmlEscape(const std::string &in) {
            std::string out;
            out.reserve(in.size());
            for (const char ch: in) {
                switch (ch) {
                    case '&': out += "&amp;";
                        break;
                    case '<': out += "&lt;";
                        break;
                    case '>': out += "&gt;";
                        break;
                    case '"': out += "&quot;";
                        break;
                    case '\'': out += "&apos;";
                        break;
                    default: out.push_back(ch);
                        break;
                }
            }
            return out;
        }
    }

    void writeKML(std::ostream &out, const Task &task, const std::vector<size_t> &idx) {
        struct Pt {
            std::string when;
            double lat = 0.0, lon = 0.0, h = 0.0;
        };
        std::vector<Pt> pts;
        pts.reserve(idx.size());
        {
            Row row;
            for (const std::size_t i: idx) {
                task.exportRow(i, row);
                if (!row.solved) continue;
                const double lat = row.get(F::Lat);
                const double lon = row.get(F::Lon);
                const double h = row.get(F::H);
                if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(h)) continue;
                pts.push_back({isoUTC(row.stamp), lat, lon, h});
            }
        }

        const std::string title = task.fileName.empty() ? std::string("gnssLab") : task.fileName;
        out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        out << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
        out << "  <Document>\n";
        out << "    <name>" << xmlEscape(title) << "</name>\n";
        out << "    <description>" << I18n::tr("export.kml.desc_head", modeLabel(task.mode), pts.size());
        if (pts.size() >= 2) out << I18n::tr("export.kml.desc_range", pts.front().when, pts.back().when);
        out << "</description>\n";
        out << "    <Style id=\"traj\">\n"
                "      <LineStyle><color>ff2a2aff</color><width>2</width></LineStyle>\n"
                "      <IconStyle><scale>0.4</scale></IconStyle>\n"
                "    </Style>\n";
        if (pts.size() >= 2) {
            out << "    <Placemark>\n";
            out << "      <name>" << I18n::trc("Trajectory") << "</name>\n";
            out << "      <styleUrl>#traj</styleUrl>\n";
            out << "      <LineString>\n";
            out << "        <tessellate>1</tessellate>\n";
            out << "        <altitudeMode>absolute</altitudeMode>\n";
            out << "        <coordinates>\n";
            for (const Pt &p: pts) {
                out << "          " << std::fixed << std::setprecision(8) << p.lon << ',' << p.lat << ','
                        << std::setprecision(3) << p.h << '\n';
            }
            out << "        </coordinates>\n";
            out << "      </LineString>\n";
            out << "    </Placemark>\n";
        } else if (pts.size() == 1) {
            out << "    <Placemark>\n";
            out << "      <name>" << I18n::trc("Position") << "</name>\n";
            out << "      <styleUrl>#traj</styleUrl>\n";
            out << "      <Point>\n";
            out << "        <altitudeMode>absolute</altitudeMode>\n";
            out << "        <coordinates>" << std::fixed << std::setprecision(8) << pts[0].lon << ','
                    << pts[0].lat << ',' << std::setprecision(3) << pts[0].h << "</coordinates>\n";
            out << "      </Point>\n";
            out << "    </Placemark>\n";
        } else {
            out << "    <!-- " << I18n::trc("export.kml.no_epochs") << " -->\n";
        }
        out << "  </Document>\n";
        out << "</kml>\n";
    }
}
