
#include <vector>

#include "Core.h"
#include "product/sp3/Writer.h"
#include "solvers/common/Task.h"

namespace Solvers::Export {
    void writeSP3(std::ostream &out, const Task &task, const Spec &spec, const std::vector<std::size_t> &idx) {
        std::vector<std::size_t> rows;
        {
            Row probe;
            for (const std::size_t i: idx) {
                task.exportRow(i, probe);
                if (const Eigen::Vector3d p(probe.get(Field::X), probe.get(Field::Y), probe.get(Field::Z)); probe.solved && p.allFinite()) rows.push_back(i);
            }
        }

        SP3::Writer::Header head;
        head.satId = spec.satId;
        head.marker = spec.marker;
        head.frame = task.hasOrbitFrame ? task.orbitFrame : Frame::Ref::IGS14;
        head.interval = spec.sampling.seconds > 0.0 ? spec.sampling.seconds : 30.0;
        if (!rows.empty()) {
            const Stamp s0 = task.exportStamp(rows.front());
            head.epoch = toCivil(s0);
            head.week = s0.week;
            head.sow = s0.tow;
            if (spec.sampling.mode != Sampling::Mode::Seconds && rows.size() > 1) {
                if (const double dt = task.exportStamp(rows[1]).abs() - s0.abs(); dt > 0.0) head.interval = std::round(dt * 1000.0) / 1000.0;
            }
        }

        if (spec.sp3Velocity) {
            Row probe;
            for (const std::size_t i: rows) {
                task.exportRow(i, probe);
                if (const Eigen::Vector3d v(probe.get(Field::Vx), probe.get(Field::Vy), probe.get(Field::Vz)); v.allFinite() && v.squaredNorm() > 0.0) {
                    head.hasVel = true;
                    break;
                }
            }
        }

        std::vector<SP3::Writer::Record> records;
        records.reserve(rows.size());
        Row row;
        for (const std::size_t i: rows) {
            task.exportRow(i, row);
            const double x = row.get(Field::X);
            const double y = row.get(Field::Y);
            const double z = row.get(Field::Z);
            SP3::Writer::Record r;
            r.epoch = toCivil(row.stamp);
            r.pos = Eigen::Vector3d(x, y, z);
            if (head.hasVel) {
                const Eigen::Vector3d raw(row.get(Field::Vx), row.get(Field::Vy), row.get(Field::Vz));
                r.vel = raw.allFinite() ? raw : Eigen::Vector3d::Zero();
            }
            records.push_back(r);
        }

        SP3::Writer::write(out, head, records);
    }
}
