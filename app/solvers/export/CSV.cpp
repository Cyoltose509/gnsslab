
#include <cmath>
#include <ostream>

#include "Core.h"
#include "solvers/common/Task.h"
#include "util/CSV.h"

namespace Solvers::Export {
    namespace {
        const char *statText(const int stat) {
            return stat == 1 ? "Fixed" : stat == 2 ? "Float" : "Invalid";
        }

        void numCell(CSV::Writer &w, const double v, const Kind kind, const int prec) {
            switch (kind) {
                case Kind::Int: w.cell(static_cast<long>(std::llround(v)));
                    break;
                case Kind::Fixed: w.cell(CSV::fx(v, prec));
                    break;
                default: w.cell(CSV::f(v, prec));
                    break;
            }
        }
    }

    void writeCSV(std::ostream &out, const Task &task, const Spec &spec, const std::vector<std::size_t> &idx) {
        CSV::Writer::writeBom(out);
        CSV::Writer w(out);
        task.writeCSVMeta(w);

        const std::vector<Col> cols = resolveColumns(spec, Format::CSV, task);
        for (const Col &c: cols) w.cell(c.name);
        w.endRow();

        Row row;
        for (const std::size_t i: idx) {
            task.exportRow(i, row);
            for (const Col &c: cols) {
                const FieldDef &d = fieldDef(c.field);
                if (d.kind == Kind::Text) {
                    w.cell(c.field == Field::Stat ? statText(row.stat) : std::string());
                    continue;
                }
                const double v = row.get(c.field);
                if (!std::isfinite(v)) {
                    w.cell(""); // 本行没有这个量：留空格
                    continue;
                }
                numCell(w, v, d.kind, d.prec);
            }
            w.endRow();
        }
    }
}
