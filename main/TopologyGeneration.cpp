#include <iostream>
#include <vector>
using namespace std;


struct Polygon;

struct Point {
    int id;
    double x, y;

    explicit Point(const int _id = 0, const double _x = 0, const double _y = 0)
        : id(_id), x(_x), y(_y) {
    }
};

struct Arc {
    int id;
    Point *start;
    Point *end;
    Polygon *leftPoly;
    Polygon *rightPoly;
    Arc *Snext;
    Arc *Enext;

    explicit Arc(const int _id = 0)
        : id(_id),
          start(nullptr),
          end(nullptr),
          leftPoly(nullptr),
          rightPoly(nullptr),
          Snext(nullptr),
          Enext(nullptr) {
    }
};

struct Polygon {
    int id;
    vector<Arc *> edges;

    explicit Polygon(const int _id = 0) : id(_id) {
    }

    void addEdge(Arc *arc) {
        edges.push_back(arc);
    }
};


vector<Polygon *> generate(const vector<Arc *> &arcs) {
    vector<Polygon *> polygons;
    int polyId = 1;

    for (Arc *sc: arcs) {
        if (sc->leftPoly != nullptr && sc->rightPoly != nullptr)
            continue;
        if (sc->leftPoly == nullptr) {
            const auto pi = new Polygon(polyId++);
            Polygon *currentPoly = pi;
            Arc *cur = sc;
            while (true) {
                currentPoly->addEdge(cur);

                // 设置左多边形
                cur->leftPoly = currentPoly;

                const auto *N0 = cur->start;
                const auto *Nc = cur->end;

                // 回到起点，闭合
                if (N0 == Nc)
                    break;
                cur = cur->Snext;
                if (cur == nullptr)
                    break;
                if (Nc == cur->start) {
                    cur->leftPoly = currentPoly;
                } else if (Nc == cur->end) {
                    cur->rightPoly = currentPoly;
                } else {
                    break;
                }

                if (cur == sc)
                    break;
            }

            polygons.push_back(currentPoly);
        } else if (sc->rightPoly == nullptr) {
            const auto pi = new Polygon(polyId++);
            Polygon *currentPoly = pi;

            Arc *cur = sc;

            while (true) {
                currentPoly->addEdge(cur);
                cur->rightPoly = currentPoly;
                const auto *N0 = cur->end;
                const auto *Nc = cur->start;
                if (N0 == Nc)
                    break;
                cur = cur->Enext;
                if (cur == nullptr)
                    break;
                if (Nc == cur->start) {
                    cur->leftPoly = currentPoly;
                } else if (Nc == cur->end) {
                    cur->rightPoly = currentPoly;
                } else {
                    break;
                }

                if (cur == sc)
                    break;
            }

            polygons.push_back(currentPoly);
        }
    }

    return polygons;
}

int main() {
    Point p1(1, 0, 0);
    Point p2(2, 1, 0);
    Point p3(3, 1, 1);
    Point p4(4, 0, 1);

    Arc a1(1), a2(2), a3(3), a4(4);

    a1.start = &p1;
    a1.end = &p2;
    a2.start = &p2;
    a2.end = &p3;
    a3.start = &p3;
    a3.end = &p4;
    a4.start = &p4;
    a4.end = &p1;

    a1.Snext = &a2;
    a2.Snext = &a3;
    a3.Snext = &a4;
    a4.Snext = &a1;

    a1.Enext = &a4;
    a2.Enext = &a1;
    a3.Enext = &a2;
    a4.Enext = &a3;

    const vector arcs = {&a1, &a2, &a3, &a4};

    const auto polys = generate(arcs);

    cout << "生成多边形数量: " << polys.size() << endl;

    for (const auto p: polys) {
        cout << "Polygon " << p->id << " : ";
        for (const auto e: p->edges) {
            cout << e->id << " ";
        }
        cout << endl;
    }

    return 0;
}
