#define BOOSTVORONOI_EXPORTS
#include "boost_voronoi_wrapper.h"

#include <boost/polygon/voronoi.hpp>
#include <vector>
#include <unordered_map>
#include <cstdint>

using namespace boost::polygon;

// ─── Input segment type with Boost.Polygon specialisations ───────────────────

struct BvInputSegment {
    point_data<int32_t> p0, p1;
    BvInputSegment(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
        : p0(x0, y0), p1(x1, y1) {}
};

namespace boost { namespace polygon {
    template<> struct geometry_concept<BvInputSegment> {
        typedef segment_concept type;
    };
    template<> struct segment_traits<BvInputSegment> {
        typedef int32_t            coordinate_type;
        typedef point_data<int32_t> point_type;
        static point_type get(const BvInputSegment& s, direction_1d dir) {
            return dir.to_int() ? s.p1 : s.p0;
        }
    };
}} // namespace boost::polygon

// ─── Opaque diagram handle ────────────────────────────────────────────────────

struct BvDiagram {
    voronoi_diagram<double> vd;

    // Pointer-to-index maps built once after construction.
    std::unordered_map<const voronoi_edge<double>*,   int32_t> edge_idx;
    std::unordered_map<const voronoi_vertex<double>*, int32_t> vertex_idx;
    std::unordered_map<const voronoi_cell<double>*,   int32_t> cell_idx;

    void build_index_maps() {
        int32_t i = 0;
        for (const auto& e : vd.edges())    edge_idx[&e]   = i++;
        i = 0;
        for (const auto& v : vd.vertices()) vertex_idx[&v] = i++;
        i = 0;
        for (const auto& c : vd.cells())    cell_idx[&c]   = i++;
    }
};

// ─── C API ───────────────────────────────────────────────────────────────────

extern "C" {

void* bv_construct(
    const int32_t* x0, const int32_t* y0,
    const int32_t* x1, const int32_t* y1,
    int32_t count)
{
    try {
        std::vector<BvInputSegment> segs;
        segs.reserve(static_cast<size_t>(count));
        for (int32_t i = 0; i < count; ++i)
            segs.emplace_back(x0[i], y0[i], x1[i], y1[i]);

        auto* d = new BvDiagram();
        construct_voronoi(segs.begin(), segs.end(), &d->vd);
        d->build_index_maps();
        return d;
    } catch (...) {
        return nullptr;
    }
}

void bv_destroy(void* diagram) {
    delete static_cast<BvDiagram*>(diagram);
}

int32_t bv_edge_count(void* diagram) {
    return static_cast<int32_t>(
        static_cast<BvDiagram*>(diagram)->vd.edges().size());
}

int32_t bv_vertex_count(void* diagram) {
    return static_cast<int32_t>(
        static_cast<BvDiagram*>(diagram)->vd.vertices().size());
}

int32_t bv_cell_count(void* diagram) {
    return static_cast<int32_t>(
        static_cast<BvDiagram*>(diagram)->vd.cells().size());
}

void bv_get_edges(void* diagram, BvEdge* out, int32_t count) {
    auto* d = static_cast<BvDiagram*>(diagram);
    int32_t i = 0;
    for (const auto& e : d->vd.edges()) {
        if (i >= count) break;
        BvEdge& o = out[i++];

        o.twin_index      = e.twin()                       ? d->edge_idx.at(e.twin())           : -1;
        o.prev_index      = e.prev()                       ? d->edge_idx.at(e.prev())           : -1;
        o.next_index      = e.next()                       ? d->edge_idx.at(e.next())           : -1;
        o.vertex0_index   = e.vertex0()                    ? d->vertex_idx.at(e.vertex0())      : -1;
        o.vertex1_index   = e.vertex1()                    ? d->vertex_idx.at(e.vertex1())      : -1;
        o.cell_index      = e.cell()                       ? d->cell_idx.at(e.cell())           : -1;
        o.twin_cell_index = (e.twin() && e.twin()->cell()) ? d->cell_idx.at(e.twin()->cell())   : -1;

        o.is_primary  = e.is_primary() ? 1 : 0;
        o.is_linear   = e.is_linear()  ? 1 : 0;
        o.is_curved   = e.is_curved()  ? 1 : 0;
        o.is_infinite = e.is_finite()  ? 0 : 1;
    }
}

void bv_get_vertices(void* diagram, BvVertex* out, int32_t count) {
    auto* d = static_cast<BvDiagram*>(diagram);
    int32_t i = 0;
    for (const auto& v : d->vd.vertices()) {
        if (i >= count) break;
        out[i].x = v.x();
        out[i].y = v.y();
        ++i;
    }
}

void bv_get_cells(void* diagram, BvCell* out, int32_t count) {
    auto* d = static_cast<BvDiagram*>(diagram);
    int32_t i = 0;
    for (const auto& c : d->vd.cells()) {
        if (i >= count) break;
        out[i].source_index    = static_cast<int32_t>(c.source_index());
        out[i].source_category = static_cast<int32_t>(c.source_category());
        out[i].contains_point  = c.contains_point() ? 1 : 0;
        ++i;
    }
}

} // extern "C"
