#define BOOSTVORONOI_EXPORTS
#include "boost_voronoi_wrapper.h"

#include <boost/polygon/voronoi.hpp>
#include <boost/multiprecision/cpp_int.hpp>
#include <memory>
#include <vector>
#if defined(BV_REFERENCE_INDEX_MAPS)
#include <unordered_map>
#endif
#include <cstdint>

using namespace boost::polygon;

// Used only when exact fractional contacts need a common integer denominator.
// The managed arrangement restricts coordinates to 50 bits. Doubled products
// therefore fit 128 bits; adaptive circle predicates retain arbitrary precision.
struct BvWideFloatConverter {
    template<typename T> double operator()(const T& value) const { return static_cast<double>(value); }
    double operator()(const detail::extended_exponent_fpt<double>& value) const { return value.d(); }
};
struct BvWideExtendedConverter {
    template<typename T> detail::extended_exponent_fpt<double> operator()(const T& value) const {
        if (value == 0) return detail::extended_exponent_fpt<double>(0);
        T magnitude = value < 0 ? -value : value;
        int shift = (std::max)(0, static_cast<int>(boost::multiprecision::msb(magnitude)) - 63);
        double leading = static_cast<double>(magnitude >> shift);
        return detail::extended_exponent_fpt<double>(value < 0 ? -leading : leading, shift);
    }
};
struct BvWideTraits {
    using int_type = int64_t;
    using int_x2_type = boost::multiprecision::int128_t;
    using uint_x2_type = boost::multiprecision::uint128_t;
    using big_int_type = boost::multiprecision::number<boost::multiprecision::cpp_int_backend<>, boost::multiprecision::et_off>;
    using fpt_type = double;
    using efpt_type = detail::extended_exponent_fpt<double>;
    using ulp_cmp_type = detail::ulp_comparison<double>;
    using to_fpt_converter_type = BvWideFloatConverter;
    using to_efpt_converter_type = BvWideExtendedConverter;
};

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

#if defined(BV_REFERENCE_INDEX_MAPS)
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
#endif

    // Boost stores each completed diagram container in a std::vector. No
    // container is mutated after construction, so pointer subtraction gives
    // exactly the same ordinal as the former ordered pointer-map pass.
    int32_t edge_index(const voronoi_edge<double>* value) const {
#if defined(BV_REFERENCE_INDEX_MAPS)
        return value ? edge_idx.at(value) : -1;
#else
        return value ? static_cast<int32_t>(value - vd.edges().data()) : -1;
#endif
    }
    int32_t vertex_index(const voronoi_vertex<double>* value) const {
#if defined(BV_REFERENCE_INDEX_MAPS)
        return value ? vertex_idx.at(value) : -1;
#else
        return value ? static_cast<int32_t>(value - vd.vertices().data()) : -1;
#endif
    }
    int32_t cell_index(const voronoi_cell<double>* value) const {
#if defined(BV_REFERENCE_INDEX_MAPS)
        return value ? cell_idx.at(value) : -1;
#else
        return value ? static_cast<int32_t>(value - vd.cells().data()) : -1;
#endif
    }
};

// ─── C API ───────────────────────────────────────────────────────────────────

extern "C" {

void* bv_construct_wide(
    const int64_t* x0, const int64_t* y0,
    const int64_t* x1, const int64_t* y1, int32_t count)
{
    try {
        voronoi_builder<int64_t, BvWideTraits> builder;
        const int64_t limit = int64_t(1) << 50;
        for (int32_t i = 0; i < count; ++i) {
            if (x0[i] < -limit || x0[i] > limit || y0[i] < -limit || y0[i] > limit ||
                x1[i] < -limit || x1[i] > limit || y1[i] < -limit || y1[i] > limit) return nullptr;
            builder.insert_segment(x0[i], y0[i], x1[i], y1[i]);
        }
        auto d = std::make_unique<BvDiagram>();
        builder.construct(&d->vd);
#if defined(BV_REFERENCE_INDEX_MAPS)
        d->build_index_maps();
#endif
        return d.release();
    } catch (...) {
        return nullptr;
    }
}

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
#if defined(BV_REFERENCE_INDEX_MAPS)
        d->build_index_maps();
#endif
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

        o.twin_index      = d->edge_index(e.twin());
        o.prev_index      = d->edge_index(e.prev());
        o.next_index      = d->edge_index(e.next());
        o.vertex0_index   = d->vertex_index(e.vertex0());
        o.vertex1_index   = d->vertex_index(e.vertex1());
        o.cell_index      = d->cell_index(e.cell());
        o.twin_cell_index = d->cell_index(e.twin() ? e.twin()->cell() : nullptr);

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
