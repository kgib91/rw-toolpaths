#pragma once
#include <stdint.h>

#ifdef _WIN32
#  ifdef BOOSTVORONOI_EXPORTS
#    define BV_API __declspec(dllexport)
#  else
#    define BV_API __declspec(dllimport)
#  endif
#else
#  define BV_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Flat edge record.  All index fields are -1 when the referenced element
 * does not exist (e.g. vertex0_index == -1 for an infinite edge).
 */
typedef struct {
    int32_t twin_index;
    int32_t prev_index;
    int32_t next_index;
    int32_t vertex0_index;
    int32_t vertex1_index;
    int32_t cell_index;
    int32_t twin_cell_index;
    int32_t is_primary;   /* 1 = primary, 0 = secondary */
    int32_t is_linear;
    int32_t is_curved;
    int32_t is_infinite;
} BvEdge;

typedef struct {
    double x;
    double y;
} BvVertex;

/*
 * source_category matches boost::polygon SOURCE_CATEGORY_* enum bits:
 *   0x0 = SINGLE_POINT
 *   0x1 = SEGMENT_START_POINT  (the "low" endpoint by Boost's y-then-x order)
 *   0x2 = SEGMENT_END_POINT    (the "high" endpoint)
 *   0x4 = INITIAL_SEGMENT
 *   0x8 = REVERSE_SEGMENT
 */
typedef struct {
    int32_t source_index;
    int32_t source_category;
    int32_t contains_point;  /* 1 if cell is a point site, 0 if segment site */
} BvCell;

/*
 * Construct a Voronoi diagram from integer segment sites supplied as four
 * parallel arrays:  (x0[i], y0[i]) -> (x1[i], y1[i])  for i in [0, count).
 *
 * Returns an opaque handle on success, NULL on failure.
 * The caller must release it with bv_destroy().
 */
BV_API void*   bv_construct(const int32_t* x0, const int32_t* y0,
                             const int32_t* x1, const int32_t* y1,
                             int32_t count);

BV_API void    bv_destroy(void* diagram);

BV_API int32_t bv_edge_count(void* diagram);
BV_API int32_t bv_vertex_count(void* diagram);
BV_API int32_t bv_cell_count(void* diagram);

/* Fill caller-allocated flat arrays.  count must match the corresponding
   bv_*_count() value; no bounds checking is performed inside. */
BV_API void bv_get_edges   (void* diagram, BvEdge*   out_buf, int32_t count);
BV_API void bv_get_vertices(void* diagram, BvVertex* out_buf, int32_t count);
BV_API void bv_get_cells   (void* diagram, BvCell*   out_buf, int32_t count);

#ifdef __cplusplus
}
#endif
