// Owned, untrusted positive-witness nomination. No CUDA result is a negative
// proof or cutting authority: the managed caller validates each pose against
// the literal physical profile and subdivides or searches on device misses.
struct DiscSegment {
    double ax, ay, az, bx, by, bz, maximum_radius;
    int source_path_index;
};
struct DiscNode {
    double x0, y0, x1, y1, maximum_radius;
    int first, count, left, right;
};
struct DiscSection {
    double bottom_radius, top_radius, height, start_height;
    int kind;
};
struct DiscQuery { double x, y, desired; };
struct DiscNomination { double parameter; int segment_ordinal, status; };
struct BoxQuery { double x0, y0, x1, y1, desired; };
struct BoxNomination { double t0, t1, t2, t3; int segment_ordinal, status; };
struct TriangleQuery { double ax, ay, bx, by, cx, cy, desired; };
struct TriangleNomination { double t0, t1, t2; int segment_ordinal, status; };

#if GPU_FP32_NOMINATION
typedef float NomScalar;
#define nomination_maximum ((NomScalar)3.402823466e38f)
#else
typedef double NomScalar;
#define nomination_maximum ((NomScalar)1.7976931348623157e308)
#endif
struct NomQuery { NomScalar x, y, desired; };
struct NomSegment { NomScalar ax, ay, az, bx, by, bz, maximum_radius; };
struct NomNode {
    NomScalar x0, y0, x1, y1, maximum_radius;
    int first, count, left, right;
};
struct NomSection { NomScalar bottom_radius, top_radius, height, start_height; int kind; };
__device__ NomScalar nom_sqrt(NomScalar value) {
#if GPU_FP32_NOMINATION
    return sqrtf(value);
#else
    return sqrt(value);
#endif
}
__device__ NomSegment load_segment(DiscSegment s, double origin_x, double origin_y) {
    // Subtraction stays FP64 BEFORE conversion. Tiny distances near large world
    // coordinates retain local precision; all output remains CPU validated.
#if GPU_FP32_NOMINATION
    NomSegment n = { (NomScalar)(s.ax - origin_x), (NomScalar)(s.ay - origin_y), (NomScalar)s.az,
        (NomScalar)(s.bx - origin_x), (NomScalar)(s.by - origin_y), (NomScalar)s.bz, (NomScalar)s.maximum_radius };
#else
    NomSegment n = { s.ax, s.ay, s.az, s.bx, s.by, s.bz, s.maximum_radius };
#endif
    return n;
}
__device__ NomNode load_node(DiscNode n, double origin_x, double origin_y) {
#if GPU_FP32_NOMINATION
    NomNode result = { (NomScalar)(n.x0 - origin_x), (NomScalar)(n.y0 - origin_y),
        (NomScalar)(n.x1 - origin_x), (NomScalar)(n.y1 - origin_y), (NomScalar)n.maximum_radius,
        n.first, n.count, n.left, n.right };
#else
    NomNode result = { n.x0, n.y0, n.x1, n.y1, n.maximum_radius, n.first, n.count, n.left, n.right };
#endif
    return result;
}
__device__ NomSection load_section(DiscSection s) {
    NomSection n = { (NomScalar)s.bottom_radius, (NomScalar)s.top_radius, (NomScalar)s.height,
        (NomScalar)s.start_height, s.kind }; return n;
}

__device__ NomScalar clamp01(NomScalar value) {
    return value < 0 ? 0 : value > 1 ? 1 : value;
}
__device__ NomScalar maximum(NomScalar a, NomScalar b) { return a > b ? a : b; }
__device__ NomScalar minimum(NomScalar a, NomScalar b) { return a < b ? a : b; }
__device__ NomScalar absolute(NomScalar a) { return a < 0 ? -a : a; }
__device__ NomScalar radius_at_depth(NomScalar depth, const DiscSection* sections, int count) {
    if (!(depth > 0)) return 0;
    NomScalar radius = 0;
    for (int i = 0; i < count; ++i) {
        NomSection s = load_section(sections[i]);
        if (s.start_height > depth) break;
        NomScalar h = minimum(s.height, depth - s.start_height);
        NomScalar top = s.kind == 1 ? nom_sqrt(maximum(0, h * (2 * s.top_radius - h)))
            : s.bottom_radius + (s.top_radius - s.bottom_radius) * h / s.height;
        radius = maximum(radius, maximum(s.bottom_radius, top));
    }
    return radius;
}
__device__ NomScalar pose_value(NomQuery q, NomSegment s, NomScalar t,
    const DiscSection* sections, int count) {
    NomScalar z = s.az + (s.bz - s.az) * t;
    if (!(z > 0)) return nomination_maximum;
    NomScalar dx = s.ax + (s.bx - s.ax) * t - q.x;
    NomScalar dy = s.ay + (s.by - s.ay) * t - q.y;
    return nom_sqrt(dx * dx + dy * dy) - radius_at_depth(z, sections, count);
}
__device__ NomScalar projection(NomQuery q, NomSegment s) {
    NomScalar dx = s.bx - s.ax, dy = s.by - s.ay, square = dx * dx + dy * dy;
    return square > 0 ? clamp01(((q.x - s.ax) * dx + (q.y - s.ay) * dy) / square) : 0;
}
__device__ NomScalar node_lower(NomQuery q, NomNode n) {
    NomScalar dx = maximum(0, maximum(n.x0 - q.x, q.x - n.x1));
    NomScalar dy = maximum(0, maximum(n.y0 - q.y, q.y - n.y1));
    return nom_sqrt(dx * dx + dy * dy) - n.maximum_radius - 1e-9;
}
__device__ bool try_pose(NomQuery q, NomSegment s, NomScalar t,
    const DiscSection* sections, int section_count, NomScalar* best_t, NomScalar* best_value) {
    NomScalar value = pose_value(q, s, t, sections, section_count);
    if (value < *best_value) { *best_value = value; *best_t = t; }
    return value <= q.desired;
}
__device__ bool find_pose(NomQuery q, NomSegment s, const DiscSection* sections,
    int section_count, double* selected) {
    NomScalar best_t = 0, best = nomination_maximum;
    if (try_pose(q, s, 0, sections, section_count, &best_t, &best)
        || try_pose(q, s, 1, sections, section_count, &best_t, &best)
        || try_pose(q, s, projection(q, s), sections, section_count, &best_t, &best)) {
        *selected = best_t; return true;
    }
    if (s.az == s.bz) return false;
    // First-hemisphere sphere-center projection is a cheap strong nomination.
    // Section membership and the final physical inequality still gate the pose.
    if (section_count > 0 && sections[0].kind == 1) {
        NomScalar radius = sections[0].top_radius;
        NomScalar dx = s.bx - s.ax, dy = s.by - s.ay, dz = s.az - s.bz;
        NomScalar square = dx * dx + dy * dy + dz * dz;
        NomScalar ox = s.ax - q.x, oy = s.ay - q.y, oz = radius - s.az;
        NomScalar dot = ox * dx + oy * dy + oz * dz;
        NomScalar t = square > 0 ? clamp01(-dot / square) : 0;
        bool first_hemisphere = s.az >= 0 && s.bz >= 0 && s.az <= radius && s.bz <= radius;
        if (first_hemisphere) {
            // Every possible disc witness must lie within this expanded sphere.
            // This is only a nomination prune: misses receive managed fallback.
            NomScalar cx = ox + dx * t, cy = oy + dy * t, cz = oz + dz * t;
            NomScalar expanded = radius + maximum(0, q.desired) + 1e-7;
            if (cx * cx + cy * cy + cz * cz > expanded * expanded) return false;
        }
        if (try_pose(q, s, t, sections, section_count, &best_t, &best)) {
            *selected = best_t; return true;
        }
        if (first_hemisphere && q.desired <= 0) {
            // For erosion e=-desired, passing poses obey
            // |C(t)|^2 + 2e*|Cxy(t)| + e^2 <= R^2. The left side is
            // convex. Safeguarded Newton steps cheaply nominate its minimum.
            // The real physical disc inequality, not this expression, accepts.
            NomScalar erosion = -q.desired, lo = 0, hi = 1;
            NomScalar lateral_square = dx * dx + dy * dy;
            for (int step = 0; step < 12; ++step) {
                NomScalar cx = ox + dx * t, cy = oy + dy * t;
                NomScalar distance_square = cx * cx + cy * cy;
                NomScalar distance = nom_sqrt(distance_square);
                if (!(distance > 1e-20)) break;
                NomScalar lateral_dot = cx * dx + cy * dy;
                NomScalar derivative = square * t + dot + erosion * lateral_dot / distance;
                if (derivative > 0) hi = t; else lo = t;
                NomScalar curvature = square + erosion * maximum(0,
                    lateral_square * distance_square - lateral_dot * lateral_dot) / (distance_square * distance);
                NomScalar next = curvature > 0 ? t - derivative / curvature : (lo + hi) / 2;
                if (!(next > lo && next < hi)) next = (lo + hi) / 2;
                if (next == t) break;
                t = next;
                if (try_pose(q, s, t, sections, section_count, &best_t, &best)) {
                    *selected = best_t; return true;
                }
            }
            return false;
        }
    }
    // Linear-section stationary poses use the exact lateral/radius gradient.
    NomScalar length = nom_sqrt((s.bx - s.ax) * (s.bx - s.ax) + (s.by - s.ay) * (s.by - s.ay));
    if (length > 0) {
        NomScalar along = ((q.x - s.ax) * (s.bx - s.ax) + (q.y - s.ay) * (s.by - s.ay)) / length;
        NomScalar across = absolute((q.x - s.ax) * (s.by - s.ay) - (q.y - s.ay) * (s.bx - s.ax)) / length;
        for (int i = 0; i < section_count; ++i) {
            NomSection section = load_section(sections[i]);
            if (section.kind != 0) continue;
            NomScalar gradient = (section.top_radius - section.bottom_radius) * (s.bz - s.az) / (section.height * length);
            if (!(absolute(gradient) < 1)) continue;
            NomScalar t = clamp01((along + gradient * across / nom_sqrt(1 - gradient * gradient)) / length);
            NomScalar depth = s.az + (s.bz - s.az) * t;
            if (depth < section.start_height || depth > section.start_height + section.height) continue;
            if (try_pose(q, s, t, sections, section_count, &best_t, &best)) {
                *selected = best_t; return true;
            }
            if (minimum(s.az, s.bz) >= section.start_height
                && maximum(s.az, s.bz) <= section.start_height + section.height) return false;
        }
    }
    // Analytic misses deliberately remain nominations without negative proof.
    // The reference golden-search arm exists for isolated performance replay.
#if GPU_GOLDEN_REFERENCE
    // The same forty-step pose search supplies real candidate poses only.
    const NomScalar ratio = .6180339887498949;
    NomScalar lo = 0, hi = 1, t0 = hi - (hi - lo) * ratio, t1 = lo + (hi - lo) * ratio;
    NomScalar v0 = pose_value(q, s, t0, sections, section_count);
    NomScalar v1 = pose_value(q, s, t1, sections, section_count);
    for (int step = 0; step < 40; ++step) {
        if (v0 < best) { best = v0; best_t = t0; }
        if (v1 < best) { best = v1; best_t = t1; }
        if (best <= q.desired) { *selected = best_t; return true; }
        if (v0 <= v1) {
            hi = t1; t1 = t0; v1 = v0; t0 = hi - (hi - lo) * ratio;
            v0 = pose_value(q, s, t0, sections, section_count);
        } else {
            lo = t0; t0 = t1; v0 = v1; t1 = lo + (hi - lo) * ratio;
            v1 = pose_value(q, s, t1, sections, section_count);
        }
    }
    if (v0 < best) { best = v0; best_t = t0; }
    if (v1 < best) { best = v1; best_t = t1; }
    if (best <= q.desired) { *selected = best_t; return true; }
#endif
    return false;
}

extern "C" __global__ void nominate_disc_witnesses(const DiscSegment* segments,
    const DiscNode* nodes, int node_count, const DiscSection* sections, int section_count,
    const DiscQuery* queries, DiscNomination* output, int query_count, int traversal_budget) {
    int ordinal = blockIdx.x * blockDim.x + threadIdx.x;
    if (ordinal >= query_count) return;
    DiscQuery original = queries[ordinal];
#if GPU_FP32_NOMINATION
    double origin_x = original.x, origin_y = original.y;
    NomQuery q = { 0, 0, (NomScalar)original.desired };
#else
    double origin_x = 0, origin_y = 0;
    NomQuery q = { original.x, original.y, original.desired };
#endif
    DiscNomination result = { 0, -1, 0 };
    int stack[64], pending = 1, visits = 0, tested = 0;
    stack[0] = 0;
    while (pending > 0) {
        int index = stack[--pending];
        if (index < 0 || index >= node_count || ++visits > traversal_budget) { result.status = 2; break; }
        NomNode node = load_node(nodes[index], origin_x, origin_y);
        if (node_lower(q, node) > q.desired) continue;
        if (node.left >= 0) {
            if (pending + 2 > 64 || node.left >= node_count || node.right < 0 || node.right >= node_count) { result.status = 2; break; }
            int first = node.left, last = node.right;
            if (node_lower(q, load_node(nodes[first], origin_x, origin_y))
                > node_lower(q, load_node(nodes[last], origin_x, origin_y))) { int swap = first; first = last; last = swap; }
            stack[pending++] = last; stack[pending++] = first;
            continue;
        }
        for (int i = node.first; i < node.first + node.count; ++i) {
            if (++tested > 512) { result.status = 2; break; }
            NomSegment s = load_segment(segments[i], origin_x, origin_y);
            NomScalar t = projection(q, s);
            NomScalar dx = s.ax + (s.bx - s.ax) * t - q.x;
            NomScalar dy = s.ay + (s.by - s.ay) * t - q.y;
            if (nom_sqrt(dx * dx + dy * dy) - s.maximum_radius - (NomScalar)1e-9 > q.desired) continue;
            if (find_pose(q, s, sections, section_count, &result.parameter)) {
                result.segment_ordinal = i; result.status = 1; output[ordinal] = result; return;
            }
        }
        if (result.status == 2) break;
    }
    output[ordinal] = result;
}

extern "C" __global__ void nominate_box_witnesses(const DiscSegment* segments,
    const DiscNode* nodes, int node_count, const DiscSection* sections, int section_count,
    const BoxQuery* queries, BoxNomination* output, int query_count, int traversal_budget) {
    int ordinal = blockIdx.x * blockDim.x + threadIdx.x;
    if (ordinal >= query_count) return;
    BoxQuery box = queries[ordinal];
#if GPU_FP32_NOMINATION
    double origin_x = box.x0 + (box.x1 - box.x0) / 2, origin_y = box.y0 + (box.y1 - box.y0) / 2;
    NomQuery center = { 0, 0, (NomScalar)box.desired };
#else
    double origin_x = 0, origin_y = 0;
    NomQuery center = { box.x0 + (box.x1 - box.x0) / 2, box.y0 + (box.y1 - box.y0) / 2, box.desired };
#endif
    NomQuery corners[4] = { { (NomScalar)(box.x0-origin_x), (NomScalar)(box.y0-origin_y), (NomScalar)box.desired },
        { (NomScalar)(box.x1-origin_x), (NomScalar)(box.y0-origin_y), (NomScalar)box.desired },
        { (NomScalar)(box.x1-origin_x), (NomScalar)(box.y1-origin_y), (NomScalar)box.desired },
        { (NomScalar)(box.x0-origin_x), (NomScalar)(box.y1-origin_y), (NomScalar)box.desired } };
    BoxNomination result = { 0, 0, 0, 0, -1, 0 };
    int stack[64], pending = 1, visits = 0, tested = 0; stack[0] = 0;
    while (pending > 0) {
        int index = stack[--pending];
        if (index < 0 || index >= node_count || ++visits > traversal_budget) { result.status = 2; break; }
        NomNode node = load_node(nodes[index], origin_x, origin_y);
        if (node_lower(center, node) > center.desired) continue;
        if (node.left >= 0) {
            if (pending + 2 > 64 || node.left >= node_count || node.right < 0 || node.right >= node_count) { result.status = 2; break; }
            int first = node.left, last = node.right;
            if (node_lower(center, load_node(nodes[first], origin_x, origin_y))
                > node_lower(center, load_node(nodes[last], origin_x, origin_y))) { int swap = first; first = last; last = swap; }
            stack[pending++] = last; stack[pending++] = first; continue;
        }
        for (int i = node.first; i < node.first + node.count; ++i) {
            if (++tested > 512) { result.status = 2; break; }
            NomSegment s = load_segment(segments[i], origin_x, origin_y);
            NomScalar projected = projection(center, s);
            NomScalar dx = s.ax + (s.bx - s.ax) * projected - center.x;
            NomScalar dy = s.ay + (s.by - s.ay) * projected - center.y;
            if (nom_sqrt(dx * dx + dy * dy) - s.maximum_radius - (NomScalar)1e-9 > center.desired) continue;
            if (!find_pose(corners[0], s, sections, section_count, &result.t0)
                || !find_pose(corners[1], s, sections, section_count, &result.t1)
                || !find_pose(corners[2], s, sections, section_count, &result.t2)
                || !find_pose(corners[3], s, sections, section_count, &result.t3)) continue;
            double first_t = minimum(minimum(result.t0, result.t1), minimum(result.t2, result.t3));
            double last_t = maximum(maximum(result.t0, result.t1), maximum(result.t2, result.t3));
            double first_depth = s.az + (s.bz - s.az) * first_t, last_depth = s.az + (s.bz - s.az) * last_t;
            double low = minimum(first_depth, last_depth), high = maximum(first_depth, last_depth);
            bool same_section = false;
            if (low >= .001)
                for (int section = 0; section < section_count; ++section)
                    if (low > sections[section].start_height + 1e-12
                        && high < sections[section].start_height + sections[section].height - 1e-12) same_section = true;
            if (!same_section) continue;
            result.segment_ordinal = i; result.status = 1; output[ordinal] = result; return;
        }
        if (result.status == 2) break;
    }
    output[ordinal] = result;
}

extern "C" __global__ void nominate_triangle_witnesses(const DiscSegment* segments,
    const DiscNode* nodes, int node_count, const DiscSection* sections, int section_count,
    const TriangleQuery* queries, TriangleNomination* output, int query_count, int traversal_budget) {
    int ordinal = blockIdx.x * blockDim.x + threadIdx.x;
    if (ordinal >= query_count) return;
    TriangleQuery triangle = queries[ordinal];
    double centroid_x = triangle.ax + (triangle.bx - triangle.ax) / 3 + (triangle.cx - triangle.ax) / 3;
    double centroid_y = triangle.ay + (triangle.by - triangle.ay) / 3 + (triangle.cy - triangle.ay) / 3;
#if GPU_FP32_NOMINATION
    // Translate using original double vertices before scalar conversion.
    double origin_x = centroid_x, origin_y = centroid_y;
    NomQuery center = { 0, 0, (NomScalar)triangle.desired };
#else
    double origin_x = 0, origin_y = 0;
    NomQuery center = { centroid_x, centroid_y, triangle.desired };
#endif
    NomQuery vertices[3] = { { (NomScalar)(triangle.ax-origin_x), (NomScalar)(triangle.ay-origin_y), (NomScalar)triangle.desired },
        { (NomScalar)(triangle.bx-origin_x), (NomScalar)(triangle.by-origin_y), (NomScalar)triangle.desired },
        { (NomScalar)(triangle.cx-origin_x), (NomScalar)(triangle.cy-origin_y), (NomScalar)triangle.desired } };
    TriangleNomination result = { 0, 0, 0, -1, 0 };
    int stack[64], pending = 1, visits = 0, tested = 0; stack[0] = 0;
    while (pending > 0) {
        int index = stack[--pending];
        if (index < 0 || index >= node_count || ++visits > traversal_budget) { result.status = 2; break; }
        NomNode node = load_node(nodes[index], origin_x, origin_y);
        // Broad phase chooses candidates only. Its misses/refusals confer no
        // exterior, negative coverage, or permission to drop this triangle.
        // One actual interval must cover ALL vertices. An enclosing node must
        // therefore reach each vertex with its maximum physical radius. The
        // existing lower-bound margin remains; scalar error can only lose a
        // nominee, never gain machining authority.
        if (node_lower(vertices[0], node) > vertices[0].desired
            || node_lower(vertices[1], node) > vertices[1].desired
            || node_lower(vertices[2], node) > vertices[2].desired) continue;
        if (node.left >= 0) {
            if (pending + 2 > 64 || node.left >= node_count || node.right < 0 || node.right >= node_count) { result.status = 2; break; }
            int first = node.left, last = node.right;
            if (node_lower(center, load_node(nodes[first], origin_x, origin_y))
                > node_lower(center, load_node(nodes[last], origin_x, origin_y))) { int swap = first; first = last; last = swap; }
            stack[pending++] = last; stack[pending++] = first; continue;
        }
        for (int i = node.first; i < node.first + node.count; ++i) {
            if (++tested > 512) { result.status = 2; break; }
            NomSegment s = load_segment(segments[i], origin_x, origin_y);
            bool reaches_vertices = true;
            for (int vertex = 0; vertex < 3; ++vertex) {
                NomScalar projected = projection(vertices[vertex], s);
                NomScalar dx = s.ax + (s.bx - s.ax) * projected - vertices[vertex].x;
                NomScalar dy = s.ay + (s.by - s.ay) * projected - vertices[vertex].y;
                if (nom_sqrt(dx * dx + dy * dy) - s.maximum_radius - (NomScalar)1e-9 > vertices[vertex].desired) {
                    reaches_vertices = false; break;
                }
            }
            if (!reaches_vertices) continue;
            if (!find_pose(vertices[0], s, sections, section_count, &result.t0)
                || !find_pose(vertices[1], s, sections, section_count, &result.t1)
                || !find_pose(vertices[2], s, sections, section_count, &result.t2)) continue;
            NomScalar first_t = minimum((NomScalar)result.t0, minimum((NomScalar)result.t1, (NomScalar)result.t2));
            NomScalar last_t = maximum((NomScalar)result.t0, maximum((NomScalar)result.t1, (NomScalar)result.t2));
            NomScalar first_depth = s.az + (s.bz - s.az) * first_t, last_depth = s.az + (s.bz - s.az) * last_t;
            NomScalar low = minimum(first_depth, last_depth), high = maximum(first_depth, last_depth);
            bool same_section = false;
            if (low >= (NomScalar).001)
                for (int section = 0; section < section_count; ++section) {
                    NomSection span = load_section(sections[section]);
                    if (low > span.start_height + (NomScalar)1e-12
                        && high < span.start_height + span.height - (NomScalar)1e-12) same_section = true;
                }
            if (!same_section) continue;
            result.segment_ordinal = i; result.status = 1; output[ordinal] = result; return;
        }
        if (result.status == 2) break;
    }
    output[ordinal] = result;
}
// Boundary samples are nominations only. No distance, winding, or refusal
// establishes cutting/coverage authority. Managed input validation guarantees
// |all original integer coordinates| < 2^29 and a complete immutable BVH.
struct BoundarySegment { long long ax, ay, bx, by; };
struct BoundaryNode { long long x0, y0, x1, y1; int first, count, left, right; };
struct BoundaryQuery { long long x, y; };
struct BoundarySampleNomination {
    double parameter, squared_distance;
    int segment_ordinal, winding, status, refusal_reason;
};

__device__ bool boundary_in_domain(long long value) {
    return value > -(1LL << 29) && value < (1LL << 29);
}
__device__ BoundarySampleNomination boundary_refused(int reason) {
    BoundarySampleNomination n = { __longlong_as_double(0x7ff8000000000000LL),
        __longlong_as_double(0x7ff0000000000000LL), -1, 0, 2, reason }; return n;
}
__device__ double boundary_node_square(BoundaryQuery q, BoundaryNode n) {
    long long ix = q.x < n.x0 ? n.x0 - q.x : q.x > n.x1 ? q.x - n.x1 : 0;
    long long iy = q.y < n.y0 ? n.y0 - q.y : q.y > n.y1 ? q.y - n.y1 : 0;
    double dx = (double)ix, dy = (double)iy; return dx * dx + dy * dy;
}
__device__ bool boundary_ray_needs_node(BoundaryQuery q, BoundaryNode n) {
    // Equality at MaxX remains conservative. Half-open Y avoids duplicate
    // vertex crossings; horizontal-only bounds cannot contribute winding.
    return n.y0 <= q.y && n.y1 > q.y && n.x1 >= q.x;
}
__device__ bool boundary_nearest_needs_node(double lower, double best) {
    // Distance is nomination-only. Keep a rounding cushion at AABB ties.
    return lower <= best + 1e-12 * (best > 1 ? best : 1);
}
__device__ double boundary_finite_square(BoundaryQuery q, BoundarySegment s, double* parameter) {
    // Integer differences are exact before conversion; their magnitudes <2^30
    // are exactly representable as doubles. Projection is onto a FINITE edge.
    double dx = (double)(s.bx - s.ax), dy = (double)(s.by - s.ay);
    double qx = (double)(q.x - s.ax), qy = (double)(q.y - s.ay);
    double square = dx * dx + dy * dy;
    double t = square > 0 ? (qx * dx + qy * dy) / square : 0;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    double ex = qx - dx * t, ey = qy - dy * t;
    *parameter = t; return ex * ex + ey * ey;
}
__device__ int boundary_winding_delta(BoundaryQuery q, BoundarySegment s) {
    bool up = s.ay <= q.y && s.by > q.y;
    bool down = s.ay > q.y && s.by <= q.y;
    if (!up && !down) return 0;
    // Each difference <2^30, each product <2^60, cross magnitude <2^61:
    // signed64 arithmetic is exact without overflow in the admitted domain.
    long long cross = (s.bx - s.ax) * (q.y - s.ay) - (s.by - s.ay) * (q.x - s.ax);
    return up && cross > 0 ? 1 : down && cross < 0 ? -1 : 0;
}

extern "C" __global__ void nominate_boundary_samples(const BoundarySegment* segments,
    const BoundaryNode* nodes, int segment_count, int node_count,
    const BoundaryQuery* queries, BoundarySampleNomination* nominations, int query_count,
    int maximum_nodes, int maximum_segments) {
    int query_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (query_index >= query_count) return;
    BoundaryQuery q = queries[query_index]; BoundarySampleNomination refused = boundary_refused(4);
    nominations[query_index] = refused;
    if (!boundary_in_domain(q.x) || !boundary_in_domain(q.y) || segment_count <= 0
        || node_count <= 0 || maximum_nodes <= 0 || maximum_segments <= 0) return;
    int stack[64], top = 1, visited = 0, tested = 0, best_ordinal = -1, winding = 0;
    stack[0] = 0; double best_square = 1.7976931348623157e308, best_t = 0;
    while (top > 0) {
        if (visited >= maximum_nodes) { nominations[query_index] = boundary_refused(1); return; }
        int index = stack[--top]; ++visited;
        if (index < 0 || index >= node_count) return;
        BoundaryNode node = nodes[index];
        if (!boundary_in_domain(node.x0) || !boundary_in_domain(node.y0)
            || !boundary_in_domain(node.x1) || !boundary_in_domain(node.y1)
            || node.x0 > node.x1 || node.y0 > node.y1) return;
        bool winding_work = boundary_ray_needs_node(q, node);
        bool nearest_work = boundary_nearest_needs_node(boundary_node_square(q, node), best_square);
        if (!winding_work && !nearest_work) continue;
        if (node.left == -1 && node.right == -1) {
            if (node.first < 0 || node.count <= 0 || node.count > segment_count
                || node.first > segment_count - node.count) return;
            for (int ordinal = node.first; ordinal < node.first + node.count; ++ordinal) {
                if (tested >= maximum_segments) { nominations[query_index] = boundary_refused(2); return; }
                ++tested; BoundarySegment s = segments[ordinal];
                if (!boundary_in_domain(s.ax) || !boundary_in_domain(s.ay)
                    || !boundary_in_domain(s.bx) || !boundary_in_domain(s.by)) return;
                if (winding_work) winding += boundary_winding_delta(q, s);
                if (nearest_work) {
                    double t = 0, square = boundary_finite_square(q, s, &t);
                    if (square < best_square || (square == best_square && (best_ordinal < 0 || ordinal < best_ordinal))) {
                        best_square = square; best_t = t; best_ordinal = ordinal;
                    }
                }
            }
        } else {
            if (node.first != 0 || node.count != 0 || node.left < 0 || node.right < 0
                || node.left >= node_count || node.right >= node_count || node.left == node.right) return;
            if (top > 62) { nominations[query_index] = boundary_refused(3); return; }
            double left_square = boundary_node_square(q, nodes[node.left]);
            double right_square = boundary_node_square(q, nodes[node.right]);
            // Pop the nearer node first to obtain a useful finite-edge bound.
            // Ray work is independently retained even when that node is far.
            if (left_square <= right_square) { stack[top++] = node.right; stack[top++] = node.left; }
            else { stack[top++] = node.left; stack[top++] = node.right; }
        }
    }
    if (best_ordinal < 0) return;
    BoundarySampleNomination result = { best_t, best_square, best_ordinal, winding, 1, 0 };
    nominations[query_index] = result;
}

// Optional float nearest-edge NOMINATION. Exact original int64 winding and
// double external buffers remain unchanged. CPU physical predicates retain
// authority; approximate distance, winding, or misses cannot establish stock.
__device__ bool boundary_fp32_relative_domain(BoundaryQuery q, BoundaryNode root) {
    const long long limit = 1LL << 24;
    return root.x1 - root.x0 <= limit && root.y1 - root.y0 <= limit
        && q.x - root.x0 >= -limit && q.x - root.x0 <= limit
        && q.x - root.x1 >= -limit && q.x - root.x1 <= limit
        && q.y - root.y0 >= -limit && q.y - root.y0 <= limit
        && q.y - root.y1 >= -limit && q.y - root.y1 <= limit;
}
__device__ float boundary_node_square_fp32(BoundaryQuery q, BoundaryNode n) {
    long long ix = q.x < n.x0 ? n.x0 - q.x : q.x > n.x1 ? q.x - n.x1 : 0;
    long long iy = q.y < n.y0 ? n.y0 - q.y : q.y > n.y1 ? q.y - n.y1 : 0;
    // Integer local subtraction precedes float conversion. Absolute grid
    // coordinates never enter float arithmetic; guard covers every delta.
    float dx = (float)ix, dy = (float)iy; return dx * dx + dy * dy;
}
__device__ bool boundary_nearest_needs_node_fp32(float lower, float best) {
    // A nomination cushion only. Rounding/pruning misses confer no authority.
    return lower <= best + 1e-5f * (best > 1 ? best : 1);
}
__device__ float boundary_finite_square_fp32(BoundaryQuery q, BoundarySegment s, float* parameter) {
    float dx = (float)(s.bx - s.ax), dy = (float)(s.by - s.ay);
    float qx = (float)(q.x - s.ax), qy = (float)(q.y - s.ay);
    float square = dx * dx + dy * dy;
    float t = square > 0 ? (qx * dx + qy * dy) / square : 0;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    float ex = qx - dx * t, ey = qy - dy * t;
    *parameter = t; return ex * ex + ey * ey;
}
extern "C" __global__ void nominate_boundary_samples_fp32(const BoundarySegment* segments,
    const BoundaryNode* nodes, int segment_count, int node_count,
    const BoundaryQuery* queries, BoundarySampleNomination* nominations, int query_count,
    int maximum_nodes, int maximum_segments) {
    int query_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (query_index >= query_count) return;
    BoundaryQuery q = queries[query_index]; nominations[query_index] = boundary_refused(4);
    if (!boundary_in_domain(q.x) || !boundary_in_domain(q.y) || segment_count <= 0
        || node_count <= 0 || maximum_nodes <= 0 || maximum_segments <= 0
        || !boundary_fp32_relative_domain(q, nodes[0])) return;
    int stack[64], top = 1, visited = 0, tested = 0, best_ordinal = -1, winding = 0;
    stack[0] = 0; float best_square = 3.402823466e38f, best_t = 0;
    while (top > 0) {
        if (visited >= maximum_nodes) { nominations[query_index] = boundary_refused(1); return; }
        int index = stack[--top]; ++visited;
        if (index < 0 || index >= node_count) return;
        BoundaryNode node = nodes[index];
        if (!boundary_in_domain(node.x0) || !boundary_in_domain(node.y0)
            || !boundary_in_domain(node.x1) || !boundary_in_domain(node.y1)
            || node.x0 > node.x1 || node.y0 > node.y1) return;
        bool winding_work = boundary_ray_needs_node(q, node);
        bool nearest_work = boundary_nearest_needs_node_fp32(boundary_node_square_fp32(q, node), best_square);
        if (!winding_work && !nearest_work) continue;
        if (node.left == -1 && node.right == -1) {
            if (node.first < 0 || node.count <= 0 || node.count > segment_count
                || node.first > segment_count - node.count) return;
            for (int ordinal = node.first; ordinal < node.first + node.count; ++ordinal) {
                if (tested >= maximum_segments) { nominations[query_index] = boundary_refused(2); return; }
                ++tested; BoundarySegment s = segments[ordinal];
                if (!boundary_in_domain(s.ax) || !boundary_in_domain(s.ay)
                    || !boundary_in_domain(s.bx) || !boundary_in_domain(s.by)) return;
                // This uses ONLY original signed64 operands and the original
                // half-open predicate. Nearest numeric mode cannot change it.
                if (winding_work) winding += boundary_winding_delta(q, s);
                if (nearest_work) {
                    float t = 0, square = boundary_finite_square_fp32(q, s, &t);
                    if (!(t >= 0 && t <= 1 && square >= 0 && square <= 3.402823466e38f)) return;
                    if (square < best_square || (square == best_square && (best_ordinal < 0 || ordinal < best_ordinal))) {
                        best_square = square; best_t = t; best_ordinal = ordinal;
                    }
                }
            }
        } else {
            if (node.first != 0 || node.count != 0 || node.left < 0 || node.right < 0
                || node.left >= node_count || node.right >= node_count || node.left == node.right) return;
            if (top > 62) { nominations[query_index] = boundary_refused(3); return; }
            float left_square = boundary_node_square_fp32(q, nodes[node.left]);
            float right_square = boundary_node_square_fp32(q, nodes[node.right]);
            if (left_square <= right_square) { stack[top++] = node.right; stack[top++] = node.left; }
            else { stack[top++] = node.left; stack[top++] = node.right; }
        }
    }
    if (best_ordinal < 0) return;
    BoundarySampleNomination result = { (double)best_t, (double)best_square, best_ordinal, winding, 1, 0 };
    nominations[query_index] = result;
}
