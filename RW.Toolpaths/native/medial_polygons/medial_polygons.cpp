#include "clipper2/clipper.h"
#include <cstdint>
#include <memory>
#if defined(_WIN32)
#define C2M_API extern "C" __declspec(dllexport)
#else
#define C2M_API extern "C" __attribute__((visibility("default")))
#endif
using namespace Clipper2Lib;
struct MedialPolygons { Paths64 paths; };
C2M_API int c2m_abi() { return 1; }
C2M_API const char* c2m_version() { return CLIPPER2_VERSION; }
C2M_API void* c2m_execute(const int64_t* xy, const int* lengths, int count,
    int operation, int fill_rule, double distance) noexcept {
  try {
    if (count < 0 || (count && (!xy || !lengths)) || (operation != 0 && operation != 1)) return nullptr;
    Paths64 paths; size_t offset = 0;
    for (int i=0; i<count; ++i) {
      if (lengths[i] < 0) return nullptr;
      Path64 path; path.reserve(lengths[i]);
      for (int j=0;j<lengths[i];++j) { path.emplace_back(xy[offset],xy[offset+1]); offset+=2; }
      paths.emplace_back(std::move(path));
    }
    auto result=std::make_unique<MedialPolygons>();
    result->paths = operation == 0 ? Union(paths,static_cast<FillRule>(fill_rule))
      : InflatePaths(paths,distance,JoinType::Miter,EndType::Polygon,2,0);
    return result.release();
  } catch (...) { return nullptr; }
}
C2M_API int c2m_path_count(void* value) { return value ? static_cast<int>(static_cast<MedialPolygons*>(value)->paths.size()) : -1; }
C2M_API int c2m_point_count(void* value,int index) {
  auto* result=static_cast<MedialPolygons*>(value);
  return result && index>=0 && index<static_cast<int>(result->paths.size()) ? static_cast<int>(result->paths[index].size()) : -1;
}
C2M_API int c2m_copy_path(void* value,int index,int64_t* xy,int capacity) {
  int count=c2m_point_count(value,index); if(count<0 || capacity<count || (count && !xy))return -1;
  const auto& path=static_cast<MedialPolygons*>(value)->paths[index];
  for(int i=0;i<count;++i){xy[2*i]=path[i].x;xy[2*i+1]=path[i].y;}return count;
}
C2M_API void c2m_destroy(void* value) { delete static_cast<MedialPolygons*>(value); }
