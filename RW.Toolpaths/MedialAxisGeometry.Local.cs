using Clipper2Lib;
using RW.Toolpaths.Milling;

namespace RW.Toolpaths;

public static partial class MedialAxisGeometry
{
    /// <summary>Shares full-source conditioning across conservative local site queries.
    /// The subset never chooses a new origin, scale, maximum radius or boundary.</summary>
    internal sealed class PreparedLocalSource
    {
        private readonly double _originX, _originY, _scale;
        private readonly BoostVoronoiProvider.LocalSiteSet _sites;
        internal int SiteCount => _sites.Sites.Count;

        internal PreparedLocalSource(MillingRegion source, CancellationToken token, bool preserveNativeGrid = false)
        {
            var rings = new[] { source.Outer }.Concat(source.Holes).ToArray();
            double minX = rings.Min(ring => ring.Min(p => p.x)), minY = rings.Min(ring => ring.Min(p => p.y));
            double maxX = rings.Max(ring => ring.Max(p => p.x)), maxY = rings.Max(ring => ring.Max(p => p.y));
            double width = maxX - minX, height = maxY - minY, span = Math.Max(width, height);
            _originX = minX + width / 2; _originY = minY + height / 2;
            // Matches OffsetTerminalPaths.BuildSourceMedial's full-source call.
            double sourceOuterSpan = Math.Max(source.Outer.Max(p => p.x) - source.Outer.Min(p => p.x),
                source.Outer.Max(p => p.y) - source.Outer.Min(p => p.y));
            double requestedRadius = Math.Max(1, sourceOuterSpan * 2);
            _scale = Math.Min(PathUtils.Scale, MaximumProviderSpan / span);
            _scale = Math.Min(_scale, (int.MaxValue - 1d) / requestedRadius);
            if (!double.IsFinite(_scale) || _scale <= 0) throw new ArgumentException("Invalid local medial conditioning.");
            if (preserveNativeGrid)
            {
                _originX = Math.Round(_originX * _scale) / _scale;
                _originY = Math.Round(_originY * _scale) / _scale;
            }
            _sites = new(Scale(source.Outer), source.Holes.Select(hole => (IReadOnlyList<PointD>)Scale(hole)).ToArray(), token);
            Console.WriteLine($"[Offset.Terminal.LocalPreparation] sites={SiteCount} originX={_originX:R} "
                + $"originY={_originY:R} scale={_scale:R} conditioning=complete-original-source nativeGridTranslation={preserveNativeGrid}");
            List<PointD> Scale(IReadOnlyList<PointD> ring)
            {
                var output = new List<PointD>(ring.Count);
                for (int i = 0; i < ring.Count; i++)
                {
                    if ((i & 4095) == 0) token.ThrowIfCancellationRequested();
                    output.Add(ConditionSourcePoint(ring[i], _originX, _originY, _scale, preserveNativeGrid));
                }
                return output;
            }
        }

        internal IReadOnlyList<MedialSegment>? TryBuild(Rect64 worldGridWindow, CancellationToken token)
        {
            const double maximumWorldChordError = .0005;
            var query = new RectD((worldGridWindow.left / PathUtils.Scale - _originX) * _scale,
                (worldGridWindow.top / PathUtils.Scale - _originY) * _scale,
                (worldGridWindow.right / PathUtils.Scale - _originX) * _scale,
                (worldGridWindow.bottom / PathUtils.Scale - _originY) * _scale);
            var diagram = BoostVoronoiProvider.TryConstructLocalMedialAxis(_sites, query,
                maximumWorldChordError * _scale, 3 * Math.PI / 4, token);
            if (diagram is null) return null;
            var result = diagram.Segments.Select(segment => new MedialSegment(World(segment.Point0), World(segment.Point1))).ToArray();
            Console.WriteLine($"[Offset.Terminal.LocalCertificate] originalSites={diagram.SourceSiteCount} "
                + $"retainedSites={diagram.OriginalSiteOrdinals.Count} segments={result.Length} "
                + $"upperMm={diagram.InfluenceUpperBound / _scale:R} arithmeticMm={diagram.NumericalReserve / _scale:R} "
                + $"maximumChordError={maximumWorldChordError:R} nearestSiteIdentity=complete-original "
                + "fullSignedMaterial=caller-authority replacementWalls=0");
            return result;
        }
        private MedialPoint World(MedialPoint point) => new(point.X / _scale + _originX,
            point.Y / _scale + _originY, point.Radius / _scale);
    }
}
