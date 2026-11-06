using Clipper2Lib;

namespace RW.Toolpaths.Geometry;

/// <summary>Bounded memoization of complete, unchanged geometric proof inputs.</summary>
internal static class FilledFrontEvidenceCache
{
    internal static bool Enabled
        => Environment.GetEnvironmentVariable("RW_TOOLPATHS_FRONT_EVIDENCE_REFERENCE") != "1";

    internal sealed class Key
    {
        private readonly long[] _words;
        internal ulong Hash { get; }
        internal long Bytes => 64L + sizeof(long) * (long)_words.Length;
        internal int Points { get; }

        private Key(long[] words, int points, CancellationToken token)
        {
            _words = words;
            Points = points;
            ulong hash = 14695981039346656037UL;
            for (int index = 0; index < words.Length; index++)
            {
                if ((index & 511) == 0) token.ThrowIfCancellationRequested();
                hash ^= unchecked((ulong)words[index]);
                hash = unchecked(hash * 1099511628211UL);
            }
            Hash = hash;
        }

        internal bool Same(Key other) => Hash == other.Hash
            && _words.AsSpan().SequenceEqual(other._words);

        internal static Key? Capture(IEnumerable<IReadOnlyList<PointD>> paths,
            double radius, double arcTolerance, double coverageError, int modes,
            IReadOnlyList<PointD>? clip, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            var rings = paths.ToArray();
            long points = rings.Sum(path => (long)path.Count) + (clip?.Count ?? 0);
            // Oversized inputs retain the unchanged proof without allocating a
            // cache snapshot. Every admitted key has a complete ordered copy.
            if (points > 500_000 || !double.IsFinite(radius)
                || !double.IsFinite(arcTolerance) || !double.IsFinite(coverageError)) return null;
            var words = new long[checked((int)(9 + rings.Length + (clip is null ? 0 : 1) + 2 * points))];
            int next = 0;
            int copied = 0;
            words[next++] = 1; // Exact-effect schema; proof constants below are part of the key.
            words[next++] = BitConverter.DoubleToInt64Bits(PathUtils.Scale);
            words[next++] = BitConverter.DoubleToInt64Bits(radius);
            words[next++] = BitConverter.DoubleToInt64Bits(arcTolerance);
            words[next++] = BitConverter.DoubleToInt64Bits(coverageError);
            words[next++] = modes;
            words[next++] = rings.Length;
            words[next++] = clip is null ? 0 : 1;
            words[next++] = clip?.Count ?? 0;
            bool finite = true;
            void Copy(IReadOnlyList<PointD> ring)
            {
                words[next++] = ring.Count;
                foreach (var point in ring)
                {
                    if ((copied++ & 255) == 0) token.ThrowIfCancellationRequested();
                    finite &= double.IsFinite(point.x) && double.IsFinite(point.y);
                    words[next++] = BitConverter.DoubleToInt64Bits(point.x);
                    words[next++] = BitConverter.DoubleToInt64Bits(point.y);
                }
            }
            foreach (var ring in rings) Copy(ring);
            if (clip is not null) Copy(clip);
            token.ThrowIfCancellationRequested();
            return finite ? new Key(words, (int)points, token) : null;
        }
    }

    internal readonly record struct State(int Entries, long Bytes, long Hits, long Misses,
        long Publications, long Evictions, long Oversized);

    internal sealed class Cache<T> where T : class
    {
        private sealed record Entry(Key Key, T Value, long Bytes);
        private readonly object _sync = new();
        private readonly LinkedList<Entry> _entries = new();
        private readonly long _byteLimit;
        private readonly int _entryLimit;
        private long _bytes, _hits, _misses, _publications, _evictions, _oversized;

        internal Cache(long byteLimit, int entryLimit)
        { _byteLimit = byteLimit; _entryLimit = entryLimit; }

        internal T? Find(Key? key, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            if (key is null) return null;
            lock (_sync)
            {
                token.ThrowIfCancellationRequested();
                for (var node = _entries.First; node is not null; node = node.Next)
                {
                    if (!node.Value.Key.Same(key)) continue;
                    _entries.Remove(node); _entries.AddFirst(node);
                    _hits++;
                    return node.Value.Value;
                }
                _misses++;
                return null;
            }
        }

        /// <summary>Transfers one complete value to a sole mutable consumer.</summary>
        internal T? Take(Key? key,CancellationToken token)
        {
            token.ThrowIfCancellationRequested();if(key is null)return null;
            lock(_sync)
            {
                token.ThrowIfCancellationRequested();
                for(var node=_entries.First;node is not null;node=node.Next)
                {
                    if(!node.Value.Key.Same(key))continue;
                    _entries.Remove(node);_bytes-=node.Value.Bytes;_hits++;
                    return node.Value.Value;
                }
                _misses++;return null;
            }
        }

        internal void Publish(Key? key, T value, long valueBytes, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();
            if (key is null) return;
            long bytes = checked(key.Bytes + valueBytes + 96);
            lock (_sync)
            {
                token.ThrowIfCancellationRequested();
                if (bytes > _byteLimit) { _oversized++; return; }
                for (var node = _entries.First; node is not null; node = node.Next)
                    if (node.Value.Key.Same(key)) return;
                while (_entries.Count >= _entryLimit || _bytes + bytes > _byteLimit)
                {
                    _bytes -= _entries.Last!.Value.Bytes;
                    _entries.RemoveLast(); _evictions++;
                }
                _entries.AddFirst(new Entry(key, value, bytes));
                _bytes += bytes; _publications++;
            }
        }

        internal State Inspect()
        {
            lock (_sync) return new State(_entries.Count, _bytes, _hits, _misses,
                _publications, _evictions, _oversized);
        }

        internal void Clear()
        {
            lock (_sync)
            {
                _entries.Clear(); _bytes = _hits = _misses = _publications = _evictions = _oversized = 0;
            }
        }
    }
}
