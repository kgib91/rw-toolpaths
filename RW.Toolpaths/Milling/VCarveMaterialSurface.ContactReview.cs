using Clipper2Lib;
namespace RW.Toolpaths.Milling;
internal sealed partial class VCarveMaterialSurface
{
 internal sealed record ContactReviewBand(double Top,double Bottom,IReadOnlyList<IReadOnlyList<PointD>> Rings);
 internal IReadOnlyList<ContactReviewBand> ContactReviewBands()=>layers.Select((b,i)=>new ContactReviewBand(b.TopDepth,i+1<layers.Count?layers[i+1].TopDepth:maximumDepth,b.FreeRegion)).ToArray();
}
