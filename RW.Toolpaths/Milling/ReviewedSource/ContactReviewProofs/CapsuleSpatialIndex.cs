namespace RW.Toolpaths.Milling.ReviewedSource.ContactReviewProofs;
/// <summary>Exact closed-box candidate index only. It changes neither the
/// stock capsules, their posting reserve nor any containment certificate.</summary>
internal sealed class CapsuleSpatialIndex
{
 readonly record struct Box(double L,double B,double R,double T)
 {
  internal bool Intersects(Box q)=>L<=q.R&&R>=q.L&&B<=q.T&&T>=q.B;
  internal static Box Of(AnalyticCapsuleCover.Capsule c)=>new(Math.Min(c.A.X,c.B.X)-c.Radius,Math.Min(c.A.Y,c.B.Y)-c.Radius,Math.Max(c.A.X,c.B.X)+c.Radius,Math.Max(c.A.Y,c.B.Y)+c.Radius);
 }
 sealed record Node(Box Bounds,Node? Left,Node? Right,AnalyticCapsuleCover.Capsule[] Items);
 readonly Node? root;
 internal CapsuleSpatialIndex(IEnumerable<AnalyticCapsuleCover.Capsule> capsules)
 {
  Node? Build(AnalyticCapsuleCover.Capsule[] a)
  {
   if(a.Length==0)return null;
   var boxes=a.Select(Box.Of).ToArray();var box=new Box(boxes.Min(b=>b.L),boxes.Min(b=>b.B),boxes.Max(b=>b.R),boxes.Max(b=>b.T));
   if(a.Length<=16)return new(box,null,null,a);
   bool x=box.R-box.L>=box.T-box.B;
   Array.Sort(a,(p,q)=>(x?p.A.X+p.B.X:p.A.Y+p.B.Y).CompareTo(x?q.A.X+q.B.X:q.A.Y+q.B.Y));
   int half=a.Length/2;return new(box,Build(a[..half]),Build(a[half..]),[]);
  }
  root=Build(capsules.ToArray());
 }
 internal AnalyticCapsuleCover.Capsule[] Query(double left,double bottom,double right,double top)
 {
  var box=new Box(left,bottom,right,top);var found=new List<AnalyticCapsuleCover.Capsule>();
  void Visit(Node? node)
  {
   if(node is null||!node.Bounds.Intersects(box))return;
   foreach(var c in node.Items)if(Box.Of(c).Intersects(box))found.Add(c);
   Visit(node.Left);Visit(node.Right);
  }
  Visit(root);return found.OrderBy(c=>c.ActualStockChord).ToArray();
 }
}
