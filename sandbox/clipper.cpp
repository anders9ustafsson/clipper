/*******************************************************************************
*                                                                              *
* Author    :  Angus Johnson                                                   *
* Version   :  6.1.3 (float) - Experimental                                    *
* Date      :  11 January 2014                                                 *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2014                                         *
*                                                                              *
* License:                                                                     *
* Use, modification & distribution is subject to Boost Software License Ver 1. *
* http://www.boost.org/LICENSE_1_0.txt                                         *
*                                                                              *
* Attributions:                                                                *
* The code in this library is an extension of Bala Vatti's clipping algorithm: *
* "A generic solution to polygon clipping"                                     *
* Communications of the ACM, Vol 35, Issue 7 (July 1992) pp 56-63.             *
* http://portal.acm.org/citation.cfm?id=129906                                 *
*                                                                              *
* Computer graphics and geometric modeling: implementation and algorithms      *
* By Max K. Agoston                                                            *
* Springer; 1 edition (January 4, 2005)                                        *
* http://books.google.com/books?q=vatti+clipping+agoston                       *
*                                                                              *
* See also:                                                                    *
* "Polygon Offsetting by Computing Winding Numbers"                            *
* Paper no. DETC2005-85513 pp. 565-575                                         *
* ASME 2005 International Design Engineering Technical Conferences             *
* and Computers and Information in Engineering Conference (IDETC/CIE2005)      *
* September 24-28, 2005 , Long Beach, California, USA                          *
* http://www.me.berkeley.edu/~mcmains/pubs/DAC05OffsetPolygon.pdf              *
*                                                                              *
*******************************************************************************/

/*******************************************************************************
*                                                                              *
* This is a translation of the Delphi Clipper library and the naming style     *
* used has retained a Delphi flavour.                                          *
*                                                                              *
*******************************************************************************/

#include "clipper.hpp"
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cstring>
#include <cstdlib>
#include <ostream>
#include <functional>

namespace ClipperLib {

typedef signed long long long64;
typedef unsigned long long ulong64;

static double const pi = 3.141592653589793238;
static double const two_pi = pi *2;
static double const def_arc_tolerance = 0.25;

enum Direction { dRightToLeft, dLeftToRight };

static int const Unassigned = -1;  //edge not currently 'owning' a solution
static int const Skip = -2;        //edge that would otherwise close a path

#define HORIZONTAL (-1.0E+40)

struct TEdge {
  FPoint Bot;
  FPoint Curr;
  FPoint Top;
  FPoint Delta;
  double Dx;
  PolyType PolyTyp;
  EdgeSide Side;
  int WindDelta; //1 or -1 depending on winding direction
  int WindCnt;
  int WindCnt2; //winding count of the opposite polytype
  int OutIdx;
  TEdge *Next;
  TEdge *Prev;
  TEdge *NextInLML;
  TEdge *NextInAEL;
  TEdge *PrevInAEL;
  TEdge *NextInSEL;
  TEdge *PrevInSEL;
};

struct IntersectNode {
  TEdge          *Edge1;
  TEdge          *Edge2;
  FPoint        Pt;
};

struct LocalMinima {
  double          Y;
  TEdge        *LeftBound;
  TEdge        *RightBound;
  LocalMinima  *Next;
};

struct OutPt;

struct OutRec {
  int       Idx;
  bool      IsHole;
  bool      IsOpen;
  OutRec   *FirstLeft;  //see comments in clipper.pas
  PolyNode *PolyNd;
  OutPt    *Pts;
  OutPt    *BottomPt;
};

struct OutPt {
  int       Idx;
  FPoint  Pt;
  OutPt    *Next;
  OutPt    *Prev;
};

struct Join {
  OutPt    *OutPt1;
  OutPt    *OutPt2;
  FPoint  OffPt;
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline double Round(double val)
{
  if ((val < 0)) return static_cast<double>(val - 0.5); 
  else return static_cast<double>(val + 0.5);
}
//------------------------------------------------------------------------------

inline double Abs(double val)
{
  return val < 0 ? -val : val;
}

//------------------------------------------------------------------------------
// PolyTree methods ...
//------------------------------------------------------------------------------

void PolyTree::Clear()
{
    for (PolyNodes::size_type i = 0; i < AllNodes.size(); ++i)
      delete AllNodes[i];
    AllNodes.resize(0); 
    Childs.resize(0);
}
//------------------------------------------------------------------------------

PolyNode* PolyTree::GetFirst() const
{
  if (!Childs.empty())
      return Childs[0];
  else
      return 0;
}
//------------------------------------------------------------------------------

int PolyTree::Total() const
{
  return (int)AllNodes.size();
}

//------------------------------------------------------------------------------
// PolyNode methods ...
//------------------------------------------------------------------------------

PolyNode::PolyNode(): Childs(), Parent(0), Index(0), m_IsOpen(false)
{
}
//------------------------------------------------------------------------------

int PolyNode::ChildCount() const
{
  return (int)Childs.size();
}
//------------------------------------------------------------------------------

void PolyNode::AddChild(PolyNode& child)
{
  unsigned cnt = (unsigned)Childs.size();
  Childs.push_back(&child);
  child.Parent = this;
  child.Index = cnt;
}
//------------------------------------------------------------------------------

PolyNode* PolyNode::GetNext() const
{ 
  if (!Childs.empty()) 
      return Childs[0]; 
  else
      return GetNextSiblingUp();    
}  
//------------------------------------------------------------------------------

PolyNode* PolyNode::GetNextSiblingUp() const
{ 
  if (!Parent) //protects against PolyTree.GetNextSiblingUp()
      return 0;
  else if (Index == Parent->Childs.size() - 1)
      return Parent->GetNextSiblingUp();
  else
      return Parent->Childs[Index + 1];
}  
//------------------------------------------------------------------------------

bool PolyNode::IsHole() const
{ 
  bool result = true;
  PolyNode* node = Parent;
  while (node)
  {
      result = !result;
      node = node->Parent;
  }
  return result;
}  
//------------------------------------------------------------------------------

bool PolyNode::IsOpen() const
{ 
  return m_IsOpen;
}  

//------------------------------------------------------------------------------
// Miscellaneous global functions
//------------------------------------------------------------------------------

bool Orientation(const Path &poly)
{
    return Area(poly) >= 0;
}
//------------------------------------------------------------------------------

double Area(const Path &poly)
{
  int size = (int)poly.size();
  if (size < 3) return 0;

  double a = 0;
  for (int i = 0, j = size -1; i < size; ++i)
  {
    a += (poly[j].X + poly[i].X) * (poly[j].Y - poly[i].Y);
    j = i;
  }
  return -a * 0.5;
}
//------------------------------------------------------------------------------

double Area(const OutRec &outRec)
{
  OutPt *op = outRec.Pts;
  if (!op) return 0;
  double a = 0;
  do {
    a +=  (op->Prev->Pt.X + op->Pt.X) * (op->Prev->Pt.Y - op->Pt.Y);
    op = op->Next;
  } while (op != outRec.Pts);
  return a * 0.5;
}
//------------------------------------------------------------------------------

bool PointIsVertex(const FPoint &Pt, OutPt *pp)
{
  OutPt *pp2 = pp;
  do
  {
    if (pp2->Pt == Pt) return true;
    pp2 = pp2->Next;
  }
  while (pp2 != pp);
  return false;
}
//------------------------------------------------------------------------------

int PointInPolygon (const FPoint& pt, OutPt* op)
{
  //returns 0 if false, +1 if true, -1 if pt ON polygon boundary
  //http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.88.5498&rep=rep1&type=pdf
  int result = 0;
  OutPt* startOp = op;
  for(;;)
  {
    if (op->Next->Pt.Y == pt.Y)
    {
        if ((op->Next->Pt.X == pt.X) || (op->Pt.Y == pt.Y && 
          ((op->Next->Pt.X > pt.X) == (op->Pt.X < pt.X)))) return -1;
    }
    if ((op->Pt.Y < pt.Y) != (op->Next->Pt.Y < pt.Y))
    {
      if (op->Pt.X >= pt.X)
      {
        if (op->Next->Pt.X > pt.X) result = 1 - result;
        else
        {
          double d = (op->Pt.X - pt.X) * (op->Next->Pt.Y - pt.Y) - 
            (op->Next->Pt.X - pt.X) * (op->Pt.Y - pt.Y);
          if (!d) return -1;
          if ((d > 0) == (op->Next->Pt.Y > op->Pt.Y)) result = 1 - result;
        }
      } else
      {
        if (op->Next->Pt.X > pt.X)
        {
          double d = (op->Pt.X - pt.X) * (op->Next->Pt.Y - pt.Y) - 
            (op->Next->Pt.X - pt.X) * (op->Pt.Y - pt.Y);
          if (!d) return -1;
          if ((d > 0) == (op->Next->Pt.Y > op->Pt.Y)) result = 1 - result;
        }
      }
    } 
    op = op->Next;
    if (startOp == op) break;
  } 
  return result;
}
//------------------------------------------------------------------------------

bool Poly2ContainsPoly1(OutPt* OutPt1, OutPt* OutPt2)
{
  OutPt* op = OutPt1;
  do
  {
    int res = PointInPolygon(op->Pt, OutPt2);
    if (res >= 0) return res != 0;
    op = op->Next; 
  }
  while (op != OutPt1);
  return true; 
}
//----------------------------------------------------------------------

bool IsAlmostEqual(double A, double B)
{
    //http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
    long long aInt = reinterpret_cast<long long&>(A);
    if (aInt < 0) aInt = -9223372036854775808LL - aInt;
    long long bInt = reinterpret_cast<long long&>(B);
    if (bInt < 0) bInt = -9223372036854775808LL - bInt;
    return (std::abs(aInt - bInt) <= 10000000000);
}
//----------------------------------------------------------------------

bool operator== (const FPoint& a, const FPoint& b)
{
  return IsAlmostEqual(a.X, b.X) && IsAlmostEqual(a.Y, b.Y);
}
//----------------------------------------------------------------------

bool operator!= (const FPoint& a, const FPoint& b)
{
  return !IsAlmostEqual(a.X, b.X) || !IsAlmostEqual(a.Y, b.Y);
}
//----------------------------------------------------------------------

bool SlopesEqual(const TEdge &e1, const TEdge &e2)
{
  return IsAlmostEqual(e1.Delta.Y * e2.Delta.X, e1.Delta.X * e2.Delta.Y);
}
//------------------------------------------------------------------------------

bool SlopesEqual(const FPoint pt1, const FPoint pt2, const FPoint pt3)
{
  return IsAlmostEqual((pt1.Y-pt2.Y)*(pt2.X-pt3.X), (pt1.X-pt2.X)*(pt2.Y-pt3.Y));
}
//------------------------------------------------------------------------------

bool SlopesEqual(const FPoint pt1, const FPoint pt2,
  const FPoint pt3, const FPoint pt4)
{
  return IsAlmostEqual((pt1.Y-pt2.Y)*(pt3.X-pt4.X), (pt1.X-pt2.X)*(pt3.Y-pt4.Y));
}
//------------------------------------------------------------------------------

inline bool IsHorizontal(TEdge &e)
{
  return e.Dx == HORIZONTAL;
}
//------------------------------------------------------------------------------

inline double GetDx(const FPoint pt1, const FPoint pt2)
{
  return (pt1.Y == pt2.Y) ?
    HORIZONTAL : (pt2.X - pt1.X) / (pt2.Y - pt1.Y);
}
//---------------------------------------------------------------------------

inline void SetDx(TEdge &e)
{
  e.Delta.X = (e.Top.X - e.Bot.X);
  e.Delta.Y = (e.Top.Y - e.Bot.Y);

  if (e.Delta.Y == 0) e.Dx = HORIZONTAL;
  else e.Dx = e.Delta.X / e.Delta.Y;
}
//---------------------------------------------------------------------------

inline void SwapSides(TEdge &Edge1, TEdge &Edge2)
{
  EdgeSide Side =  Edge1.Side;
  Edge1.Side = Edge2.Side;
  Edge2.Side = Side;
}
//------------------------------------------------------------------------------

inline void SwapPolyIndexes(TEdge &Edge1, TEdge &Edge2)
{
  int OutIdx =  Edge1.OutIdx;
  Edge1.OutIdx = Edge2.OutIdx;
  Edge2.OutIdx = OutIdx;
}
//------------------------------------------------------------------------------

inline double TopX(TEdge &edge, const double currentY)
{
  return ( currentY == edge.Top.Y ) ?
    edge.Top.X : edge.Bot.X + edge.Dx *(currentY - edge.Bot.Y);
}
//------------------------------------------------------------------------------

bool IntersectPoint(TEdge &Edge1, TEdge &Edge2,
  FPoint &ip)
{
#ifdef use_xyz  
  ip.Z = 0;
#endif
  double b1, b2;
  //nb: with very large coordinate values, it's possible for SlopesEqual() to 
  //return false but for the edge.Dx value be equal due to double precision rounding.
  if (SlopesEqual(Edge1, Edge2) || Edge1.Dx == Edge2.Dx)
  {
    if (Edge2.Bot.Y > Edge1.Bot.Y) ip = Edge2.Bot;
    else ip = Edge1.Bot;
    return false;
  }
  else if (Edge1.Delta.X == 0)
  {
    ip.X = Edge1.Bot.X;
    if (IsHorizontal(Edge2))
      ip.Y = Edge2.Bot.Y;
    else
    {
      b2 = Edge2.Bot.Y - (Edge2.Bot.X / Edge2.Dx);
      ip.Y = ip.X / Edge2.Dx + b2;
    }
  }
  else if (Edge2.Delta.X == 0)
  {
    ip.X = Edge2.Bot.X;
    if (IsHorizontal(Edge1))
      ip.Y = Edge1.Bot.Y;
    else
    {
      b1 = Edge1.Bot.Y - (Edge1.Bot.X / Edge1.Dx);
      ip.Y = ip.X / Edge1.Dx + b1;
    }
  } 
  else 
  {
    b1 = Edge1.Bot.X - Edge1.Bot.Y * Edge1.Dx;
    b2 = Edge2.Bot.X - Edge2.Bot.Y * Edge2.Dx;
    double q = (b2-b1) / (Edge1.Dx - Edge2.Dx);
    ip.Y = q;
    if (std::fabs(Edge1.Dx) < std::fabs(Edge2.Dx))
      ip.X = Edge1.Dx * q + b1;
    else 
      ip.X = Edge2.Dx * q + b2;
  }

  if (ip.Y < Edge1.Top.Y || ip.Y < Edge2.Top.Y) 
  {
    if (Edge1.Top.Y > Edge2.Top.Y)
      ip.Y = Edge1.Top.Y;
    else
      ip.Y = Edge2.Top.Y;
    if (std::fabs(Edge1.Dx) < std::fabs(Edge2.Dx))
      ip.X = TopX(Edge1, ip.Y);
    else
      ip.X = TopX(Edge2, ip.Y);
  } 
  return true;
}
//------------------------------------------------------------------------------

void ReversePolyPtLinks(OutPt *pp)
{
  if (!pp) return;
  OutPt *pp1, *pp2;
  pp1 = pp;
  do {
  pp2 = pp1->Next;
  pp1->Next = pp1->Prev;
  pp1->Prev = pp2;
  pp1 = pp2;
  } while( pp1 != pp );
}
//------------------------------------------------------------------------------

void DisposeOutPts(OutPt*& pp)
{
  if (pp == 0) return;
    pp->Prev->Next = 0;
  while( pp )
  {
    OutPt *tmpPp = pp;
    pp = pp->Next;
    delete tmpPp;
  }
}
//------------------------------------------------------------------------------

inline void InitEdge(TEdge* e, TEdge* eNext, TEdge* ePrev, const FPoint& Pt)
{
  std::memset(e, 0, sizeof(TEdge));
  e->Next = eNext;
  e->Prev = ePrev;
  e->Curr = Pt;
  e->OutIdx = Unassigned;
}
//------------------------------------------------------------------------------

void InitEdge2(TEdge& e, PolyType Pt)
{
  if (e.Curr.Y >= e.Next->Curr.Y)
  {
    e.Bot = e.Curr;
    e.Top = e.Next->Curr;
  } else
  {
    e.Top = e.Curr;
    e.Bot = e.Next->Curr;
  }
  SetDx(e);
  e.PolyTyp = Pt;
}
//------------------------------------------------------------------------------

TEdge* RemoveEdge(TEdge* e)
{
  //removes e from double_linked_list (but without removing from memory)
  e->Prev->Next = e->Next;
  e->Next->Prev = e->Prev;
  TEdge* result = e->Next;
  e->Prev = 0; //flag as removed (see ClipperBase.Clear)
  return result;
}
//------------------------------------------------------------------------------

inline void ReverseHorizontal(TEdge &e)
{
  //swap horizontal edges' Top and Bottom x's so they follow the natural
  //progression of the bounds - ie so their xbots will align with the
  //adjoining lower edge. [Helpful in the ProcessHorizontal() method.]
  double tmp = e.Top.X;
  e.Top.X = e.Bot.X;
  e.Bot.X = tmp;
#ifdef use_xyz  
  long64 tmp2 = e.Top.Z;
  e.Top.Z = e.Bot.Z;
  e.Bot.Z = tmp2;
#endif
}
//------------------------------------------------------------------------------

void SwapPoints(FPoint &pt1, FPoint &pt2)
{
  FPoint tmp = pt1;
  pt1 = pt2;
  pt2 = tmp;
}
//------------------------------------------------------------------------------

bool GetOverlapSegment(FPoint pt1a, FPoint pt1b, FPoint pt2a,
  FPoint pt2b, FPoint &pt1, FPoint &pt2)
{
  //precondition: segments are Collinear.
  if (Abs(pt1a.X - pt1b.X) > Abs(pt1a.Y - pt1b.Y))
  {
    if (pt1a.X > pt1b.X) SwapPoints(pt1a, pt1b);
    if (pt2a.X > pt2b.X) SwapPoints(pt2a, pt2b);
    if (pt1a.X > pt2a.X) pt1 = pt1a; else pt1 = pt2a;
    if (pt1b.X < pt2b.X) pt2 = pt1b; else pt2 = pt2b;
    return pt1.X < pt2.X;
  } else
  {
    if (pt1a.Y < pt1b.Y) SwapPoints(pt1a, pt1b);
    if (pt2a.Y < pt2b.Y) SwapPoints(pt2a, pt2b);
    if (pt1a.Y < pt2a.Y) pt1 = pt1a; else pt1 = pt2a;
    if (pt1b.Y > pt2b.Y) pt2 = pt1b; else pt2 = pt2b;
    return pt1.Y > pt2.Y;
  }
}
//------------------------------------------------------------------------------

bool FirstIsBottomPt(const OutPt* btmPt1, const OutPt* btmPt2)
{
  OutPt *p = btmPt1->Prev;
  while ((p->Pt == btmPt1->Pt) && (p != btmPt1)) p = p->Prev;
  double dx1p = std::fabs(GetDx(btmPt1->Pt, p->Pt));
  p = btmPt1->Next;
  while ((p->Pt == btmPt1->Pt) && (p != btmPt1)) p = p->Next;
  double dx1n = std::fabs(GetDx(btmPt1->Pt, p->Pt));

  p = btmPt2->Prev;
  while ((p->Pt == btmPt2->Pt) && (p != btmPt2)) p = p->Prev;
  double dx2p = std::fabs(GetDx(btmPt2->Pt, p->Pt));
  p = btmPt2->Next;
  while ((p->Pt == btmPt2->Pt) && (p != btmPt2)) p = p->Next;
  double dx2n = std::fabs(GetDx(btmPt2->Pt, p->Pt));
  return (dx1p >= dx2p && dx1p >= dx2n) || (dx1n >= dx2p && dx1n >= dx2n);
}
//------------------------------------------------------------------------------

OutPt* GetBottomPt(OutPt *pp)
{
  OutPt* dups = 0;
  OutPt* p = pp->Next;
  while (p != pp)
  {
    if (p->Pt.Y > pp->Pt.Y)
    {
      pp = p;
      dups = 0;
    }
    else if (p->Pt.Y == pp->Pt.Y && p->Pt.X <= pp->Pt.X)
    {
      if (p->Pt.X < pp->Pt.X)
      {
        dups = 0;
        pp = p;
      } else
      {
        if (p->Next != pp && p->Prev != pp) dups = p;
      }
    }
    p = p->Next;
  }
  if (dups)
  {
    //there appears to be at least 2 vertices at BottomPt so ...
    while (dups != p)
    {
      if (!FirstIsBottomPt(p, dups)) pp = dups;
      dups = dups->Next;
      while (dups->Pt != pp->Pt) dups = dups->Next;
    }
  }
  return pp;
}
//------------------------------------------------------------------------------

bool FindSegment(OutPt* &pp, FPoint &pt1, FPoint &pt2)
{
  //OutPt1 & OutPt2 => the overlap segment (if the function returns true)
  if (!pp) return false;
  OutPt* pp2 = pp;
  FPoint pt1a = pt1, pt2a = pt2;
  do
  {
    if (SlopesEqual(pt1a, pt2a, pp->Pt, pp->Prev->Pt) &&
      SlopesEqual(pt1a, pt2a, pp->Pt) &&
      GetOverlapSegment(pt1a, pt2a, pp->Pt, pp->Prev->Pt, pt1, pt2))
        return true;
    pp = pp->Next;
  }
  while (pp != pp2);
  return false;
}
//------------------------------------------------------------------------------

bool Pt2IsBetweenPt1AndPt3(const FPoint pt1,
  const FPoint pt2, const FPoint pt3)
{
  if ((pt1 == pt3) || (pt1 == pt2) || (pt3 == pt2))
    return false;
  else if (pt1.X != pt3.X)
    return (pt2.X > pt1.X) == (pt2.X < pt3.X);
  else
    return (pt2.Y > pt1.Y) == (pt2.Y < pt3.Y);
}
//------------------------------------------------------------------------------

OutPt* InsertPolyPtBetween(OutPt* p1, OutPt* p2, const FPoint Pt)
{
  if (p1 == p2) throw "JoinError";
  OutPt* result = new OutPt;
  result->Pt = Pt;
  if (p2 == p1->Next)
  {
    p1->Next = result;
    p2->Prev = result;
    result->Next = p2;
    result->Prev = p1;
  } else
  {
    p2->Next = result;
    p1->Prev = result;
    result->Next = p1;
    result->Prev = p2;
  }
  return result;
}
//------------------------------------------------------------------------------

bool HorzSegmentsOverlap(const FPoint& pt1a, const FPoint& pt1b, 
    const FPoint& pt2a, const FPoint& pt2b)
{
  //precondition: both segments are horizontal
  if ((pt1a.X > pt2a.X) == (pt1a.X < pt2b.X)) return true;
  else if ((pt1b.X > pt2a.X) == (pt1b.X < pt2b.X)) return true;
  else if ((pt2a.X > pt1a.X) == (pt2a.X < pt1b.X)) return true;
  else if ((pt2b.X > pt1a.X) == (pt2b.X < pt1b.X)) return true;
  else if ((pt1a.X == pt2a.X) && (pt1b.X == pt2b.X)) return true;
  else if ((pt1a.X == pt2b.X) && (pt1b.X == pt2a.X)) return true;
  else return false;
}


//------------------------------------------------------------------------------
// ClipperBase class methods ...
//------------------------------------------------------------------------------

ClipperBase::ClipperBase() //constructor
{
  m_MinimaList = 0;
  m_CurrentLM = 0;
}
//------------------------------------------------------------------------------

ClipperBase::~ClipperBase() //destructor
{
  Clear();
}
//------------------------------------------------------------------------------

TEdge* FindNextLocMin(TEdge* E)
{
  for (;;)
  {
    while (E->Bot != E->Prev->Bot || E->Curr == E->Top) E = E->Next;
    if (!IsHorizontal(*E) && !IsHorizontal(*E->Prev)) break;
    while (IsHorizontal(*E->Prev)) E = E->Prev;
    TEdge* E2 = E;
    while (IsHorizontal(*E)) E = E->Next;
    if (E->Top.Y == E->Prev->Bot.Y) continue; //ie just an intermediate horz.
    if (E2->Prev->Bot.X < E->Bot.X) E = E2;
    break;
  }
  return E;
}
//------------------------------------------------------------------------------

TEdge* ClipperBase::ProcessBound(TEdge* E, bool IsClockwise)
{
  TEdge *EStart = E, *Result = E;
  TEdge *Horz = 0;
  double StartX;
  if (IsHorizontal(*E))
  {
    //it's possible for adjacent overlapping horz edges to start heading left
    //before finishing right, so ...
    if (IsClockwise) StartX = E->Prev->Bot.X;
    else StartX = E->Next->Bot.X;
    if (E->Bot.X != StartX) ReverseHorizontal(*E);
  }
  
  if (Result->OutIdx != Skip)
  {
    if (IsClockwise)
    {
      while (Result->Top.Y == Result->Next->Bot.Y && Result->Next->OutIdx != Skip)
        Result = Result->Next;
      if (IsHorizontal(*Result) && Result->Next->OutIdx != Skip)
      {
        //nb: at the top of a bound, horizontals are added to the bound
        //only when the preceding edge attaches to the horizontal's left vertex
        //unless a Skip edge is encountered when that becomes the top divide
        Horz = Result;
        while (IsHorizontal(*Horz->Prev)) Horz = Horz->Prev;
        if (Horz->Prev->Top.X == Result->Next->Top.X) 
        {
          if (!IsClockwise) Result = Horz->Prev;
        }
        else if (Horz->Prev->Top.X > Result->Next->Top.X) Result = Horz->Prev;
      }
      while (E != Result) 
      {
        E->NextInLML = E->Next;
        if (IsHorizontal(*E) && E != EStart &&
          E->Bot.X != E->Prev->Top.X) ReverseHorizontal(*E);
        E = E->Next;
      }
      if (IsHorizontal(*E) && E != EStart && E->Bot.X != E->Prev->Top.X) 
        ReverseHorizontal(*E);
      Result = Result->Next; //move to the edge just beyond current bound
    } else
    {
      while (Result->Top.Y == Result->Prev->Bot.Y && Result->Prev->OutIdx != Skip) 
        Result = Result->Prev;
      if (IsHorizontal(*Result) && Result->Prev->OutIdx != Skip)
      {
        Horz = Result;
        while (IsHorizontal(*Horz->Next)) Horz = Horz->Next;
        if (Horz->Next->Top.X == Result->Prev->Top.X) 
        {
          if (!IsClockwise) Result = Horz->Next;
        }
        else if (Horz->Next->Top.X > Result->Prev->Top.X) Result = Horz->Next;
      }

      while (E != Result)
      {
        E->NextInLML = E->Prev;
        if (IsHorizontal(*E) && E != EStart && E->Bot.X != E->Next->Top.X) 
          ReverseHorizontal(*E);
        E = E->Prev;
      }
      if (IsHorizontal(*E) && E != EStart && E->Bot.X != E->Next->Top.X) 
        ReverseHorizontal(*E);
      Result = Result->Prev; //move to the edge just beyond current bound
    }
  }

  if (Result->OutIdx == Skip) 
  {
    //if edges still remain in the current bound beyond the skip edge then
    //create another LocMin and call ProcessBound once more
    E = Result;
    if (IsClockwise)
    {
      while (E->Top.Y == E->Next->Bot.Y) E = E->Next;
      //don't include top horizontals when parsing a bound a second time,
      //they will be contained in the opposite bound ...
      while (E != Result && IsHorizontal(*E)) E = E->Prev;
    } else
    {
      while (E->Top.Y == E->Prev->Bot.Y) E = E->Prev;
      while (E != Result && IsHorizontal(*E)) E = E->Next;
    }
    if (E == Result)
    {
      if (IsClockwise) Result = E->Next;
      else Result = E->Prev;
    } else
    {
      //there are more edges in the bound beyond result starting with E
      if (IsClockwise)
        E = Result->Next; 
      else
        E = Result->Prev;
      LocalMinima* locMin = new LocalMinima;
      locMin->Next = 0;
      locMin->Y = E->Bot.Y;
      locMin->LeftBound = 0;
      locMin->RightBound = E;
      locMin->RightBound->WindDelta = 0;
      Result = ProcessBound(locMin->RightBound, IsClockwise);
      InsertLocalMinima(locMin);
    }
  }
  return Result;
}
//------------------------------------------------------------------------------

bool ClipperBase::AddPath(const Path &pg, PolyType PolyTyp, bool Closed)
{
#ifdef use_lines
  if (!Closed && PolyTyp == ptClip)
    throw clipperException("AddPath: Open paths must be subject.");
#else
  if (!Closed)
    throw clipperException("AddPath: Open paths have been disabled.");
#endif

  int highI = (int)pg.size() -1;
  if (Closed) while (highI > 0 && (pg[highI] == pg[0])) --highI;
  while (highI > 0 && (pg[highI] == pg[highI -1])) --highI;
  if ((Closed && highI < 2) || (!Closed && highI < 1)) return false;

  //create a new edge array ...
  TEdge *edges = new TEdge [highI +1];

  bool IsFlat = true;
  //1. Basic (first) edge initialization ...
  try
  {
    edges[1].Curr = pg[1];
    InitEdge(&edges[0], &edges[1], &edges[highI], pg[0]);
    InitEdge(&edges[highI], &edges[0], &edges[highI-1], pg[highI]);
    for (int i = highI - 1; i >= 1; --i)
      InitEdge(&edges[i], &edges[i+1], &edges[i-1], pg[i]);
  }
  catch(...)
  {
    delete [] edges;
    throw;
  }
  TEdge *eStart = &edges[0];

  //2. Remove duplicate vertices, and (when closed) collinear edges ...
  TEdge *E = eStart, *eLoopStop = eStart;
  for (;;)
  {
    if ((E->Curr == E->Next->Curr))
    {
      if (E == E->Next) break;
      if (E == eStart) eStart = E->Next;
      E = RemoveEdge(E);
      eLoopStop = E;
      continue;
    }
    if (E->Prev == E->Next) 
      break; //only two vertices
    else if (Closed &&
      SlopesEqual(E->Prev->Curr, E->Curr, E->Next->Curr) && 
      (!m_PreserveCollinear ||
      !Pt2IsBetweenPt1AndPt3(E->Prev->Curr, E->Curr, E->Next->Curr)))
    {
      //Collinear edges are allowed for open paths but in closed paths
      //the default is to merge adjacent collinear edges into a single edge.
      //However, if the PreserveCollinear property is enabled, only overlapping
      //collinear edges (ie spikes) will be removed from closed paths.
      if (E == eStart) eStart = E->Next;
      E = RemoveEdge(E);
      E = E->Prev;
      eLoopStop = E;
      continue;
    }
    E = E->Next;
    if (E == eLoopStop) break;
  }

  if ((!Closed && (E == E->Next)) || (Closed && (E->Prev == E->Next)))
  {
    delete [] edges;
    return false;
  }

  if (!Closed)
  { 
    m_HasOpenPaths = true;
    eStart->Prev->OutIdx = Skip;
  }

  //3. Do second stage of edge initialization ...
  E = eStart;
  do
  {
    InitEdge2(*E, PolyTyp);
    E = E->Next;
    if (IsFlat && E->Curr.Y != eStart->Curr.Y) IsFlat = false;
  }
  while (E != eStart);

  //4. Finally, add edge bounds to LocalMinima list ...

  //Totally flat paths must be handled differently when adding them
  //to LocalMinima list to avoid endless loops etc ...
  if (IsFlat) 
  {
    if (Closed) 
    {
      delete [] edges;
      return false;
    }
    E->Prev->OutIdx = Skip;
    if (E->Prev->Bot.X < E->Prev->Top.X) ReverseHorizontal(*E->Prev);
    LocalMinima* locMin = new LocalMinima();
    locMin->Next = 0;
    locMin->Y = E->Bot.Y;
    locMin->LeftBound = 0;
    locMin->RightBound = E;
    locMin->RightBound->Side = esRight;
    locMin->RightBound->WindDelta = 0;
    while (E->Next->OutIdx != Skip)
    {
      E->NextInLML = E->Next;
      if (E->Bot.X != E->Prev->Top.X) ReverseHorizontal(*E);
      E = E->Next;
    }
    InsertLocalMinima(locMin);
    m_edges.push_back(edges);
	  return true;
  }

  m_edges.push_back(edges);
  bool clockwise;
  TEdge* EMin = 0;
  for (;;)
  {
    E = FindNextLocMin(E);
    if (E == EMin) break;
    else if (!EMin) EMin = E;

    //E and E.Prev now share a local minima (left aligned if horizontal).
    //Compare their slopes to find which starts which bound ...
    LocalMinima* locMin = new LocalMinima;
    locMin->Next = 0;
    locMin->Y = E->Bot.Y;
    if (E->Dx < E->Prev->Dx) 
    {
      locMin->LeftBound = E->Prev;
      locMin->RightBound = E;
      clockwise = false; //Q.nextInLML = Q.prev
    } else
    {
      locMin->LeftBound = E;
      locMin->RightBound = E->Prev;
      clockwise = true; //Q.nextInLML = Q.next
    }
    locMin->LeftBound->Side = esLeft;
    locMin->RightBound->Side = esRight;

    if (!Closed) locMin->LeftBound->WindDelta = 0;
    else if (locMin->LeftBound->Next == locMin->RightBound)
      locMin->LeftBound->WindDelta = -1;
    else locMin->LeftBound->WindDelta = 1;
    locMin->RightBound->WindDelta = -locMin->LeftBound->WindDelta;

    E = ProcessBound(locMin->LeftBound, clockwise);
    TEdge* E2 = ProcessBound(locMin->RightBound, !clockwise);

    if (locMin->LeftBound->OutIdx == Skip)
      locMin->LeftBound = 0;
    else if (locMin->RightBound->OutIdx == Skip)
      locMin->RightBound = 0;
    InsertLocalMinima(locMin);
    if (!clockwise) E = E2;
  }
  return true;
}
//------------------------------------------------------------------------------

bool ClipperBase::AddPaths(const Paths &ppg, PolyType PolyTyp, bool Closed)
{
  bool result = false;
  for (Paths::size_type i = 0; i < ppg.size(); ++i)
    if (AddPath(ppg[i], PolyTyp, Closed)) result = true;
  return result;
}
//------------------------------------------------------------------------------

void ClipperBase::InsertLocalMinima(LocalMinima *newLm)
{
  if( ! m_MinimaList )
  {
    m_MinimaList = newLm;
  }
  else if( newLm->Y >= m_MinimaList->Y )
  {
    newLm->Next = m_MinimaList;
    m_MinimaList = newLm;
  } else
  {
    LocalMinima* tmpLm = m_MinimaList;
    while( tmpLm->Next  && ( newLm->Y < tmpLm->Next->Y ) )
      tmpLm = tmpLm->Next;
    newLm->Next = tmpLm->Next;
    tmpLm->Next = newLm;
  }
}
//------------------------------------------------------------------------------

void ClipperBase::Clear()
{
  DisposeLocalMinimaList();
  for (EdgeList::size_type i = 0; i < m_edges.size(); ++i)
  {
    //for each edge array in turn, find the first used edge and 
    //check for and remove any hiddenPts in each edge in the array.
    TEdge* edges = m_edges[i];
    delete [] edges;
  }
  m_edges.clear();
  m_HasOpenPaths = false;
}
//------------------------------------------------------------------------------

void ClipperBase::Reset()
{
  m_CurrentLM = m_MinimaList;
  if( !m_CurrentLM ) return; //ie nothing to process

  //reset all edges ...
  LocalMinima* lm = m_MinimaList;
  while( lm )
  {
    TEdge* e = lm->LeftBound;
    if (e)
    {
      e->Curr = e->Bot;
      e->Side = esLeft;
      e->OutIdx = Unassigned;
    }

    e = lm->RightBound;
    if (e)
    {
      e->Curr = e->Bot;
      e->Side = esRight;
      e->OutIdx = Unassigned;
    }
    lm = lm->Next;
  }
}
//------------------------------------------------------------------------------

void ClipperBase::DisposeLocalMinimaList()
{
  while( m_MinimaList )
  {
    LocalMinima* tmpLm = m_MinimaList->Next;
    delete m_MinimaList;
    m_MinimaList = tmpLm;
  }
  m_CurrentLM = 0;
}
//------------------------------------------------------------------------------

void ClipperBase::PopLocalMinima()
{
  if( ! m_CurrentLM ) return;
  m_CurrentLM = m_CurrentLM->Next;
}
//------------------------------------------------------------------------------

FRect ClipperBase::GetBounds()
{
  FRect result;
  LocalMinima* lm = m_MinimaList;
  if (!lm)
  {
    result.left = result.top = result.right = result.bottom = 0;
    return result;
  }
  result.left = lm->LeftBound->Bot.X;
  result.top = lm->LeftBound->Bot.Y;
  result.right = lm->LeftBound->Bot.X;
  result.bottom = lm->LeftBound->Bot.Y;
  while (lm)
  {
    if (lm->LeftBound->Bot.Y > result.bottom)
      result.bottom = lm->LeftBound->Bot.Y;
    TEdge* e = lm->LeftBound;
    for (;;) {
      TEdge* bottomE = e;
      while (e->NextInLML)
      {
        if (e->Bot.X < result.left) result.left = e->Bot.X;
        if (e->Bot.X > result.right) result.right = e->Bot.X;
        e = e->NextInLML;
      }
      if (e->Bot.X < result.left) result.left = e->Bot.X;
      if (e->Bot.X > result.right) result.right = e->Bot.X;
      if (e->Top.X < result.left) result.left = e->Top.X;
      if (e->Top.X > result.right) result.right = e->Top.X;
      if (e->Top.Y < result.top) result.top = e->Top.Y;

      if (bottomE == lm->LeftBound) e = lm->RightBound;
      else break;
    }
    lm = lm->Next;
  }
  return result;
}

//------------------------------------------------------------------------------
// TClipper methods ...
//------------------------------------------------------------------------------

Clipper::Clipper(int initOptions) : ClipperBase() //constructor
{
  m_ActiveEdges = 0;
  m_SortedEdges = 0;
  m_ExecuteLocked = false;
  m_ReverseOutput = ((initOptions & ioReverseSolution) != 0);
  m_StrictSimple = ((initOptions & ioStrictlySimple) != 0);
  m_PreserveCollinear = ((initOptions & ioPreserveCollinear) != 0);
  m_HasOpenPaths = false;
#ifdef use_xyz  
  m_ZFill = 0;
#endif
}
//------------------------------------------------------------------------------

Clipper::~Clipper() //destructor
{
  Clear();
  m_Scanbeam.clear();
}
//------------------------------------------------------------------------------

#ifdef use_xyz  
void Clipper::ZFillFunction(TZFillCallback zFillFunc)
{  
  m_ZFill = zFillFunc;
}
//------------------------------------------------------------------------------
#endif

void Clipper::Clear()
{
  if (m_edges.empty()) return; //avoids problems with ClipperBase destructor
  DisposeAllOutRecs();
  ClipperBase::Clear();
}
//------------------------------------------------------------------------------

void Clipper::Reset()
{
  ClipperBase::Reset();
  m_Scanbeam.clear();
  m_ActiveEdges = 0;
  m_SortedEdges = 0;
  DisposeAllOutRecs();
  LocalMinima* lm = m_MinimaList;
  while (lm)
  {
    InsertScanbeam(lm->Y);
    lm = lm->Next;
  }
}
//------------------------------------------------------------------------------

bool Clipper::Execute(ClipType clipType, Paths &solution,
    PolyFillType subjFillType, PolyFillType clipFillType)
{
  if( m_ExecuteLocked ) return false;
  if (m_HasOpenPaths)
    throw clipperException("Error: PolyTree struct is need for open path clipping.");
  m_ExecuteLocked = true;
  solution.resize(0);
  m_SubjFillType = subjFillType;
  m_ClipFillType = clipFillType;
  m_ClipType = clipType;
  m_UsingPolyTree = false;
  bool succeeded = ExecuteInternal();
  if (succeeded) BuildResult(solution);
  m_ExecuteLocked = false;
  return succeeded;
}
//------------------------------------------------------------------------------

bool Clipper::Execute(ClipType clipType, PolyTree& polytree,
    PolyFillType subjFillType, PolyFillType clipFillType)
{
  if( m_ExecuteLocked ) return false;
  m_ExecuteLocked = true;
  m_SubjFillType = subjFillType;
  m_ClipFillType = clipFillType;
  m_ClipType = clipType;
  m_UsingPolyTree = true;
  bool succeeded = ExecuteInternal();
  if (succeeded) BuildResult2(polytree);
  m_ExecuteLocked = false;
  return succeeded;
}
//------------------------------------------------------------------------------

void Clipper::FixHoleLinkage(OutRec &outrec)
{
  //skip OutRecs that (a) contain outermost polygons or
  //(b) already have the correct owner/child linkage ...
  if (!outrec.FirstLeft ||                
      (outrec.IsHole != outrec.FirstLeft->IsHole &&
      outrec.FirstLeft->Pts)) return;

  OutRec* orfl = outrec.FirstLeft;
  while (orfl && ((orfl->IsHole == outrec.IsHole) || !orfl->Pts))
      orfl = orfl->FirstLeft;
  outrec.FirstLeft = orfl;
}
//------------------------------------------------------------------------------

bool Clipper::ExecuteInternal()
{
  bool succeeded = true;
  try {
    Reset();
    if (!m_CurrentLM) return false;
    double botY = PopScanbeam();
    do {
      InsertLocalMinimaIntoAEL(botY);
      ClearGhostJoins();
      ProcessHorizontals(false);
      if (m_Scanbeam.empty()) break;
      double topY = PopScanbeam();
      succeeded = ProcessIntersections(botY, topY);
      if (!succeeded) break;
      ProcessEdgesAtTopOfScanbeam(topY);
      botY = topY;
    } while (!m_Scanbeam.empty() || m_CurrentLM);
  }
  catch(...) 
  {
    succeeded = false;
  }

  if (succeeded)
  {
    //fix orientations ...
    for (PolyOutList::size_type i = 0; i < m_PolyOuts.size(); ++i)
    {
      OutRec *outRec = m_PolyOuts[i];
      if (!outRec->Pts || outRec->IsOpen) continue;
      if ((outRec->IsHole ^ m_ReverseOutput) == (Area(*outRec) > 0))
        ReversePolyPtLinks(outRec->Pts);
    }

    if (!m_Joins.empty()) JoinCommonEdges();

    //unfortunately FixupOutPolygon() must be done after JoinCommonEdges()
    for (PolyOutList::size_type i = 0; i < m_PolyOuts.size(); ++i)
    {
      OutRec *outRec = m_PolyOuts[i];
      if (outRec->Pts && !outRec->IsOpen)
        FixupOutPolygon(*outRec);
    }

    if (m_StrictSimple) DoSimplePolygons();
  }

  ClearJoins();
  ClearGhostJoins();
  return succeeded;
}
//------------------------------------------------------------------------------

void Clipper::InsertScanbeam(const double Y)
{
  m_Scanbeam.insert(Y);
}
//------------------------------------------------------------------------------

double Clipper::PopScanbeam()
{
  double Y = *m_Scanbeam.begin();
  m_Scanbeam.erase(m_Scanbeam.begin());
  return Y;
}
//------------------------------------------------------------------------------

void Clipper::DisposeAllOutRecs(){
  for (PolyOutList::size_type i = 0; i < m_PolyOuts.size(); ++i)
    DisposeOutRec(i);
  m_PolyOuts.clear();
}
//------------------------------------------------------------------------------

void Clipper::DisposeOutRec(PolyOutList::size_type index)
{
  OutRec *outRec = m_PolyOuts[index];
  if (outRec->Pts) DisposeOutPts(outRec->Pts);
  delete outRec;
  m_PolyOuts[index] = 0;
}
//------------------------------------------------------------------------------

void Clipper::SetWindingCount(TEdge &edge)
{
  TEdge *e = edge.PrevInAEL;
  //find the edge of the same polytype that immediately preceeds 'edge' in AEL
  while (e  && ((e->PolyTyp != edge.PolyTyp) || (e->WindDelta == 0))) e = e->PrevInAEL;
  if (!e)
  {
    edge.WindCnt = (edge.WindDelta == 0 ? 1 : edge.WindDelta);
    edge.WindCnt2 = 0;
    e = m_ActiveEdges; //ie get ready to calc WindCnt2
  }   
  else if (edge.WindDelta == 0 && m_ClipType != ctUnion)
  {
    edge.WindCnt = 1;
    edge.WindCnt2 = e->WindCnt2;
    e = e->NextInAEL; //ie get ready to calc WindCnt2
  }
  else if (IsEvenOddFillType(edge))
  {
    //EvenOdd filling ...
    if (edge.WindDelta == 0)
    {
      //are we inside a subj polygon ...
      bool Inside = true;
      TEdge *e2 = e->PrevInAEL;
      while (e2)
      {
        if (e2->PolyTyp == e->PolyTyp && e2->WindDelta != 0) 
          Inside = !Inside;
        e2 = e2->PrevInAEL;
      }
      edge.WindCnt = (Inside ? 0 : 1);
    }
    else
    {
      edge.WindCnt = edge.WindDelta;
    }
    edge.WindCnt2 = e->WindCnt2;
    e = e->NextInAEL; //ie get ready to calc WindCnt2
  } 
  else
  {
    //nonZero, Positive or Negative filling ...
    if (e->WindCnt * e->WindDelta < 0)
    {
      //prev edge is 'decreasing' WindCount (WC) toward zero
      //so we're outside the previous polygon ...
      if (Abs(e->WindCnt) > 1)
      {
        //outside prev poly but still inside another.
        //when reversing direction of prev poly use the same WC 
        if (e->WindDelta * edge.WindDelta < 0) edge.WindCnt = e->WindCnt;
        //otherwise continue to 'decrease' WC ...
        else edge.WindCnt = e->WindCnt + edge.WindDelta;
      } 
      else
        //now outside all polys of same polytype so set own WC ...
        edge.WindCnt = (edge.WindDelta == 0 ? 1 : edge.WindDelta);
    } else
    {
      //prev edge is 'increasing' WindCount (WC) away from zero
      //so we're inside the previous polygon ...
      if (edge.WindDelta == 0) 
        edge.WindCnt = (e->WindCnt < 0 ? e->WindCnt - 1 : e->WindCnt + 1);
      //if wind direction is reversing prev then use same WC
      else if (e->WindDelta * edge.WindDelta < 0) edge.WindCnt = e->WindCnt;
      //otherwise add to WC ...
      else edge.WindCnt = e->WindCnt + edge.WindDelta;
    }
    edge.WindCnt2 = e->WindCnt2;
    e = e->NextInAEL; //ie get ready to calc WindCnt2
  }

  //update WindCnt2 ...
  if (IsEvenOddAltFillType(edge))
  {
    //EvenOdd filling ...
    while (e != &edge)
    {
      if (e->WindDelta != 0)
        edge.WindCnt2 = (edge.WindCnt2 == 0 ? 1 : 0);
      e = e->NextInAEL;
    }
  } else
  {
    //nonZero, Positive or Negative filling ...
    while ( e != &edge )
    {
      edge.WindCnt2 += e->WindDelta;
      e = e->NextInAEL;
    }
  }
}
//------------------------------------------------------------------------------

bool Clipper::IsEvenOddFillType(const TEdge& edge) const
{
  if (edge.PolyTyp == ptSubject)
    return m_SubjFillType == pftEvenOdd; else
    return m_ClipFillType == pftEvenOdd;
}
//------------------------------------------------------------------------------

bool Clipper::IsEvenOddAltFillType(const TEdge& edge) const
{
  if (edge.PolyTyp == ptSubject)
    return m_ClipFillType == pftEvenOdd; else
    return m_SubjFillType == pftEvenOdd;
}
//------------------------------------------------------------------------------

bool Clipper::IsContributing(const TEdge& edge) const
{
  PolyFillType pft, pft2;
  if (edge.PolyTyp == ptSubject)
  {
    pft = m_SubjFillType;
    pft2 = m_ClipFillType;
  } else
  {
    pft = m_ClipFillType;
    pft2 = m_SubjFillType;
  }

  switch(pft)
  {
    case pftEvenOdd: 
      //return false if a subj line has been flagged as inside a subj polygon
      if (edge.WindDelta == 0 && edge.WindCnt != 1) return false;
      break;
    case pftNonZero:
      if (Abs(edge.WindCnt) != 1) return false;
      break;
    case pftPositive: 
      if (edge.WindCnt != 1) return false;
      break;
    default: //pftNegative
      if (edge.WindCnt != -1) return false;
  }

  switch(m_ClipType)
  {
    case ctIntersection:
      switch(pft2)
      {
        case pftEvenOdd: 
        case pftNonZero: 
          return (edge.WindCnt2 != 0);
        case pftPositive: 
          return (edge.WindCnt2 > 0);
        default: 
          return (edge.WindCnt2 < 0);
      }
      break;
    case ctUnion:
      switch(pft2)
      {
        case pftEvenOdd: 
        case pftNonZero: 
          return (edge.WindCnt2 == 0);
        case pftPositive: 
          return (edge.WindCnt2 <= 0);
        default: 
          return (edge.WindCnt2 >= 0);
      }
      break;
    case ctDifference:
      if (edge.PolyTyp == ptSubject)
        switch(pft2)
        {
          case pftEvenOdd: 
          case pftNonZero: 
            return (edge.WindCnt2 == 0);
          case pftPositive: 
            return (edge.WindCnt2 <= 0);
          default: 
            return (edge.WindCnt2 >= 0);
        }
      else
        switch(pft2)
        {
          case pftEvenOdd: 
          case pftNonZero: 
            return (edge.WindCnt2 != 0);
          case pftPositive: 
            return (edge.WindCnt2 > 0);
          default: 
            return (edge.WindCnt2 < 0);
        }
      break;
    case ctXor:
      if (edge.WindDelta == 0) //XOr always contributing unless open
        switch(pft2)
        {
          case pftEvenOdd: 
          case pftNonZero: 
            return (edge.WindCnt2 == 0);
          case pftPositive: 
            return (edge.WindCnt2 <= 0);
          default: 
            return (edge.WindCnt2 >= 0);
        }
      else 
        return true;
      break;
    default:
      return true;
  }
}
//------------------------------------------------------------------------------

OutPt* Clipper::AddLocalMinPoly(TEdge *e1, TEdge *e2, const FPoint &Pt)
{
  OutPt* result;
  TEdge *e, *prevE;
  if (IsHorizontal(*e2) || ( e1->Dx > e2->Dx ))
  {
    result = AddOutPt(e1, Pt);
    e2->OutIdx = e1->OutIdx;
    e1->Side = esLeft;
    e2->Side = esRight;
    e = e1;
    if (e->PrevInAEL == e2)
      prevE = e2->PrevInAEL; 
    else
      prevE = e->PrevInAEL;
  } else
  {
    result = AddOutPt(e2, Pt);
    e1->OutIdx = e2->OutIdx;
    e1->Side = esRight;
    e2->Side = esLeft;
    e = e2;
    if (e->PrevInAEL == e1)
        prevE = e1->PrevInAEL;
    else
        prevE = e->PrevInAEL;
  }

  if (prevE && prevE->OutIdx >= 0 &&
      (TopX(*prevE, Pt.Y) == TopX(*e, Pt.Y)) &&
      SlopesEqual(*e, *prevE) &&
      (e->WindDelta != 0) && (prevE->WindDelta != 0))
  {
    OutPt* outPt = AddOutPt(prevE, Pt);
    AddJoin(result, outPt, e->Top);
  }
  return result;
}
//------------------------------------------------------------------------------

void Clipper::AddLocalMaxPoly(TEdge *e1, TEdge *e2, const FPoint &Pt)
{
  AddOutPt( e1, Pt );
  if (e2->WindDelta == 0) AddOutPt(e2, Pt);
  if( e1->OutIdx == e2->OutIdx )
  {
    e1->OutIdx = Unassigned;
    e2->OutIdx = Unassigned;
  }
  else if (e1->OutIdx < e2->OutIdx) 
    AppendPolygon(e1, e2); 
  else 
    AppendPolygon(e2, e1);
}
//------------------------------------------------------------------------------

void Clipper::AddEdgeToSEL(TEdge *edge)
{
  //SEL pointers in PEdge are reused to build a list of horizontal edges.
  //However, we don't need to worry about order with horizontal edge processing.
  if( !m_SortedEdges )
  {
    m_SortedEdges = edge;
    edge->PrevInSEL = 0;
    edge->NextInSEL = 0;
  }
  else
  {
    edge->NextInSEL = m_SortedEdges;
    edge->PrevInSEL = 0;
    m_SortedEdges->PrevInSEL = edge;
    m_SortedEdges = edge;
  }
}
//------------------------------------------------------------------------------

void Clipper::CopyAELToSEL()
{
  TEdge* e = m_ActiveEdges;
  m_SortedEdges = e;
  while ( e )
  {
    e->PrevInSEL = e->PrevInAEL;
    e->NextInSEL = e->NextInAEL;
    e = e->NextInAEL;
  }
}
//------------------------------------------------------------------------------

void Clipper::AddJoin(OutPt *op1, OutPt *op2, const FPoint OffPt)
{
  Join* j = new Join;
  j->OutPt1 = op1;
  j->OutPt2 = op2;
  j->OffPt = OffPt;
  m_Joins.push_back(j);
}
//------------------------------------------------------------------------------

void Clipper::ClearJoins()
{
  for (JoinList::size_type i = 0; i < m_Joins.size(); i++)
    delete m_Joins[i];
  m_Joins.resize(0);
}
//------------------------------------------------------------------------------

void Clipper::ClearGhostJoins()
{
  for (JoinList::size_type i = 0; i < m_GhostJoins.size(); i++)
    delete m_GhostJoins[i];
  m_GhostJoins.resize(0);
}
//------------------------------------------------------------------------------

void Clipper::AddGhostJoin(OutPt *op, const FPoint OffPt)
{
  Join* j = new Join;
  j->OutPt1 = op;
  j->OutPt2 = 0;
  j->OffPt = OffPt;
  m_GhostJoins.push_back(j);
}
//------------------------------------------------------------------------------

void Clipper::InsertLocalMinimaIntoAEL(const double botY)
{
  while(  m_CurrentLM  && ( m_CurrentLM->Y == botY ) )
  {
    TEdge* lb = m_CurrentLM->LeftBound;
    TEdge* rb = m_CurrentLM->RightBound;
    PopLocalMinima();
    OutPt *Op1 = 0;
    if (!lb)
    {
      //nb: don't insert LB into either AEL or SEL
      InsertEdgeIntoAEL(rb, 0);
      SetWindingCount(*rb);
      if (IsContributing(*rb))
        Op1 = AddOutPt(rb, rb->Bot); 
    } 
    else if (!rb)
    {
      InsertEdgeIntoAEL(lb, 0);
      SetWindingCount(*lb);
      if (IsContributing(*lb))
        Op1 = AddOutPt(lb, lb->Bot);
      InsertScanbeam(lb->Top.Y);
    }
    else
    {
      InsertEdgeIntoAEL(lb, 0);
      InsertEdgeIntoAEL(rb, lb);
      SetWindingCount( *lb );
      rb->WindCnt = lb->WindCnt;
      rb->WindCnt2 = lb->WindCnt2;
      if (IsContributing(*lb))
        Op1 = AddLocalMinPoly(lb, rb, lb->Bot);      
      InsertScanbeam(lb->Top.Y);
    }

     if (rb)
     {
       if(IsHorizontal(*rb)) AddEdgeToSEL(rb);
       else InsertScanbeam( rb->Top.Y );
     }

    if (!lb || !rb) continue;

    //if any output polygons share an edge, they'll need joining later ...
    if (Op1 && IsHorizontal(*rb) && 
      m_GhostJoins.size() > 0 && (rb->WindDelta != 0))
    {
      for (JoinList::size_type i = 0; i < m_GhostJoins.size(); ++i)
      {
        Join* jr = m_GhostJoins[i];
        //if the horizontal Rb and a 'ghost' horizontal overlap, then convert
        //the 'ghost' join to a real join ready for later ...
        if (HorzSegmentsOverlap(jr->OutPt1->Pt, jr->OffPt, rb->Bot, rb->Top))
          AddJoin(jr->OutPt1, Op1, jr->OffPt);
      }
    }

    if (lb->OutIdx >= 0 && lb->PrevInAEL && 
      lb->PrevInAEL->Curr.X == lb->Bot.X &&
      lb->PrevInAEL->OutIdx >= 0 &&
      SlopesEqual(*lb->PrevInAEL, *lb) &&
      (lb->WindDelta != 0) && (lb->PrevInAEL->WindDelta != 0))
    {
        OutPt *Op2 = AddOutPt(lb->PrevInAEL, lb->Bot);
        AddJoin(Op1, Op2, lb->Top);
    }

    if(lb->NextInAEL != rb)
    {

      if (rb->OutIdx >= 0 && rb->PrevInAEL->OutIdx >= 0 &&
        SlopesEqual(*rb->PrevInAEL, *rb) &&
        (rb->WindDelta != 0) && (rb->PrevInAEL->WindDelta != 0))
      {
          OutPt *Op2 = AddOutPt(rb->PrevInAEL, rb->Bot);
          AddJoin(Op1, Op2, rb->Top);
      }

      TEdge* e = lb->NextInAEL;
      if (e)
      {
        while( e != rb )
        {
          //nb: For calculating winding counts etc, IntersectEdges() assumes
          //that param1 will be to the Right of param2 ABOVE the intersection ...
          IntersectEdges(rb , e , lb->Curr); //order important here
          e = e->NextInAEL;
        }
      }
    }
    
  }
}
//------------------------------------------------------------------------------

void Clipper::DeleteFromAEL(TEdge *e)
{
  TEdge* AelPrev = e->PrevInAEL;
  TEdge* AelNext = e->NextInAEL;
  if(  !AelPrev &&  !AelNext && (e != m_ActiveEdges) ) return; //already deleted
  if( AelPrev ) AelPrev->NextInAEL = AelNext;
  else m_ActiveEdges = AelNext;
  if( AelNext ) AelNext->PrevInAEL = AelPrev;
  e->NextInAEL = 0;
  e->PrevInAEL = 0;
}
//------------------------------------------------------------------------------

void Clipper::DeleteFromSEL(TEdge *e)
{
  TEdge* SelPrev = e->PrevInSEL;
  TEdge* SelNext = e->NextInSEL;
  if( !SelPrev &&  !SelNext && (e != m_SortedEdges) ) return; //already deleted
  if( SelPrev ) SelPrev->NextInSEL = SelNext;
  else m_SortedEdges = SelNext;
  if( SelNext ) SelNext->PrevInSEL = SelPrev;
  e->NextInSEL = 0;
  e->PrevInSEL = 0;
}
//------------------------------------------------------------------------------

#ifdef use_xyz

void Clipper::SetZ(FPoint& pt, TEdge& e)
{
  pt.Z = 0;
  if (m_ZFill)
  {
    //put the 'preferred' point as first parameter ...
    if (e.OutIdx < 0)
      (*m_ZFill)(e.Bot, e.Top, pt); //outside a path so presume entering
    else
      (*m_ZFill)(e.Top, e.Bot, pt); //inside a path so presume exiting
  }
}
//------------------------------------------------------------------------------
#endif

void Clipper::IntersectEdges(TEdge *e1, TEdge *e2,
     const FPoint &Pt, bool protect)
{
  //e1 will be to the Left of e2 BELOW the intersection. Therefore e1 is before
  //e2 in AEL except when e1 is being inserted at the intersection point ...
  bool e1stops = !protect &&  !e1->NextInLML &&
    e1->Top.X == Pt.X && e1->Top.Y == Pt.Y;
  bool e2stops = !protect &&  !e2->NextInLML &&
    e2->Top.X == Pt.X && e2->Top.Y == Pt.Y;
  bool e1Contributing = ( e1->OutIdx >= 0 );
  bool e2Contributing = ( e2->OutIdx >= 0 );

#ifdef use_lines
  //if either edge is on an OPEN path ...
  if (e1->WindDelta == 0 || e2->WindDelta == 0)
  {
    //ignore subject-subject open path intersections UNLESS they
    //are both open paths, AND they are both 'contributing maximas' ...
    if (e1->WindDelta == 0 && e2->WindDelta == 0)
    {
      if ((e1stops || e2stops) && e1Contributing && e2Contributing)
        AddLocalMaxPoly(e1, e2, Pt);
    }

    //if intersecting a subj line with a subj poly ...
    else if (e1->PolyTyp == e2->PolyTyp && 
      e1->WindDelta != e2->WindDelta && m_ClipType == ctUnion)
    {
      if (e1->WindDelta == 0)
      {
        if (e2Contributing)
        {
          AddOutPt(e1, Pt);
          if (e1Contributing) e1->OutIdx = Unassigned;
        }
      }
      else
      {
        if (e1Contributing)
        {
          AddOutPt(e2, Pt);
          if (e2Contributing) e2->OutIdx = Unassigned;
        }
      }
    }
    else if (e1->PolyTyp != e2->PolyTyp)
    {
      //toggle subj open path OutIdx on/off when Abs(clip.WndCnt) == 1 ...
      if ((e1->WindDelta == 0) && abs(e2->WindCnt) == 1 && 
        (m_ClipType != ctUnion || e2->WindCnt2 == 0))
      {
        AddOutPt(e1, Pt);
        if (e1Contributing) e1->OutIdx = Unassigned;
      }
      else if ((e2->WindDelta == 0) && (abs(e1->WindCnt) == 1) && 
        (m_ClipType != ctUnion || e1->WindCnt2 == 0))
      {
        AddOutPt(e2, Pt);
        if (e2Contributing) e2->OutIdx = Unassigned;
      }
    }

    if (e1stops)
      if (e1->OutIdx < 0) DeleteFromAEL(e1);
      else throw clipperException("Error intersecting polylines");
    if (e2stops) 
      if (e2->OutIdx < 0) DeleteFromAEL(e2);
      else throw clipperException("Error intersecting polylines");
    return;
  }
#endif

  //update winding counts...
  //assumes that e1 will be to the Right of e2 ABOVE the intersection
  if ( e1->PolyTyp == e2->PolyTyp )
  {
    if ( IsEvenOddFillType( *e1) )
    {
      int oldE1WindCnt = e1->WindCnt;
      e1->WindCnt = e2->WindCnt;
      e2->WindCnt = oldE1WindCnt;
    } else
    {
      if (e1->WindCnt + e2->WindDelta == 0 ) e1->WindCnt = -e1->WindCnt;
      else e1->WindCnt += e2->WindDelta;
      if ( e2->WindCnt - e1->WindDelta == 0 ) e2->WindCnt = -e2->WindCnt;
      else e2->WindCnt -= e1->WindDelta;
    }
  } else
  {
    if (!IsEvenOddFillType(*e2)) e1->WindCnt2 += e2->WindDelta;
    else e1->WindCnt2 = ( e1->WindCnt2 == 0 ) ? 1 : 0;
    if (!IsEvenOddFillType(*e1)) e2->WindCnt2 -= e1->WindDelta;
    else e2->WindCnt2 = ( e2->WindCnt2 == 0 ) ? 1 : 0;
  }

  PolyFillType e1FillType, e2FillType, e1FillType2, e2FillType2;
  if (e1->PolyTyp == ptSubject)
  {
    e1FillType = m_SubjFillType;
    e1FillType2 = m_ClipFillType;
  } else
  {
    e1FillType = m_ClipFillType;
    e1FillType2 = m_SubjFillType;
  }
  if (e2->PolyTyp == ptSubject)
  {
    e2FillType = m_SubjFillType;
    e2FillType2 = m_ClipFillType;
  } else
  {
    e2FillType = m_ClipFillType;
    e2FillType2 = m_SubjFillType;
  }

  double e1Wc, e2Wc;
  switch (e1FillType)
  {
    case pftPositive: e1Wc = e1->WindCnt; break;
    case pftNegative: e1Wc = -e1->WindCnt; break;
    default: e1Wc = Abs(e1->WindCnt);
  }
  switch(e2FillType)
  {
    case pftPositive: e2Wc = e2->WindCnt; break;
    case pftNegative: e2Wc = -e2->WindCnt; break;
    default: e2Wc = Abs(e2->WindCnt);
  }

  if ( e1Contributing && e2Contributing )
  {
    if ( e1stops || e2stops || 
      (e1Wc != 0 && e1Wc != 1) || (e2Wc != 0 && e2Wc != 1) ||
      (e1->PolyTyp != e2->PolyTyp && m_ClipType != ctXor) )
        AddLocalMaxPoly(e1, e2, Pt); 
    else
    {
      AddOutPt(e1, Pt);
      AddOutPt(e2, Pt);
      SwapSides( *e1 , *e2 );
      SwapPolyIndexes( *e1 , *e2 );
    }
  }
  else if ( e1Contributing )
  {
    if (e2Wc == 0 || e2Wc == 1) 
    {
      AddOutPt(e1, Pt);
      SwapSides(*e1, *e2);
      SwapPolyIndexes(*e1, *e2);
    }
  }
  else if ( e2Contributing )
  {
    if (e1Wc == 0 || e1Wc == 1) 
    {
      AddOutPt(e2, Pt);
      SwapSides(*e1, *e2);
      SwapPolyIndexes(*e1, *e2);
    }
  } 
  else if ( (e1Wc == 0 || e1Wc == 1) && 
    (e2Wc == 0 || e2Wc == 1) && !e1stops && !e2stops )
  {
    //neither edge is currently contributing ...

    double e1Wc2, e2Wc2;
    switch (e1FillType2)
    {
      case pftPositive: e1Wc2 = e1->WindCnt2; break;
      case pftNegative : e1Wc2 = -e1->WindCnt2; break;
      default: e1Wc2 = Abs(e1->WindCnt2);
    }
    switch (e2FillType2)
    {
      case pftPositive: e2Wc2 = e2->WindCnt2; break;
      case pftNegative: e2Wc2 = -e2->WindCnt2; break;
      default: e2Wc2 = Abs(e2->WindCnt2);
    }

    if (e1->PolyTyp != e2->PolyTyp)
        AddLocalMinPoly(e1, e2, Pt);
    else if (e1Wc == 1 && e2Wc == 1)
      switch( m_ClipType ) {
        case ctIntersection:
          if (e1Wc2 > 0 && e2Wc2 > 0)
            AddLocalMinPoly(e1, e2, Pt);
          break;
        case ctUnion:
          if ( e1Wc2 <= 0 && e2Wc2 <= 0 )
            AddLocalMinPoly(e1, e2, Pt);
          break;
        case ctDifference:
          if (((e1->PolyTyp == ptClip) && (e1Wc2 > 0) && (e2Wc2 > 0)) ||
              ((e1->PolyTyp == ptSubject) && (e1Wc2 <= 0) && (e2Wc2 <= 0)))
                AddLocalMinPoly(e1, e2, Pt);
          break;
        case ctXor:
          AddLocalMinPoly(e1, e2, Pt);
      }
    else
      SwapSides( *e1, *e2 );
  }

  if(  (e1stops != e2stops) &&
    ( (e1stops && (e1->OutIdx >= 0)) || (e2stops && (e2->OutIdx >= 0)) ) )
  {
    SwapSides( *e1, *e2 );
    SwapPolyIndexes( *e1, *e2 );
  }

  //finally, delete any non-contributing maxima edges  ...
  if( e1stops ) DeleteFromAEL( e1 );
  if( e2stops ) DeleteFromAEL( e2 );
}
//------------------------------------------------------------------------------

void Clipper::SetHoleState(TEdge *e, OutRec *outrec)
{
  bool IsHole = false;
  TEdge *e2 = e->PrevInAEL;
  while (e2)
  {
    if (e2->OutIdx >= 0 && e2->WindDelta != 0)
    {
      IsHole = !IsHole;
      if (! outrec->FirstLeft)
        outrec->FirstLeft = m_PolyOuts[e2->OutIdx];
    }
    e2 = e2->PrevInAEL;
  }
  if (IsHole) outrec->IsHole = true;
}
//------------------------------------------------------------------------------

OutRec* GetLowermostRec(OutRec *outRec1, OutRec *outRec2)
{
  //work out which polygon fragment has the correct hole state ...
  if (!outRec1->BottomPt) 
    outRec1->BottomPt = GetBottomPt(outRec1->Pts);
  if (!outRec2->BottomPt) 
    outRec2->BottomPt = GetBottomPt(outRec2->Pts);
  OutPt *OutPt1 = outRec1->BottomPt;
  OutPt *OutPt2 = outRec2->BottomPt;
  if (OutPt1->Pt.Y > OutPt2->Pt.Y) return outRec1;
  else if (OutPt1->Pt.Y < OutPt2->Pt.Y) return outRec2;
  else if (OutPt1->Pt.X < OutPt2->Pt.X) return outRec1;
  else if (OutPt1->Pt.X > OutPt2->Pt.X) return outRec2;
  else if (OutPt1->Next == OutPt1) return outRec2;
  else if (OutPt2->Next == OutPt2) return outRec1;
  else if (FirstIsBottomPt(OutPt1, OutPt2)) return outRec1;
  else return outRec2;
}
//------------------------------------------------------------------------------

bool Param1RightOfParam2(OutRec* outRec1, OutRec* outRec2)
{
  do
  {
    outRec1 = outRec1->FirstLeft;
    if (outRec1 == outRec2) return true;
  } while (outRec1);
  return false;
}
//------------------------------------------------------------------------------

OutRec* Clipper::GetOutRec(int Idx)
{
  OutRec* outrec = m_PolyOuts[Idx];
  while (outrec != m_PolyOuts[outrec->Idx])
    outrec = m_PolyOuts[outrec->Idx];
  return outrec;
}
//------------------------------------------------------------------------------

void Clipper::AppendPolygon(TEdge *e1, TEdge *e2)
{
  //get the start and ends of both output polygons ...
  OutRec *outRec1 = m_PolyOuts[e1->OutIdx];
  OutRec *outRec2 = m_PolyOuts[e2->OutIdx];

  OutRec *holeStateRec;
  if (Param1RightOfParam2(outRec1, outRec2)) 
    holeStateRec = outRec2;
  else if (Param1RightOfParam2(outRec2, outRec1)) 
    holeStateRec = outRec1;
  else 
    holeStateRec = GetLowermostRec(outRec1, outRec2);

  //get the start and ends of both output polygons and
  //join e2 poly onto e1 poly and delete pointers to e2 ...

  OutPt* p1_lft = outRec1->Pts;
  OutPt* p1_rt = p1_lft->Prev;
  OutPt* p2_lft = outRec2->Pts;
  OutPt* p2_rt = p2_lft->Prev;

  EdgeSide Side;
  //join e2 poly onto e1 poly and delete pointers to e2 ...
  if(  e1->Side == esLeft )
  {
    if(  e2->Side == esLeft )
    {
      //z y x a b c
      ReversePolyPtLinks(p2_lft);
      p2_lft->Next = p1_lft;
      p1_lft->Prev = p2_lft;
      p1_rt->Next = p2_rt;
      p2_rt->Prev = p1_rt;
      outRec1->Pts = p2_rt;
    } else
    {
      //x y z a b c
      p2_rt->Next = p1_lft;
      p1_lft->Prev = p2_rt;
      p2_lft->Prev = p1_rt;
      p1_rt->Next = p2_lft;
      outRec1->Pts = p2_lft;
    }
    Side = esLeft;
  } else
  {
    if(  e2->Side == esRight )
    {
      //a b c z y x
      ReversePolyPtLinks(p2_lft);
      p1_rt->Next = p2_rt;
      p2_rt->Prev = p1_rt;
      p2_lft->Next = p1_lft;
      p1_lft->Prev = p2_lft;
    } else
    {
      //a b c x y z
      p1_rt->Next = p2_lft;
      p2_lft->Prev = p1_rt;
      p1_lft->Prev = p2_rt;
      p2_rt->Next = p1_lft;
    }
    Side = esRight;
  }

  outRec1->BottomPt = 0;
  if (holeStateRec == outRec2)
  {
    if (outRec2->FirstLeft != outRec1)
      outRec1->FirstLeft = outRec2->FirstLeft;
    outRec1->IsHole = outRec2->IsHole;
  }
  outRec2->Pts = 0;
  outRec2->BottomPt = 0;
  outRec2->FirstLeft = outRec1;

  int OKIdx = e1->OutIdx;
  int ObsoleteIdx = e2->OutIdx;

  e1->OutIdx = Unassigned; //nb: safe because we only get here via AddLocalMaxPoly
  e2->OutIdx = Unassigned;

  TEdge* e = m_ActiveEdges;
  while( e )
  {
    if( e->OutIdx == ObsoleteIdx )
    {
      e->OutIdx = OKIdx;
      e->Side = Side;
      break;
    }
    e = e->NextInAEL;
  }

  outRec2->Idx = outRec1->Idx;
}
//------------------------------------------------------------------------------

OutRec* Clipper::CreateOutRec()
{
  OutRec* result = new OutRec;
  result->IsHole = false;
  result->IsOpen = false;
  result->FirstLeft = 0;
  result->Pts = 0;
  result->BottomPt = 0;
  result->PolyNd = 0;
  m_PolyOuts.push_back(result);
  result->Idx = (int)m_PolyOuts.size()-1;
  return result;
}
//------------------------------------------------------------------------------

OutPt* Clipper::AddOutPt(TEdge *e, const FPoint &pt)
{
  bool ToFront = (e->Side == esLeft);
  if(  e->OutIdx < 0 )
  {
    OutRec *outRec = CreateOutRec();
    outRec->IsOpen = (e->WindDelta == 0);
    OutPt* newOp = new OutPt;
    outRec->Pts = newOp;
    newOp->Idx = outRec->Idx;
    newOp->Pt = pt;
    newOp->Next = newOp;
    newOp->Prev = newOp;
    if (!outRec->IsOpen)
      SetHoleState(e, outRec);
#ifdef use_xyz
    if (pt == e->Bot) newOp->Pt = e->Bot;
    else if (pt == e->Top) newOp->Pt = e->Top;
    else SetZ(newOp->Pt, *e);
#endif
    e->OutIdx = outRec->Idx; //nb: do this after SetZ !
    return newOp;
  } else
  {
    OutRec *outRec = m_PolyOuts[e->OutIdx];
    //OutRec.Pts is the 'Left-most' point & OutRec.Pts.Prev is the 'Right-most'
    OutPt* op = outRec->Pts;

    if (ToFront && (pt == op->Pt)) return op;
    else if (!ToFront && (pt == op->Prev->Pt)) return op->Prev;

    OutPt* newOp = new OutPt;
    newOp->Idx = outRec->Idx;
    newOp->Pt = pt;
    newOp->Next = op;
    newOp->Prev = op->Prev;
    newOp->Prev->Next = newOp;
    op->Prev = newOp;
    if (ToFront) outRec->Pts = newOp;
#ifdef use_xyz
    if (pt == e->Bot) newOp->Pt = e->Bot;
    else if (pt == e->Top) newOp->Pt = e->Top;
    else SetZ(newOp->Pt, *e);
#endif
    return newOp;
  }
}
//------------------------------------------------------------------------------

void Clipper::ProcessHorizontals(bool IsTopOfScanbeam)
{
  TEdge* horzEdge = m_SortedEdges;
  while(horzEdge)
  {
    DeleteFromSEL(horzEdge);
    ProcessHorizontal(horzEdge, IsTopOfScanbeam);
    horzEdge = m_SortedEdges;
  }
}
//------------------------------------------------------------------------------

inline bool IsMinima(TEdge *e)
{
  return e  && (e->Prev->NextInLML != e) && (e->Next->NextInLML != e);
}
//------------------------------------------------------------------------------

inline bool IsMaxima(TEdge *e, const double Y)
{
  return e && e->Top.Y == Y && !e->NextInLML;
}
//------------------------------------------------------------------------------

inline bool IsIntermediate(TEdge *e, const double Y)
{
  return e->Top.Y == Y && e->NextInLML;
}
//------------------------------------------------------------------------------

TEdge *GetMaximaPair(TEdge *e)
{
  TEdge* result = 0;
  if ((e->Next->Top == e->Top) && !e->Next->NextInLML)
    result = e->Next;
  else if ((e->Prev->Top == e->Top) && !e->Prev->NextInLML)
    result = e->Prev;

  if (result && (result->OutIdx == Skip ||
    //result is false if both NextInAEL & PrevInAEL are nil & not horizontal ...
    (result->NextInAEL == result->PrevInAEL && !IsHorizontal(*result))))
      return 0;
  return result;
}
//------------------------------------------------------------------------------

void Clipper::SwapPositionsInAEL(TEdge *Edge1, TEdge *Edge2)
{
  //check that one or other edge hasn't already been removed from AEL ...
  if (Edge1->NextInAEL == Edge1->PrevInAEL || 
    Edge2->NextInAEL == Edge2->PrevInAEL) return;

  if(  Edge1->NextInAEL == Edge2 )
  {
    TEdge* Next = Edge2->NextInAEL;
    if( Next ) Next->PrevInAEL = Edge1;
    TEdge* Prev = Edge1->PrevInAEL;
    if( Prev ) Prev->NextInAEL = Edge2;
    Edge2->PrevInAEL = Prev;
    Edge2->NextInAEL = Edge1;
    Edge1->PrevInAEL = Edge2;
    Edge1->NextInAEL = Next;
  }
  else if(  Edge2->NextInAEL == Edge1 )
  {
    TEdge* Next = Edge1->NextInAEL;
    if( Next ) Next->PrevInAEL = Edge2;
    TEdge* Prev = Edge2->PrevInAEL;
    if( Prev ) Prev->NextInAEL = Edge1;
    Edge1->PrevInAEL = Prev;
    Edge1->NextInAEL = Edge2;
    Edge2->PrevInAEL = Edge1;
    Edge2->NextInAEL = Next;
  }
  else
  {
    TEdge* Next = Edge1->NextInAEL;
    TEdge* Prev = Edge1->PrevInAEL;
    Edge1->NextInAEL = Edge2->NextInAEL;
    if( Edge1->NextInAEL ) Edge1->NextInAEL->PrevInAEL = Edge1;
    Edge1->PrevInAEL = Edge2->PrevInAEL;
    if( Edge1->PrevInAEL ) Edge1->PrevInAEL->NextInAEL = Edge1;
    Edge2->NextInAEL = Next;
    if( Edge2->NextInAEL ) Edge2->NextInAEL->PrevInAEL = Edge2;
    Edge2->PrevInAEL = Prev;
    if( Edge2->PrevInAEL ) Edge2->PrevInAEL->NextInAEL = Edge2;
  }

  if( !Edge1->PrevInAEL ) m_ActiveEdges = Edge1;
  else if( !Edge2->PrevInAEL ) m_ActiveEdges = Edge2;
}
//------------------------------------------------------------------------------

void Clipper::SwapPositionsInSEL(TEdge *Edge1, TEdge *Edge2)
{
  if(  !( Edge1->NextInSEL ) &&  !( Edge1->PrevInSEL ) ) return;
  if(  !( Edge2->NextInSEL ) &&  !( Edge2->PrevInSEL ) ) return;

  if(  Edge1->NextInSEL == Edge2 )
  {
    TEdge* Next = Edge2->NextInSEL;
    if( Next ) Next->PrevInSEL = Edge1;
    TEdge* Prev = Edge1->PrevInSEL;
    if( Prev ) Prev->NextInSEL = Edge2;
    Edge2->PrevInSEL = Prev;
    Edge2->NextInSEL = Edge1;
    Edge1->PrevInSEL = Edge2;
    Edge1->NextInSEL = Next;
  }
  else if(  Edge2->NextInSEL == Edge1 )
  {
    TEdge* Next = Edge1->NextInSEL;
    if( Next ) Next->PrevInSEL = Edge2;
    TEdge* Prev = Edge2->PrevInSEL;
    if( Prev ) Prev->NextInSEL = Edge1;
    Edge1->PrevInSEL = Prev;
    Edge1->NextInSEL = Edge2;
    Edge2->PrevInSEL = Edge1;
    Edge2->NextInSEL = Next;
  }
  else
  {
    TEdge* Next = Edge1->NextInSEL;
    TEdge* Prev = Edge1->PrevInSEL;
    Edge1->NextInSEL = Edge2->NextInSEL;
    if( Edge1->NextInSEL ) Edge1->NextInSEL->PrevInSEL = Edge1;
    Edge1->PrevInSEL = Edge2->PrevInSEL;
    if( Edge1->PrevInSEL ) Edge1->PrevInSEL->NextInSEL = Edge1;
    Edge2->NextInSEL = Next;
    if( Edge2->NextInSEL ) Edge2->NextInSEL->PrevInSEL = Edge2;
    Edge2->PrevInSEL = Prev;
    if( Edge2->PrevInSEL ) Edge2->PrevInSEL->NextInSEL = Edge2;
  }

  if( !Edge1->PrevInSEL ) m_SortedEdges = Edge1;
  else if( !Edge2->PrevInSEL ) m_SortedEdges = Edge2;
}
//------------------------------------------------------------------------------

TEdge* GetNextInAEL(TEdge *e, Direction dir)
{
  return dir == dLeftToRight ? e->NextInAEL : e->PrevInAEL;
}
//------------------------------------------------------------------------------

void GetHorzDirection(TEdge& HorzEdge, Direction& Dir, double& Left, double& Right)
{
  if (HorzEdge.Bot.X < HorzEdge.Top.X)
  {
    Left = HorzEdge.Bot.X;
    Right = HorzEdge.Top.X;
    Dir = dLeftToRight;
  } else
  {
    Left = HorzEdge.Top.X;
    Right = HorzEdge.Bot.X;
    Dir = dRightToLeft;
  }
}
//------------------------------------------------------------------------

void Clipper::PrepareHorzJoins(TEdge* horzEdge, bool isTopOfScanbeam)
{
  //get the last Op for this horizontal edge
  //the point may be anywhere along the horizontal ...
  OutPt* outPt = m_PolyOuts[horzEdge->OutIdx]->Pts;
  if (horzEdge->Side != esLeft) outPt = outPt->Prev;

  //First, match up overlapping horizontal edges (eg when one polygon's
  //intermediate horz edge overlaps an intermediate horz edge of another, or
  //when one polygon sits on top of another) ...
  for (JoinList::size_type i = 0; i < m_GhostJoins.size(); ++i)
  {
    Join* j = m_GhostJoins[i];
    if (HorzSegmentsOverlap(j->OutPt1->Pt, j->OffPt, horzEdge->Bot, horzEdge->Top))
        AddJoin(j->OutPt1, outPt, j->OffPt);
  }
  //Also, since horizontal edges at the top of one SB are often removed from
  //the AEL before we process the horizontal edges at the bottom of the next,
  //we need to create 'ghost' Join records of 'contrubuting' horizontals that
  //we can compare with horizontals at the bottom of the next SB.
  if (isTopOfScanbeam) 
  {
    if (outPt->Pt == horzEdge->Top)
      AddGhostJoin(outPt, horzEdge->Bot); 
    else
      AddGhostJoin(outPt, horzEdge->Top);
  }
}
//------------------------------------------------------------------------------

/*******************************************************************************
* Notes: Horizontal edges (HEs) at scanline intersections (ie at the Top or    *
* Bottom of a scanbeam) are processed as if layered. The order in which HEs    *
* are processed doesn't matter. HEs intersect with other HE Bot.Xs only [#]    *
* (or they could intersect with Top.Xs only, ie EITHER Bot.Xs OR Top.Xs),      *
* and with other non-horizontal edges [*]. Once these intersections are        *
* processed, intermediate HEs then 'promote' the Edge above (NextInLML) into   *
* the AEL. These 'promoted' edges may in turn intersect [%] with other HEs.    *
*******************************************************************************/

void Clipper::ProcessHorizontal(TEdge *horzEdge, bool isTopOfScanbeam)
{
  Direction dir;
  double horzLeft, horzRight;

  GetHorzDirection(*horzEdge, dir, horzLeft, horzRight);

  TEdge* eLastHorz = horzEdge, *eMaxPair = 0;
  while (eLastHorz->NextInLML && IsHorizontal(*eLastHorz->NextInLML)) 
    eLastHorz = eLastHorz->NextInLML;
  if (!eLastHorz->NextInLML)
    eMaxPair = GetMaximaPair(eLastHorz);

  for (;;)
  {
    bool IsLastHorz = (horzEdge == eLastHorz);
    TEdge* e = GetNextInAEL(horzEdge, dir);
    while(e)
    {
      //Break if we've got to the end of an intermediate horizontal edge ...
      //nb: Smaller Dx's are to the right of larger Dx's ABOVE the horizontal.
      if (e->Curr.X == horzEdge->Top.X && horzEdge->NextInLML && 
        e->Dx < horzEdge->NextInLML->Dx) break;

      TEdge* eNext = GetNextInAEL(e, dir); //saves eNext for later

      if ((dir == dLeftToRight && e->Curr.X <= horzRight) ||
        (dir == dRightToLeft && e->Curr.X >= horzLeft))
      {
        if (horzEdge->OutIdx >= 0 && horzEdge->WindDelta != 0) 
          PrepareHorzJoins(horzEdge, isTopOfScanbeam);
        //so far we're still in range of the horizontal Edge  but make sure
        //we're at the last of consec. horizontals when matching with eMaxPair
        if(e == eMaxPair && IsLastHorz)
        {
          if (dir == dLeftToRight)
            IntersectEdges(horzEdge, e, e->Top);
          else
            IntersectEdges(e, horzEdge, e->Top);
          if (eMaxPair->OutIdx >= 0) throw clipperException("ProcessHorizontal error");
          return;
        }
        else if(dir == dLeftToRight)
        {
          FPoint Pt = FPoint(e->Curr.X, horzEdge->Curr.Y);
          IntersectEdges(horzEdge, e, Pt, true);
        }
        else
        {
          FPoint Pt = FPoint(e->Curr.X, horzEdge->Curr.Y);
          IntersectEdges( e, horzEdge, Pt, true);
        }
        SwapPositionsInAEL( horzEdge, e );
      }
      else if( (dir == dLeftToRight && e->Curr.X >= horzRight) ||
       (dir == dRightToLeft && e->Curr.X <= horzLeft) ) break;
      e = eNext;
    } //end while

    if (horzEdge->OutIdx >= 0 && horzEdge->WindDelta != 0)
      PrepareHorzJoins(horzEdge, isTopOfScanbeam);

    if (horzEdge->NextInLML && IsHorizontal(*horzEdge->NextInLML))
    {
      UpdateEdgeIntoAEL(horzEdge);
      if (horzEdge->OutIdx >= 0) AddOutPt(horzEdge, horzEdge->Bot);
      GetHorzDirection(*horzEdge, dir, horzLeft, horzRight);
    } else
      break;
  } //end for (;;)

  if(horzEdge->NextInLML)
  {
    if(horzEdge->OutIdx >= 0)
    {
      OutPt* op1 = AddOutPt( horzEdge, horzEdge->Top);
      UpdateEdgeIntoAEL(horzEdge);
      if (horzEdge->WindDelta == 0) return;
      //nb: HorzEdge is no longer horizontal here
      TEdge* ePrev = horzEdge->PrevInAEL;
      TEdge* eNext = horzEdge->NextInAEL;
      if (ePrev && ePrev->Curr.X == horzEdge->Bot.X &&
        ePrev->Curr.Y == horzEdge->Bot.Y && ePrev->WindDelta != 0 &&
        (ePrev->OutIdx >= 0 && ePrev->Curr.Y > ePrev->Top.Y &&
        SlopesEqual(*horzEdge, *ePrev)))
      {
        OutPt* op2 = AddOutPt(ePrev, horzEdge->Bot);
        AddJoin(op1, op2, horzEdge->Top);
      }
      else if (eNext && eNext->Curr.X == horzEdge->Bot.X &&
        eNext->Curr.Y == horzEdge->Bot.Y && eNext->WindDelta != 0 &&
        eNext->OutIdx >= 0 && eNext->Curr.Y > eNext->Top.Y &&
        SlopesEqual(*horzEdge, *eNext))
      {
        OutPt* op2 = AddOutPt(eNext, horzEdge->Bot);
        AddJoin(op1, op2, horzEdge->Top);
      }
    }
    else
      UpdateEdgeIntoAEL(horzEdge); 
  }
  else if (eMaxPair)
  {
    if (eMaxPair->OutIdx >= 0)
    {
      if (dir == dLeftToRight)
        IntersectEdges(horzEdge, eMaxPair, horzEdge->Top); 
      else
        IntersectEdges(eMaxPair, horzEdge, horzEdge->Top);
      if (eMaxPair->OutIdx >= 0)
        throw clipperException("ProcessHorizontal error");
    } else
    {
      DeleteFromAEL(horzEdge);
      DeleteFromAEL(eMaxPair);
    }
  } else
  {
    if (horzEdge->OutIdx >= 0) AddOutPt(horzEdge, horzEdge->Top);
    DeleteFromAEL(horzEdge);
  }
}
//------------------------------------------------------------------------------

void Clipper::UpdateEdgeIntoAEL(TEdge *&e)
{
  if( !e->NextInLML ) throw
    clipperException("UpdateEdgeIntoAEL: invalid call");

  e->NextInLML->OutIdx = e->OutIdx;
  TEdge* AelPrev = e->PrevInAEL;
  TEdge* AelNext = e->NextInAEL;
  if (AelPrev) AelPrev->NextInAEL = e->NextInLML;
  else m_ActiveEdges = e->NextInLML;
  if (AelNext) AelNext->PrevInAEL = e->NextInLML;
  e->NextInLML->Side = e->Side;
  e->NextInLML->WindDelta = e->WindDelta;
  e->NextInLML->WindCnt = e->WindCnt;
  e->NextInLML->WindCnt2 = e->WindCnt2;
  e = e->NextInLML;
  e->Curr = e->Bot;
  e->PrevInAEL = AelPrev;
  e->NextInAEL = AelNext;
  if (!IsHorizontal(*e)) InsertScanbeam(e->Top.Y);
}
//------------------------------------------------------------------------------

bool Clipper::ProcessIntersections(const double botY, const double topY)
{
  if( !m_ActiveEdges ) return true;
  try {
    BuildIntersectList(botY, topY);
    size_t IlSize = m_IntersectList.size();
    if (IlSize == 0) return true;
    if (IlSize == 1 || FixupIntersectionOrder()) ProcessIntersectList();
    else return false;
  }
  catch(...) 
  {
    m_SortedEdges = 0;
    DisposeIntersectNodes();
    throw clipperException("ProcessIntersections error");
  }
  m_SortedEdges = 0;
  return true;
}
//------------------------------------------------------------------------------

void Clipper::DisposeIntersectNodes()
{
  for (size_t i = 0; i < m_IntersectList.size(); ++i )
    delete m_IntersectList[i];
  m_IntersectList.clear();
}
//------------------------------------------------------------------------------

void Clipper::BuildIntersectList(const double botY, const double topY)
{
  if ( !m_ActiveEdges ) return;

  //prepare for sorting ...
  TEdge* e = m_ActiveEdges;
  m_SortedEdges = e;
  while( e )
  {
    e->PrevInSEL = e->PrevInAEL;
    e->NextInSEL = e->NextInAEL;
    e->Curr.X = TopX( *e, topY );
    e = e->NextInAEL;
  }

  //bubblesort ...
  bool isModified;
  do
  {
    isModified = false;
    e = m_SortedEdges;
    while( e->NextInSEL )
    {
      TEdge *eNext = e->NextInSEL;
      FPoint Pt;
      if(e->Curr.X > eNext->Curr.X)
      {
        IntersectPoint(*e, *eNext, Pt);
        if (Pt.Y > botY)
        {
            Pt.Y = botY;
            if (std::fabs(e->Dx) > std::fabs(eNext->Dx))
              Pt.X = TopX(*eNext, botY); else
              Pt.X = TopX(*e, botY);
        }

        IntersectNode * newNode = new IntersectNode;
        newNode->Edge1 = e;
        newNode->Edge2 = eNext;
        newNode->Pt = Pt;
        m_IntersectList.push_back(newNode);

        SwapPositionsInSEL(e, eNext);
        isModified = true;
      }
      else
        e = eNext;
    }
    if( e->PrevInSEL ) e->PrevInSEL->NextInSEL = 0;
    else break;
  }
  while ( isModified );
  m_SortedEdges = 0; //important
}
//------------------------------------------------------------------------------


void Clipper::ProcessIntersectList()
{
  for (size_t i = 0; i < m_IntersectList.size(); ++i)
  {
    IntersectNode* iNode = m_IntersectList[i];
    {
      IntersectEdges( iNode->Edge1, iNode->Edge2, iNode->Pt, true);
      SwapPositionsInAEL( iNode->Edge1 , iNode->Edge2 );
    }
    delete iNode;
  }
  m_IntersectList.clear();
}
//------------------------------------------------------------------------------

bool IntersectListSort(IntersectNode* node1, IntersectNode* node2)
{
  return node2->Pt.Y < node1->Pt.Y;
}
//------------------------------------------------------------------------------

inline bool EdgesAdjacent(const IntersectNode &inode)
{
  return (inode.Edge1->NextInSEL == inode.Edge2) ||
    (inode.Edge1->PrevInSEL == inode.Edge2);
}
//------------------------------------------------------------------------------

bool Clipper::FixupIntersectionOrder()
{
  //pre-condition: intersections are sorted bottom-most first.
  //Now it's crucial that intersections are made only between adjacent edges,
  //so to ensure this the order of intersections may need adjusting ...
  CopyAELToSEL();
  std::sort(m_IntersectList.begin(), m_IntersectList.end(), IntersectListSort);
  size_t cnt = m_IntersectList.size();
  for (size_t i = 0; i < cnt; ++i) 
  {
    if (!EdgesAdjacent(*m_IntersectList[i]))
    {
      size_t j = i + 1;
      while (j < cnt && !EdgesAdjacent(*m_IntersectList[j])) j++;
      if (j == cnt)  return false;
      std::swap(m_IntersectList[i], m_IntersectList[j]);
    }
    SwapPositionsInSEL(m_IntersectList[i]->Edge1, m_IntersectList[i]->Edge2);
  }
  return true;
}
//------------------------------------------------------------------------------

void Clipper::DoMaxima(TEdge *e)
{
  TEdge* eMaxPair = GetMaximaPair(e);
  if (!eMaxPair)
  {
    if (e->OutIdx >= 0)
      AddOutPt(e, e->Top);
    DeleteFromAEL(e);
    return;
  }

  TEdge* eNext = e->NextInAEL;
  while(eNext && eNext != eMaxPair)
  {
    IntersectEdges(e, eNext, e->Top, true);
    SwapPositionsInAEL(e, eNext);
    eNext = e->NextInAEL;
  }

  if(e->OutIdx == Unassigned && eMaxPair->OutIdx == Unassigned)
  {
    DeleteFromAEL(e);
    DeleteFromAEL(eMaxPair);
  }
  else if( e->OutIdx >= 0 && eMaxPair->OutIdx >= 0 )
  {
    IntersectEdges( e, eMaxPair, e->Top);
  }
#ifdef use_lines
  else if (e->WindDelta == 0)
  {
    if (e->OutIdx >= 0) 
    {
      AddOutPt(e, e->Top);
      e->OutIdx = Unassigned;
    }
    DeleteFromAEL(e);

    if (eMaxPair->OutIdx >= 0)
    {
      AddOutPt(eMaxPair, e->Top);
      eMaxPair->OutIdx = Unassigned;
    }
    DeleteFromAEL(eMaxPair);
  } 
#endif
  else throw clipperException("DoMaxima error");
}
//------------------------------------------------------------------------------

void Clipper::ProcessEdgesAtTopOfScanbeam(const double topY)
{
  TEdge* e = m_ActiveEdges;
  while( e )
  {
    //1. process maxima, treating them as if they're 'bent' horizontal edges,
    //   but exclude maxima with horizontal edges. nb: e can't be a horizontal.
    bool IsMaximaEdge = IsMaxima(e, topY);

    if(IsMaximaEdge)
    {
      TEdge* eMaxPair = GetMaximaPair(e);
      IsMaximaEdge = (!eMaxPair || !IsHorizontal(*eMaxPair));
    }

    if(IsMaximaEdge)
    {
      TEdge* ePrev = e->PrevInAEL;
      DoMaxima(e);
      if( !ePrev ) e = m_ActiveEdges;
      else e = ePrev->NextInAEL;
    }
    else
    {
      //2. promote horizontal edges, otherwise update Curr.X and Curr.Y ...
      if (IsIntermediate(e, topY) && IsHorizontal(*e->NextInLML))
      {
        UpdateEdgeIntoAEL(e);
        if (e->OutIdx >= 0)
          AddOutPt(e, e->Bot);
        AddEdgeToSEL(e);
      } 
      else
      {
        e->Curr.X = TopX( *e, topY );
        e->Curr.Y = topY;
      }

      if (m_StrictSimple)
      {  
        TEdge* ePrev = e->PrevInAEL;
        if ((e->OutIdx >= 0) && (e->WindDelta != 0) && ePrev && (ePrev->OutIdx >= 0) &&
          (ePrev->Curr.X == e->Curr.X) && (ePrev->WindDelta != 0))
        {
          OutPt* op = AddOutPt(ePrev, e->Curr);
          OutPt* op2 = AddOutPt(e, e->Curr);
          AddJoin(op, op2, e->Curr); //StrictlySimple (type-3) join
        }
      }

      e = e->NextInAEL;
    }
  }

  //3. Process horizontals at the Top of the scanbeam ...
  ProcessHorizontals(true);

  //4. Promote intermediate vertices ...
  e = m_ActiveEdges;
  while(e)
  {
    if(IsIntermediate(e, topY))
    {
      OutPt* op = 0;
      if( e->OutIdx >= 0 ) 
        op = AddOutPt(e, e->Top);
      UpdateEdgeIntoAEL(e);

      //if output polygons share an edge, they'll need joining later ...
      TEdge* ePrev = e->PrevInAEL;
      TEdge* eNext = e->NextInAEL;
      if (ePrev && ePrev->Curr.X == e->Bot.X &&
        ePrev->Curr.Y == e->Bot.Y && op &&
        ePrev->OutIdx >= 0 && ePrev->Curr.Y > ePrev->Top.Y &&
        SlopesEqual(*e, *ePrev) &&
        (e->WindDelta != 0) && (ePrev->WindDelta != 0))
      {
        OutPt* op2 = AddOutPt(ePrev, e->Bot);
        AddJoin(op, op2, e->Top);
      }
      else if (eNext && eNext->Curr.X == e->Bot.X &&
        eNext->Curr.Y == e->Bot.Y && op &&
        eNext->OutIdx >= 0 && eNext->Curr.Y > eNext->Top.Y &&
        SlopesEqual(*e, *eNext) &&
        (e->WindDelta != 0) && (eNext->WindDelta != 0))
      {
        OutPt* op2 = AddOutPt(eNext, e->Bot);
        AddJoin(op, op2, e->Top);
      }
    }
    e = e->NextInAEL;
  }
}
//------------------------------------------------------------------------------

void Clipper::FixupOutPolygon(OutRec &outrec)
{
  //FixupOutPolygon() - removes duplicate points and simplifies consecutive
  //parallel edges by removing the middle vertex.
  OutPt *lastOK = 0;
  outrec.BottomPt = 0;
  OutPt *pp = outrec.Pts;

  for (;;)
  {
    if (pp->Prev == pp || pp->Prev == pp->Next )
    {
      DisposeOutPts(pp);
      outrec.Pts = 0;
      return;
    }

    //test for duplicate points and collinear edges ...
    if ((pp->Pt == pp->Next->Pt) || (pp->Pt == pp->Prev->Pt) || 
      (SlopesEqual(pp->Prev->Pt, pp->Pt, pp->Next->Pt) &&
      (!m_PreserveCollinear || 
      !Pt2IsBetweenPt1AndPt3(pp->Prev->Pt, pp->Pt, pp->Next->Pt))))
    {
      lastOK = 0;
      OutPt *tmp = pp;
      pp->Prev->Next = pp->Next;
      pp->Next->Prev = pp->Prev;
      pp = pp->Prev;
      delete tmp;
    }
    else if (pp == lastOK) break;
    else
    {
      if (!lastOK) lastOK = pp;
      pp = pp->Next;
    }
  }
  outrec.Pts = pp;
}
//------------------------------------------------------------------------------

int PointCount(OutPt *Pts)
{
    if (!Pts) return 0;
    int result = 0;
    OutPt* p = Pts;
    do
    {
        result++;
        p = p->Next;
    }
    while (p != Pts);
    return result;
}
//------------------------------------------------------------------------------

void Clipper::BuildResult(Paths &polys)
{
  polys.reserve(m_PolyOuts.size());
  for (PolyOutList::size_type i = 0; i < m_PolyOuts.size(); ++i)
  {
    if (!m_PolyOuts[i]->Pts) continue;
    Path pg;
    OutPt* p = m_PolyOuts[i]->Pts->Prev;
    int cnt = PointCount(p);
    if (cnt < 2) continue;
    pg.reserve(cnt);
    for (int i = 0; i < cnt; ++i)
    {
      pg.push_back(p->Pt);
      p = p->Prev;
    }
    polys.push_back(pg);
  }
}
//------------------------------------------------------------------------------

void Clipper::BuildResult2(PolyTree& polytree)
{
    polytree.Clear();
    polytree.AllNodes.reserve(m_PolyOuts.size());
    //add each output polygon/contour to polytree ...
    for (PolyOutList::size_type i = 0; i < m_PolyOuts.size(); i++)
    {
        OutRec* outRec = m_PolyOuts[i];
        int cnt = PointCount(outRec->Pts);
        if ((outRec->IsOpen && cnt < 2) || (!outRec->IsOpen && cnt < 3)) continue;
        FixHoleLinkage(*outRec);
        PolyNode* pn = new PolyNode();
        //nb: polytree takes ownership of all the PolyNodes
        polytree.AllNodes.push_back(pn);
        outRec->PolyNd = pn;
        pn->Parent = 0;
        pn->Index = 0;
        pn->Contour.reserve(cnt);
        OutPt *op = outRec->Pts->Prev;
        for (int j = 0; j < cnt; j++)
        {
            pn->Contour.push_back(op->Pt);
            op = op->Prev;
        }
    }

    //fixup PolyNode links etc ...
    polytree.Childs.reserve(m_PolyOuts.size());
    for (PolyOutList::size_type i = 0; i < m_PolyOuts.size(); i++)
    {
        OutRec* outRec = m_PolyOuts[i];
        if (!outRec->PolyNd) continue;
        if (outRec->IsOpen) 
        {
          outRec->PolyNd->m_IsOpen = true;
          polytree.AddChild(*outRec->PolyNd);
        }
        else if (outRec->FirstLeft && outRec->FirstLeft->PolyNd) 
          outRec->FirstLeft->PolyNd->AddChild(*outRec->PolyNd);
        else
          polytree.AddChild(*outRec->PolyNd);
    }
}
//------------------------------------------------------------------------------

void SwapIntersectNodes(IntersectNode &int1, IntersectNode &int2)
{
  //just swap the contents (because fIntersectNodes is a single-linked-list)
  IntersectNode inode = int1; //gets a copy of Int1
  int1.Edge1 = int2.Edge1;
  int1.Edge2 = int2.Edge2;
  int1.Pt = int2.Pt;
  int2.Edge1 = inode.Edge1;
  int2.Edge2 = inode.Edge2;
  int2.Pt = inode.Pt;
}
//------------------------------------------------------------------------------

inline bool E2InsertsBeforeE1(TEdge &e1, TEdge &e2)
{
  if (e2.Curr.X == e1.Curr.X) 
  {
    if (e2.Top.Y > e1.Top.Y)
      return e2.Top.X < TopX(e1, e2.Top.Y); 
      else return e1.Top.X > TopX(e2, e1.Top.Y);
  } 
  else return e2.Curr.X < e1.Curr.X;
}
//------------------------------------------------------------------------------

bool GetOverlap(const double a1, const double a2, const double b1, const double b2, 
    double& Left, double& Right)
{
  if (a1 < a2)
  {
    if (b1 < b2) {Left = std::max(a1,b1); Right = std::min(a2,b2);}
    else {Left = std::max(a1,b2); Right = std::min(a2,b1);}
  } 
  else
  {
    if (b1 < b2) {Left = std::max(a2,b1); Right = std::min(a1,b2);}
    else {Left = std::max(a2,b2); Right = std::min(a1,b1);}
  }
  return Left < Right;
}
//------------------------------------------------------------------------------

inline void UpdateOutPtIdxs(OutRec& outrec)
{  
  OutPt* op = outrec.Pts;
  do
  {
    op->Idx = outrec.Idx;
    op = op->Prev;
  }
  while(op != outrec.Pts);
}
//------------------------------------------------------------------------------

void Clipper::InsertEdgeIntoAEL(TEdge *edge, TEdge* startEdge)
{
  if(!m_ActiveEdges)
  {
    edge->PrevInAEL = 0;
    edge->NextInAEL = 0;
    m_ActiveEdges = edge;
  }
  else if(!startEdge && E2InsertsBeforeE1(*m_ActiveEdges, *edge))
  {
      edge->PrevInAEL = 0;
      edge->NextInAEL = m_ActiveEdges;
      m_ActiveEdges->PrevInAEL = edge;
      m_ActiveEdges = edge;
  } 
  else
  {
    if(!startEdge) startEdge = m_ActiveEdges;
    while(startEdge->NextInAEL  && 
      !E2InsertsBeforeE1(*startEdge->NextInAEL , *edge))
        startEdge = startEdge->NextInAEL;
    edge->NextInAEL = startEdge->NextInAEL;
    if(startEdge->NextInAEL) startEdge->NextInAEL->PrevInAEL = edge;
    edge->PrevInAEL = startEdge;
    startEdge->NextInAEL = edge;
  }
}
//----------------------------------------------------------------------

OutPt* DupOutPt(OutPt* outPt, bool InsertAfter)
{
  OutPt* result = new OutPt;
  result->Pt = outPt->Pt;
  result->Idx = outPt->Idx;
  if (InsertAfter)
  {
    result->Next = outPt->Next;
    result->Prev = outPt;
    outPt->Next->Prev = result;
    outPt->Next = result;
  } 
  else
  {
    result->Prev = outPt->Prev;
    result->Next = outPt;
    outPt->Prev->Next = result;
    outPt->Prev = result;
  }
  return result;
}
//------------------------------------------------------------------------------

bool JoinHorz(OutPt* op1, OutPt* op1b, OutPt* op2, OutPt* op2b,
  const FPoint Pt, bool DiscardLeft)
{
  Direction Dir1 = (op1->Pt.X > op1b->Pt.X ? dRightToLeft : dLeftToRight);
  Direction Dir2 = (op2->Pt.X > op2b->Pt.X ? dRightToLeft : dLeftToRight);
  if (Dir1 == Dir2) return false;

  //When DiscardLeft, we want Op1b to be on the Left of Op1, otherwise we
  //want Op1b to be on the Right. (And likewise with Op2 and Op2b.)
  //So, to facilitate this while inserting Op1b and Op2b ...
  //when DiscardLeft, make sure we're AT or RIGHT of Pt before adding Op1b,
  //otherwise make sure we're AT or LEFT of Pt. (Likewise with Op2b.)
  if (Dir1 == dLeftToRight) 
  {
    while (op1->Next->Pt.X <= Pt.X && 
      op1->Next->Pt.X >= op1->Pt.X && op1->Next->Pt.Y == Pt.Y)  
        op1 = op1->Next;
    if (DiscardLeft && (op1->Pt.X != Pt.X)) op1 = op1->Next;
    op1b = DupOutPt(op1, !DiscardLeft);
    if (op1b->Pt != Pt) 
    {
      op1 = op1b;
      op1->Pt = Pt;
      op1b = DupOutPt(op1, !DiscardLeft);
    }
  } 
  else
  {
    while (op1->Next->Pt.X >= Pt.X && 
      op1->Next->Pt.X <= op1->Pt.X && op1->Next->Pt.Y == Pt.Y) 
        op1 = op1->Next;
    if (!DiscardLeft && (op1->Pt.X != Pt.X)) op1 = op1->Next;
    op1b = DupOutPt(op1, DiscardLeft);
    if (op1b->Pt != Pt)
    {
      op1 = op1b;
      op1->Pt = Pt;
      op1b = DupOutPt(op1, DiscardLeft);
    }
  }

  if (Dir2 == dLeftToRight)
  {
    while (op2->Next->Pt.X <= Pt.X && 
      op2->Next->Pt.X >= op2->Pt.X && op2->Next->Pt.Y == Pt.Y)
        op2 = op2->Next;
    if (DiscardLeft && (op2->Pt.X != Pt.X)) op2 = op2->Next;
    op2b = DupOutPt(op2, !DiscardLeft);
    if (op2b->Pt != Pt)
    {
      op2 = op2b;
      op2->Pt = Pt;
      op2b = DupOutPt(op2, !DiscardLeft);
    };
  } else
  {
    while (op2->Next->Pt.X >= Pt.X && 
      op2->Next->Pt.X <= op2->Pt.X && op2->Next->Pt.Y == Pt.Y) 
        op2 = op2->Next;
    if (!DiscardLeft && (op2->Pt.X != Pt.X)) op2 = op2->Next;
    op2b = DupOutPt(op2, DiscardLeft);
    if (op2b->Pt != Pt)
    {
      op2 = op2b;
      op2->Pt = Pt;
      op2b = DupOutPt(op2, DiscardLeft);
    };
  };

  if ((Dir1 == dLeftToRight) == DiscardLeft)
  {
    op1->Prev = op2;
    op2->Next = op1;
    op1b->Next = op2b;
    op2b->Prev = op1b;
  }
  else
  {
    op1->Next = op2;
    op2->Prev = op1;
    op1b->Prev = op2b;
    op2b->Next = op1b;
  }
  return true;
}
//------------------------------------------------------------------------------

bool Clipper::JoinPoints(Join *j, OutRec* outRec1, OutRec* outRec2)
{
  OutPt *op1 = j->OutPt1, *op1b;
  OutPt *op2 = j->OutPt2, *op2b;

  //There are 3 kinds of joins for output polygons ...
  //1. Horizontal joins where Join.OutPt1 & Join.OutPt2 are a vertices anywhere
  //along (horizontal) collinear edges (& Join.OffPt is on the same horizontal).
  //2. Non-horizontal joins where Join.OutPt1 & Join.OutPt2 are at the same
  //location at the Bottom of the overlapping segment (& Join.OffPt is above).
  //3. StrictSimple joins where edges touch but are not collinear and where
  //Join.OutPt1, Join.OutPt2 & Join.OffPt all share the same point.
  bool isHorizontal = (j->OutPt1->Pt.Y == j->OffPt.Y);

  if (isHorizontal  && (j->OffPt == j->OutPt1->Pt) &&
  (j->OffPt == j->OutPt2->Pt))
  {
    //Strictly Simple join ...
    op1b = j->OutPt1->Next;
    while (op1b != op1 && (op1b->Pt == j->OffPt)) 
      op1b = op1b->Next;
    bool reverse1 = (op1b->Pt.Y > j->OffPt.Y);
    op2b = j->OutPt2->Next;
    while (op2b != op2 && (op2b->Pt == j->OffPt)) 
      op2b = op2b->Next;
    bool reverse2 = (op2b->Pt.Y > j->OffPt.Y);
    if (reverse1 == reverse2) return false;
    if (reverse1)
    {
      op1b = DupOutPt(op1, false);
      op2b = DupOutPt(op2, true);
      op1->Prev = op2;
      op2->Next = op1;
      op1b->Next = op2b;
      op2b->Prev = op1b;
      j->OutPt1 = op1;
      j->OutPt2 = op1b;
      return true;
    } else
    {
      op1b = DupOutPt(op1, true);
      op2b = DupOutPt(op2, false);
      op1->Next = op2;
      op2->Prev = op1;
      op1b->Prev = op2b;
      op2b->Next = op1b;
      j->OutPt1 = op1;
      j->OutPt2 = op1b;
      return true;
    }
  } 
  else if (isHorizontal)
  {
    //treat horizontal joins differently to non-horizontal joins since with
    //them we're not yet sure where the overlapping is. OutPt1.Pt & OutPt2.Pt
    //may be anywhere along the horizontal edge.
    op1b = op1;
    while (op1->Prev->Pt.Y == op1->Pt.Y && op1->Prev != op1b && op1->Prev != op2)
      op1 = op1->Prev;
    while (op1b->Next->Pt.Y == op1b->Pt.Y && op1b->Next != op1 && op1b->Next != op2)
      op1b = op1b->Next;
    if (op1b->Next == op1 || op1b->Next == op2) return false; //a flat 'polygon'

    op2b = op2;
    while (op2->Prev->Pt.Y == op2->Pt.Y && op2->Prev != op2b && op2->Prev != op1b)
      op2 = op2->Prev;
    while (op2b->Next->Pt.Y == op2b->Pt.Y && op2b->Next != op2 && op2b->Next != op1)
      op2b = op2b->Next;
    if (op2b->Next == op2 || op2b->Next == op1) return false; //a flat 'polygon'

    double Left, Right;
    //Op1 --> Op1b & Op2 --> Op2b are the extremites of the horizontal edges
    if (!GetOverlap(op1->Pt.X, op1b->Pt.X, op2->Pt.X, op2b->Pt.X, Left, Right))
      return false;

    //DiscardLeftSide: when overlapping edges are joined, a spike will created
    //which needs to be cleaned up. However, we don't want Op1 or Op2 caught up
    //on the discard Side as either may still be needed for other joins ...
    FPoint Pt;
    bool DiscardLeftSide;
    if (op1->Pt.X >= Left && op1->Pt.X <= Right) 
    {
      Pt = op1->Pt; DiscardLeftSide = (op1->Pt.X > op1b->Pt.X);
    } 
    else if (op2->Pt.X >= Left&& op2->Pt.X <= Right) 
    {
      Pt = op2->Pt; DiscardLeftSide = (op2->Pt.X > op2b->Pt.X);
    } 
    else if (op1b->Pt.X >= Left && op1b->Pt.X <= Right)
    {
      Pt = op1b->Pt; DiscardLeftSide = op1b->Pt.X > op1->Pt.X;
    } 
    else
    {
      Pt = op2b->Pt; DiscardLeftSide = (op2b->Pt.X > op2->Pt.X);
    }
    j->OutPt1 = op1; j->OutPt2 = op2;
    return JoinHorz(op1, op1b, op2, op2b, Pt, DiscardLeftSide);
  } else
  {
    //nb: For non-horizontal joins ...
    //    1. Jr.OutPt1.Pt.Y == Jr.OutPt2.Pt.Y
    //    2. Jr.OutPt1.Pt > Jr.OffPt.Y

    //make sure the polygons are correctly oriented ...
    op1b = op1->Next;
    while ((op1b->Pt == op1->Pt) && (op1b != op1)) op1b = op1b->Next;
    bool Reverse1 = ((op1b->Pt.Y > op1->Pt.Y) ||
      !SlopesEqual(op1->Pt, op1b->Pt, j->OffPt));
    if (Reverse1)
    {
      op1b = op1->Prev;
      while ((op1b->Pt == op1->Pt) && (op1b != op1)) op1b = op1b->Prev;
      if ((op1b->Pt.Y > op1->Pt.Y) ||
        !SlopesEqual(op1->Pt, op1b->Pt, j->OffPt)) return false;
    };
    op2b = op2->Next;
    while ((op2b->Pt == op2->Pt) && (op2b != op2))op2b = op2b->Next;
    bool Reverse2 = ((op2b->Pt.Y > op2->Pt.Y) ||
      !SlopesEqual(op2->Pt, op2b->Pt, j->OffPt));
    if (Reverse2)
    {
      op2b = op2->Prev;
      while ((op2b->Pt == op2->Pt) && (op2b != op2)) op2b = op2b->Prev;
      if ((op2b->Pt.Y > op2->Pt.Y) ||
        !SlopesEqual(op2->Pt, op2b->Pt, j->OffPt)) return false;
    }

    if ((op1b == op1) || (op2b == op2) || (op1b == op2b) ||
      ((outRec1 == outRec2) && (Reverse1 == Reverse2))) return false;

    if (Reverse1)
    {
      op1b = DupOutPt(op1, false);
      op2b = DupOutPt(op2, true);
      op1->Prev = op2;
      op2->Next = op1;
      op1b->Next = op2b;
      op2b->Prev = op1b;
      j->OutPt1 = op1;
      j->OutPt2 = op1b;
      return true;
    } else
    {
      op1b = DupOutPt(op1, true);
      op2b = DupOutPt(op2, false);
      op1->Next = op2;
      op2->Prev = op1;
      op1b->Prev = op2b;
      op2b->Next = op1b;
      j->OutPt1 = op1;
      j->OutPt2 = op1b;
      return true;
    }
  }
}
//----------------------------------------------------------------------

void Clipper::FixupFirstLefts1(OutRec* OldOutRec, OutRec* NewOutRec)
{ 
  
  for (PolyOutList::size_type i = 0; i < m_PolyOuts.size(); ++i)
  {
    OutRec* outRec = m_PolyOuts[i];
    if (outRec->Pts && outRec->FirstLeft == OldOutRec) 
    {
      if (Poly2ContainsPoly1(outRec->Pts, NewOutRec->Pts))
        outRec->FirstLeft = NewOutRec;
    }
  }
}
//----------------------------------------------------------------------

void Clipper::FixupFirstLefts2(OutRec* OldOutRec, OutRec* NewOutRec)
{ 
  for (PolyOutList::size_type i = 0; i < m_PolyOuts.size(); ++i)
  {
    OutRec* outRec = m_PolyOuts[i];
    if (outRec->FirstLeft == OldOutRec) outRec->FirstLeft = NewOutRec;
  }
}
//----------------------------------------------------------------------

static OutRec* ParseFirstLeft(OutRec* FirstLeft)
{
  while (FirstLeft && !FirstLeft->Pts) 
    FirstLeft = FirstLeft->FirstLeft;
  return FirstLeft;
}
//------------------------------------------------------------------------------

void Clipper::JoinCommonEdges()
{
  for (JoinList::size_type i = 0; i < m_Joins.size(); i++)
  {
    Join* join = m_Joins[i];

    OutRec *outRec1 = GetOutRec(join->OutPt1->Idx);
    OutRec *outRec2 = GetOutRec(join->OutPt2->Idx);

    if (!outRec1->Pts || !outRec2->Pts) continue;

    //get the polygon fragment with the correct hole state (FirstLeft)
    //before calling JoinPoints() ...
    OutRec *holeStateRec;
    if (outRec1 == outRec2) holeStateRec = outRec1;
    else if (Param1RightOfParam2(outRec1, outRec2)) holeStateRec = outRec2;
    else if (Param1RightOfParam2(outRec2, outRec1)) holeStateRec = outRec1;
    else holeStateRec = GetLowermostRec(outRec1, outRec2);

    if (!JoinPoints(join, outRec1, outRec2)) continue;

    if (outRec1 == outRec2)
    {
      //instead of joining two polygons, we've just created a new one by
      //splitting one polygon into two.
      outRec1->Pts = join->OutPt1;
      outRec1->BottomPt = 0;
      outRec2 = CreateOutRec();
      outRec2->Pts = join->OutPt2;

      //update all OutRec2.Pts Idx's ...
      UpdateOutPtIdxs(*outRec2);

      //We now need to check every OutRec.FirstLeft pointer. If it points
      //to OutRec1 it may need to point to OutRec2 instead ...
      if (m_UsingPolyTree)
        for (PolyOutList::size_type j = 0; j < m_PolyOuts.size() - 1; j++)
        {
          OutRec* oRec = m_PolyOuts[j];
          if (!oRec->Pts || ParseFirstLeft(oRec->FirstLeft) != outRec1 ||
            oRec->IsHole == outRec1->IsHole) continue;
          if (Poly2ContainsPoly1(oRec->Pts, join->OutPt2))
            oRec->FirstLeft = outRec2;
        }

      if (Poly2ContainsPoly1(outRec2->Pts, outRec1->Pts))
      {
        //outRec2 is contained by outRec1 ...
        outRec2->IsHole = !outRec1->IsHole;
        outRec2->FirstLeft = outRec1;

        //fixup FirstLeft pointers that may need reassigning to OutRec1
        if (m_UsingPolyTree) FixupFirstLefts2(outRec2, outRec1);

        if ((outRec2->IsHole ^ m_ReverseOutput) == (Area(*outRec2) > 0))
          ReversePolyPtLinks(outRec2->Pts);
            
      } else if (Poly2ContainsPoly1(outRec1->Pts, outRec2->Pts))
      {
        //outRec1 is contained by outRec2 ...
        outRec2->IsHole = outRec1->IsHole;
        outRec1->IsHole = !outRec2->IsHole;
        outRec2->FirstLeft = outRec1->FirstLeft;
        outRec1->FirstLeft = outRec2;

        //fixup FirstLeft pointers that may need reassigning to OutRec1
        if (m_UsingPolyTree) FixupFirstLefts2(outRec1, outRec2);

        if ((outRec1->IsHole ^ m_ReverseOutput) == (Area(*outRec1) > 0))
          ReversePolyPtLinks(outRec1->Pts);
      } 
      else
      {
        //the 2 polygons are completely separate ...
        outRec2->IsHole = outRec1->IsHole;
        outRec2->FirstLeft = outRec1->FirstLeft;

        //fixup FirstLeft pointers that may need reassigning to OutRec2
        if (m_UsingPolyTree) FixupFirstLefts1(outRec1, outRec2);
      }
     
    } else
    {
      //joined 2 polygons together ...

      outRec2->Pts = 0;
      outRec2->BottomPt = 0;
      outRec2->Idx = outRec1->Idx;

      outRec1->IsHole = holeStateRec->IsHole;
      if (holeStateRec == outRec2) 
        outRec1->FirstLeft = outRec2->FirstLeft;
      outRec2->FirstLeft = outRec1;

      //fixup FirstLeft pointers that may need reassigning to OutRec1
      if (m_UsingPolyTree) FixupFirstLefts2(outRec2, outRec1);
    }
  }
}

//------------------------------------------------------------------------------
// ClipperOffset support functions ...
//------------------------------------------------------------------------------

FPoint GetUnitNormal(const FPoint &pt1, const FPoint &pt2)
{
  if(pt2.X == pt1.X && pt2.Y == pt1.Y) 
    return FPoint(0, 0);

  double Dx = (pt2.X - pt1.X);
  double dy = (pt2.Y - pt1.Y);
  double f = 1 *1.0/ std::sqrt( Dx*Dx + dy*dy );
  Dx *= f;
  dy *= f;
  return FPoint(dy, -Dx);
}

//------------------------------------------------------------------------------
// ClipperOffset class
//------------------------------------------------------------------------------

ClipperOffset::ClipperOffset(double miterLimit, double arcTolerance)
{
  this->MiterLimit = miterLimit;
  this->ArcTolerance = arcTolerance;
  m_lowest.X = -1;
}
//------------------------------------------------------------------------------

ClipperOffset::~ClipperOffset()
{
  Clear();
}
//------------------------------------------------------------------------------

void ClipperOffset::Clear()
{
  for (int i = 0; i < m_polyNodes.ChildCount(); ++i)
    delete m_polyNodes.Childs[i];
  m_polyNodes.Childs.clear();
  m_lowest.X = -1;
}
//------------------------------------------------------------------------------

void ClipperOffset::AddPath(const Path& path, JoinType joinType, EndType endType)
{
  int highI = (int)path.size() - 1;
  if (highI < 0) return;
  PolyNode* newNode = new PolyNode();
  newNode->m_jointype = joinType;
  newNode->m_endtype = endType;

  //strip duplicate points from path and also get index to the lowest point ...
  if (endType == etClosedLine || endType == etClosedPolygon)
    while (highI > 0 && path[0] == path[highI]) highI--;
  newNode->Contour.reserve(highI + 1);
  newNode->Contour.push_back(path[0]);
  int j = 0, k = 0;
  for (int i = 1; i <= highI; i++)
    if (newNode->Contour[j] != path[i])
    {
      j++;
      newNode->Contour.push_back(path[i]);
      if (path[i].Y > newNode->Contour[k].Y ||
        (path[i].Y == newNode->Contour[k].Y &&
        path[i].X < newNode->Contour[k].X)) k = j;
    }
  if ((endType == etClosedPolygon && j < 2) || 
    (endType != etClosedPolygon && j < 0))
  {
    delete newNode;
    return;
  }
  m_polyNodes.AddChild(*newNode);

  //if this path's lowest pt is lower than all the others then update m_lowest
  if (endType != etClosedPolygon) return;
  if (m_lowest.X < 0)
    m_lowest = FPoint(0, k);
  else
  {
    FPoint ip = m_polyNodes.Childs[(int)m_lowest.X]->Contour[(int)m_lowest.Y];
    if (newNode->Contour[k].Y > ip.Y ||
      (newNode->Contour[k].Y == ip.Y &&
      newNode->Contour[k].X < ip.X))
      m_lowest = FPoint(m_polyNodes.ChildCount() - 1, k);
  }
}
//------------------------------------------------------------------------------

void ClipperOffset::AddPaths(const Paths& paths, JoinType joinType, EndType endType)
{
  for (Paths::size_type i = 0; i < paths.size(); ++i)
    AddPath(paths[i], joinType, endType);
}
//------------------------------------------------------------------------------

void ClipperOffset::FixOrientations()
{
  //fixup orientations of all closed paths if the orientation of the
  //closed path with the lowermost vertex is wrong ...
  if (m_lowest.X >= 0 && 
    !Orientation(m_polyNodes.Childs[(int)m_lowest.X]->Contour))
  {
    for (int i = 0; i < m_polyNodes.ChildCount(); ++i)
    {
      PolyNode& node = *m_polyNodes.Childs[i];
      if (node.m_endtype == etClosedPolygon ||
        (node.m_endtype == etClosedLine && Orientation(node.Contour)))
          ReversePath(node.Contour);
    }
  } else
  {
    for (int i = 0; i < m_polyNodes.ChildCount(); ++i)
    {
      PolyNode& node = *m_polyNodes.Childs[i];
      if (node.m_endtype == etClosedLine && !Orientation(node.Contour))
        ReversePath(node.Contour);
    }
  }
}
//------------------------------------------------------------------------------

void ClipperOffset::Execute(Paths& solution, double delta)
{
  solution.clear();
  FixOrientations();
  DoOffset(delta);
  
  //now clean up 'corners' ...
  Clipper clpr;
  clpr.AddPaths(m_destPolys, ptSubject, true);
  if (delta > 0)
  {
    clpr.Execute(ctUnion, solution, pftPositive, pftPositive);
  }
  else
  {
    FRect r = clpr.GetBounds();
    Path outer(4);
    outer[0] = FPoint(r.left - 10, r.bottom + 10);
    outer[1] = FPoint(r.right + 10, r.bottom + 10);
    outer[2] = FPoint(r.right + 10, r.top - 10);
    outer[3] = FPoint(r.left - 10, r.top - 10);

    clpr.AddPath(outer, ptSubject, true);
    clpr.ReverseSolution(true);
    clpr.Execute(ctUnion, solution, pftNegative, pftNegative);
    if (solution.size() > 0) solution.erase(solution.begin());
  }
}
//------------------------------------------------------------------------------

void ClipperOffset::Execute(PolyTree& solution, double delta)
{
  solution.Clear();
  FixOrientations();
  DoOffset(delta);

  //now clean up 'corners' ...
  Clipper clpr;
  clpr.AddPaths(m_destPolys, ptSubject, true);
  if (delta > 0)
  {
    clpr.Execute(ctUnion, solution, pftPositive, pftPositive);
  }
  else
  {
    FRect r = clpr.GetBounds();
    Path outer(4);
    outer[0] = FPoint(r.left - 10, r.bottom + 10);
    outer[1] = FPoint(r.right + 10, r.bottom + 10);
    outer[2] = FPoint(r.right + 10, r.top - 10);
    outer[3] = FPoint(r.left - 10, r.top - 10);

    clpr.AddPath(outer, ptSubject, true);
    clpr.ReverseSolution(true);
    clpr.Execute(ctUnion, solution, pftNegative, pftNegative);
    //remove the outer PolyNode rectangle ...
    if (solution.ChildCount() == 1 && solution.Childs[0]->ChildCount() > 0)
    {
      PolyNode* outerNode = solution.Childs[0];
      solution.Childs.reserve(outerNode->ChildCount());
      solution.Childs[0] = outerNode->Childs[0];
      for (int i = 1; i < outerNode->ChildCount(); ++i)
        solution.AddChild(*outerNode->Childs[i]);
    }
    else
      solution.Clear();
  }
}
//------------------------------------------------------------------------------

void ClipperOffset::DoOffset(double delta)
{
  m_destPolys.clear();
  m_delta = delta;

  //if Zero offset, just copy any CLOSED polygons to m_p and return ...
  double absDelta = std::fabs(delta);
  if (absDelta < 1.0e-06) 
  {
    m_destPolys.reserve(m_polyNodes.ChildCount());
    for (int i = 0; i < m_polyNodes.ChildCount(); i++)
    {
      PolyNode& node = *m_polyNodes.Childs[i];
      if (node.m_endtype == etClosedPolygon)
        m_destPolys.push_back(node.Contour);
    }
    return;
  }

  //see offset_triginometry3.svg in the documentation folder ...
  if (MiterLimit > 2) m_miterLim = 2/(MiterLimit * MiterLimit);
  else m_miterLim = 0.5;

  double y;
  if (ArcTolerance <= 0.0) y = def_arc_tolerance;
  else if (ArcTolerance > absDelta * def_arc_tolerance) 
    y = absDelta * def_arc_tolerance;
  else y = ArcTolerance;
  //see offset_triginometry2.svg in the documentation folder ...
  double steps = pi / std::acos(1 - y / absDelta);
  if (steps > absDelta * pi) 
    steps = absDelta * pi;  //ie excessive precision check
  m_sin = std::sin(two_pi / steps);
  m_cos = std::cos(two_pi / steps);
  m_StepsPerRad = steps / two_pi;
  if (delta < 0.0) m_sin = -m_sin;

  m_destPolys.reserve(m_polyNodes.ChildCount() * 2);
  for (int i = 0; i < m_polyNodes.ChildCount(); i++)
  {
    PolyNode& node = *m_polyNodes.Childs[i];
    m_srcPoly = node.Contour;

    int len = (int)m_srcPoly.size();
    if (len == 0 || (delta <= 0 && (len < 3 || node.m_endtype != etClosedPolygon)))
        continue;

    m_destPoly.clear();
    if (len == 1)
    {
      if (node.m_jointype == jtRound)
      {
        double X = 1.0, Y = 0.0;
        for (double j = 1; j <= steps; j++)
        {
          m_destPoly.push_back(FPoint(
            m_srcPoly[0].X + X * delta,
            m_srcPoly[0].Y + Y * delta));
          double X2 = X;
          X = X * m_cos - m_sin * Y;
          Y = X2 * m_sin + Y * m_cos;
        }
      }
      else
      {
        double X = -1.0, Y = -1.0;
        for (int j = 0; j < 4; ++j)
        {
          m_destPoly.push_back(FPoint(
            m_srcPoly[0].X + X * delta,
            m_srcPoly[0].Y + Y * delta));
          if (X < 0) X = 1;
          else if (Y < 0) Y = 1;
          else X = -1;
        }
      }
      m_destPolys.push_back(m_destPoly);
      continue;
    }
    //build m_normals ...
    m_normals.clear();
    m_normals.reserve(len);
    for (int j = 0; j < len - 1; ++j)
      m_normals.push_back(GetUnitNormal(m_srcPoly[j], m_srcPoly[j + 1]));
    if (node.m_endtype == etClosedLine || node.m_endtype == etClosedPolygon)
      m_normals.push_back(GetUnitNormal(m_srcPoly[len - 1], m_srcPoly[0]));
    else
      m_normals.push_back(FPoint(m_normals[len - 2]));

    if (node.m_endtype == etClosedPolygon)
    {
      int k = len - 1;
      for (int j = 0; j < len; ++j)
        OffsetPoint(j, k, node.m_jointype);
      m_destPolys.push_back(m_destPoly);
    }
    else if (node.m_endtype == etClosedLine)
    {
      int k = len - 1;
      for (int j = 0; j < len; ++j)
        OffsetPoint(j, k, node.m_jointype);
      m_destPolys.push_back(m_destPoly);
      m_destPoly.clear();
      //re-build m_normals ...
      FPoint n = m_normals[len -1];
      for (int j = len - 1; j > 0; j--)
        m_normals[j] = FPoint(-m_normals[j - 1].X, -m_normals[j - 1].Y);
      m_normals[0] = FPoint(-n.X, -n.Y);
      k = 0;
      for (int j = len - 1; j >= 0; j--)
        OffsetPoint(j, k, node.m_jointype);
      m_destPolys.push_back(m_destPoly);
    }
    else
    {
      int k = 0;
      for (int j = 1; j < len - 1; ++j)
        OffsetPoint(j, k, node.m_jointype);

      FPoint pt1;
      if (node.m_endtype == etOpenButt)
      {
        int j = len - 1;
        pt1 = FPoint(m_srcPoly[j].X + m_normals[j].X * delta, 
          m_srcPoly[j].Y + m_normals[j].Y * delta);
        m_destPoly.push_back(pt1);
        pt1 = FPoint(m_srcPoly[j].X - m_normals[j].X * delta, 
          m_srcPoly[j].Y - m_normals[j].Y * delta);
        m_destPoly.push_back(pt1);
      }
      else
      {
        int j = len - 1;
        k = len - 2;
        m_sinA = 0;
        m_normals[j] = FPoint(-m_normals[j].X, -m_normals[j].Y);
        if (node.m_endtype == etOpenSquare)
          DoSquare(j, k);
        else
          DoRound(j, k);
      }

      //re-build m_normals ...
      for (int j = len - 1; j > 0; j--)
        m_normals[j] = FPoint(-m_normals[j - 1].X, -m_normals[j - 1].Y);
      m_normals[0] = FPoint(-m_normals[1].X, -m_normals[1].Y);

      k = len - 1;
      for (int j = k - 1; j > 0; --j) OffsetPoint(j, k, node.m_jointype);

      if (node.m_endtype == etOpenButt)
      {
        pt1 = FPoint(m_srcPoly[0].X - m_normals[0].X * delta,
          m_srcPoly[0].Y - m_normals[0].Y * delta);
        m_destPoly.push_back(pt1);
        pt1 = FPoint(m_srcPoly[0].X + m_normals[0].X * delta,
          m_srcPoly[0].Y + m_normals[0].Y * delta);
        m_destPoly.push_back(pt1);
      }
      else
      {
        k = 1;
        m_sinA = 0;
        if (node.m_endtype == etOpenSquare)
          DoSquare(0, 1);
        else
          DoRound(0, 1);
      }
      m_destPolys.push_back(m_destPoly);
    }
  }
}
//------------------------------------------------------------------------------

void ClipperOffset::OffsetPoint(int j, int& k, JoinType jointype)
{
  m_sinA = (m_normals[k].X * m_normals[j].Y - m_normals[j].X * m_normals[k].Y);
  if (m_sinA < 0.00005 && m_sinA > -0.00005) return;
  else if (m_sinA > 1.0) m_sinA = 1.0;
  else if (m_sinA < -1.0) m_sinA = -1.0;

  if (m_sinA * m_delta < 0)
  {
    m_destPoly.push_back(FPoint(m_srcPoly[j].X + m_normals[k].X * m_delta,
      m_srcPoly[j].Y + m_normals[k].Y * m_delta));
    m_destPoly.push_back(m_srcPoly[j]);
    m_destPoly.push_back(FPoint(m_srcPoly[j].X + m_normals[j].X * m_delta,
      m_srcPoly[j].Y + m_normals[j].Y * m_delta));
  }
  else
    switch (jointype)
    {
      case jtMiter:
        {
          double r = 1 + (m_normals[j].X * m_normals[k].X +
            m_normals[j].Y * m_normals[k].Y);
          if (r >= m_miterLim) DoMiter(j, k, r); else DoSquare(j, k);
          break;
        }
      case jtSquare: DoSquare(j, k); break;
      case jtRound: DoRound(j, k); break;
    }
  k = j;
}
//------------------------------------------------------------------------------

void ClipperOffset::DoSquare(int j, int k)
{
  double dx = std::tan(std::atan2(m_sinA,
      m_normals[k].X * m_normals[j].X + m_normals[k].Y * m_normals[j].Y) / 4);
  m_destPoly.push_back(FPoint(
      m_srcPoly[j].X + m_delta * (m_normals[k].X - m_normals[k].Y * dx),
      m_srcPoly[j].Y + m_delta * (m_normals[k].Y + m_normals[k].X * dx)));
  m_destPoly.push_back(FPoint(
      m_srcPoly[j].X + m_delta * (m_normals[j].X + m_normals[j].Y * dx),
      m_srcPoly[j].Y + m_delta * (m_normals[j].Y - m_normals[j].X * dx)));
}
//------------------------------------------------------------------------------

void ClipperOffset::DoMiter(int j, int k, double r)
{
  double q = m_delta / r;
  m_destPoly.push_back(FPoint(m_srcPoly[j].X + (m_normals[k].X + m_normals[j].X) * q,
      m_srcPoly[j].Y + (m_normals[k].Y + m_normals[j].Y) * q));
}
//------------------------------------------------------------------------------

void ClipperOffset::DoRound(int j, int k)
{
  double a = std::atan2(m_sinA,
  m_normals[k].X * m_normals[j].X + m_normals[k].Y * m_normals[j].Y);
  int steps = (int)Round(m_StepsPerRad * std::fabs(a));

  double X = m_normals[k].X, Y = m_normals[k].Y, X2;
  for (int i = 0; i < steps; ++i)
  {
    m_destPoly.push_back(FPoint(
        m_srcPoly[j].X + X * m_delta,
        m_srcPoly[j].Y + Y * m_delta));
    X2 = X;
    X = X * m_cos - m_sin * Y;
    Y = X2 * m_sin + Y * m_cos;
  }
  m_destPoly.push_back(FPoint(
  m_srcPoly[j].X + m_normals[j].X * m_delta,
  m_srcPoly[j].Y + m_normals[j].Y * m_delta));
}

//------------------------------------------------------------------------------
// Miscellaneous public functions
//------------------------------------------------------------------------------

void Clipper::DoSimplePolygons()
{
  PolyOutList::size_type i = 0;
  while (i < m_PolyOuts.size()) 
  {
    OutRec* outrec = m_PolyOuts[i++];
    OutPt* op = outrec->Pts;
    if (!op) continue;
    do //for each Pt in Polygon until duplicate found do ...
    {
      OutPt* op2 = op->Next;
      while (op2 != outrec->Pts) 
      {
        if ((op->Pt == op2->Pt) && op2->Next != op && op2->Prev != op) 
        {
          //split the polygon into two ...
          OutPt* op3 = op->Prev;
          OutPt* op4 = op2->Prev;
          op->Prev = op4;
          op4->Next = op;
          op2->Prev = op3;
          op3->Next = op2;

          outrec->Pts = op;
          OutRec* outrec2 = CreateOutRec();
          outrec2->Pts = op2;
          UpdateOutPtIdxs(*outrec2);
          if (Poly2ContainsPoly1(outrec2->Pts, outrec->Pts))
          {
            //OutRec2 is contained by OutRec1 ...
            outrec2->IsHole = !outrec->IsHole;
            outrec2->FirstLeft = outrec;
          }
          else
            if (Poly2ContainsPoly1(outrec->Pts, outrec2->Pts))
          {
            //OutRec1 is contained by OutRec2 ...
            outrec2->IsHole = outrec->IsHole;
            outrec->IsHole = !outrec2->IsHole;
            outrec2->FirstLeft = outrec->FirstLeft;
            outrec->FirstLeft = outrec2;
          } else
          {
            //the 2 polygons are separate ...
            outrec2->IsHole = outrec->IsHole;
            outrec2->FirstLeft = outrec->FirstLeft;
          }
          op2 = op; //ie get ready for the Next iteration
        }
        op2 = op2->Next;
      }
      op = op->Next;
    }
    while (op != outrec->Pts);
  }
}
//------------------------------------------------------------------------------

void ReversePath(Path& p)
{
  std::reverse(p.begin(), p.end());
}
//------------------------------------------------------------------------------

void ReversePaths(Paths& p)
{
  for (Paths::size_type i = 0; i < p.size(); ++i)
    ReversePath(p[i]);
}
//------------------------------------------------------------------------------

void SimplifyPolygon(const Path &in_poly, Paths &out_polys, PolyFillType fillType)
{
  Clipper c;
  c.StrictlySimple(true);
  c.AddPath(in_poly, ptSubject, true);
  c.Execute(ctUnion, out_polys, fillType, fillType);
}
//------------------------------------------------------------------------------

void SimplifyPolygons(const Paths &in_polys, Paths &out_polys, PolyFillType fillType)
{
  Clipper c;
  c.StrictlySimple(true);
  c.AddPaths(in_polys, ptSubject, true);
  c.Execute(ctUnion, out_polys, fillType, fillType);
}
//------------------------------------------------------------------------------

void SimplifyPolygons(Paths &polys, PolyFillType fillType)
{
  SimplifyPolygons(polys, polys, fillType);
}
//------------------------------------------------------------------------------

inline double DistanceSqrd(const FPoint& pt1, const FPoint& pt2)
{
  double Dx = (pt1.X - pt2.X);
  double dy = (pt1.Y - pt2.Y);
  return (Dx*Dx + dy*dy);
}
//------------------------------------------------------------------------------

double DistanceFromLineSqrd(
  const FPoint& pt, const FPoint& ln1, const FPoint& ln2)
{
  //The equation of a line in general form (Ax + By + C = 0)
  //given 2 points (x�,y�) & (x�,y�) is ...
  //(y� - y�)x + (x� - x�)y + (y� - y�)x� - (x� - x�)y� = 0
  //A = (y� - y�); B = (x� - x�); C = (y� - y�)x� - (x� - x�)y�
  //perpendicular distance of point (x�,y�) = (Ax� + By� + C)/Sqrt(A� + B�)
  //see http://en.wikipedia.org/wiki/Perpendicular_distance
  double A = double(ln1.Y - ln2.Y);
  double B = double(ln2.X - ln1.X);
  double C = A * ln1.X  + B * ln1.Y;
  C = A * pt.X + B * pt.Y - C;
  return (C * C) / (A * A + B * B);
}
//---------------------------------------------------------------------------

bool SlopesNearCollinear(const FPoint& pt1, 
    const FPoint& pt2, const FPoint& pt3, double distSqrd)
{
  return DistanceFromLineSqrd(pt2, pt1, pt3) < distSqrd;
}
//------------------------------------------------------------------------------

bool PointsAreClose(FPoint pt1, FPoint pt2, double distSqrd)
{
    double Dx = pt1.X - pt2.X;
    double dy = pt1.Y - pt2.Y;
    return ((Dx * Dx) + (dy * dy) <= distSqrd);
}
//------------------------------------------------------------------------------

OutPt* ExcludeOp(OutPt* op)
{
  OutPt* result = op->Prev;
  result->Next = op->Next;
  op->Next->Prev = result;
  result->Idx = 0;
  return result;
}
//------------------------------------------------------------------------------

void CleanPolygon(const Path& in_poly, Path& out_poly, double distance)
{
  //distance = proximity in units/pixels below which vertices
  //will be stripped. Default ~= sqrt(2).
  
  size_t size = in_poly.size();
  
  if (size == 0) 
  {
    out_poly.clear();
    return;
  }

  OutPt* outPts = new OutPt[size];
  for (size_t i = 0; i < size; ++i)
  {
    outPts[i].Pt = in_poly[i];
    outPts[i].Next = &outPts[(i + 1) % size];
    outPts[i].Next->Prev = &outPts[i];
    outPts[i].Idx = 0;
  }

  double distSqrd = distance * distance;
  OutPt* op = &outPts[0];
  while (op->Idx == 0 && op->Next != op->Prev) 
  {
    if (PointsAreClose(op->Pt, op->Prev->Pt, distSqrd))
    {
      op = ExcludeOp(op);
      size--;
    } 
    else if (PointsAreClose(op->Prev->Pt, op->Next->Pt, distSqrd))
    {
      ExcludeOp(op->Next);
      op = ExcludeOp(op);
      size -= 2;
    }
    else if (SlopesNearCollinear(op->Prev->Pt, op->Pt, op->Next->Pt, distSqrd))
    {
      op = ExcludeOp(op);
      size--;
    }
    else
    {
      op->Idx = 1;
      op = op->Next;
    }
  }

  if (size < 3) size = 0;
  out_poly.resize(size);
  for (size_t i = 0; i < size; ++i)
  {
    out_poly[i] = op->Pt;
    op = op->Next;
  }
  delete [] outPts;
}
//------------------------------------------------------------------------------

void CleanPolygon(Path& poly, double distance)
{
  CleanPolygon(poly, poly, distance);
}
//------------------------------------------------------------------------------

void CleanPolygons(const Paths& in_polys, Paths& out_polys, double distance)
{
  for (Paths::size_type i = 0; i < in_polys.size(); ++i)
    CleanPolygon(in_polys[i], out_polys[i], distance);
}
//------------------------------------------------------------------------------

void CleanPolygons(Paths& polys, double distance)
{
  CleanPolygons(polys, polys, distance);
}
//------------------------------------------------------------------------------

void Minkowski(const Path& poly, const Path& path, 
  Paths& solution, bool isSum, bool isClosed)
{
  int delta = (isClosed ? 1 : 0);
  size_t polyCnt = poly.size();
  size_t pathCnt = path.size();
  Paths pp;
  pp.reserve(pathCnt);
  if (isSum)
    for (size_t i = 0; i < pathCnt; ++i)
    {
      Path p;
      p.reserve(polyCnt);
      for (size_t j = 0; j < poly.size(); ++j)
        p.push_back(FPoint(path[i].X + poly[j].X, path[i].Y + poly[j].Y));
      pp.push_back(p);
    }
  else
    for (size_t i = 0; i < pathCnt; ++i)
    {
      Path p;
      p.reserve(polyCnt);
      for (size_t j = 0; j < poly.size(); ++j)
        p.push_back(FPoint(path[i].X - poly[j].X, path[i].Y - poly[j].Y));
      pp.push_back(p);
    }

  Paths quads; 
  quads.reserve((pathCnt + delta) * (polyCnt + 1));
  for (size_t i = 0; i <= pathCnt - 2 + delta; ++i)
    for (size_t j = 0; j <= polyCnt - 1; ++j)
    {
      Path quad;
      quad.reserve(4);
      quad.push_back(pp[i % pathCnt][j % polyCnt]);
      quad.push_back(pp[(i + 1) % pathCnt][j % polyCnt]);
      quad.push_back(pp[(i + 1) % pathCnt][(j + 1) % polyCnt]);
      quad.push_back(pp[i % pathCnt][(j + 1) % polyCnt]);
      if (!Orientation(quad)) ReversePath(quad);
      quads.push_back(quad);
    }

  Clipper c;
  c.AddPaths(quads, ptSubject, true);
  c.Execute(ctUnion, solution, pftNonZero, pftNonZero);
}
//------------------------------------------------------------------------------

void MinkowskiSum(const Path& poly, const Path& path, Paths& solution, bool isClosed)
{
  Minkowski(poly, path, solution, true, isClosed);
}
//------------------------------------------------------------------------------

void MinkowskiDiff(const Path& poly, const Path& path, Paths& solution, bool isClosed)
{
  Minkowski(poly, path, solution, false, isClosed);
}
//------------------------------------------------------------------------------

enum NodeType {ntAny, ntOpen, ntClosed};

void AddPolyNodeToPolygons(const PolyNode& polynode, NodeType nodetype, Paths& paths)
{
  bool match = true;
  if (nodetype == ntClosed) match = !polynode.IsOpen();
  else if (nodetype == ntOpen) return;

  if (!polynode.Contour.empty() && match)
    paths.push_back(polynode.Contour);
  for (int i = 0; i < polynode.ChildCount(); ++i)
    AddPolyNodeToPolygons(*polynode.Childs[i], nodetype, paths);
}
//------------------------------------------------------------------------------

void PolyTreeToPaths(const PolyTree& polytree, Paths& paths)
{
  paths.resize(0); 
  paths.reserve(polytree.Total());
  AddPolyNodeToPolygons(polytree, ntAny, paths);
}
//------------------------------------------------------------------------------

void ClosedPathsFromPolyTree(const PolyTree& polytree, Paths& paths)
{
  paths.resize(0); 
  paths.reserve(polytree.Total());
  AddPolyNodeToPolygons(polytree, ntClosed, paths);
}
//------------------------------------------------------------------------------

void OpenPathsFromPolyTree(PolyTree& polytree, Paths& paths)
{
  paths.resize(0); 
  paths.reserve(polytree.Total());
  //Open paths are top level only, so ...
  for (int i = 0; i < polytree.ChildCount(); ++i)
    if (polytree.Childs[i]->IsOpen())
      paths.push_back(polytree.Childs[i]->Contour);
}
//------------------------------------------------------------------------------

std::ostream& operator <<(std::ostream &s, const FPoint &p)
{
  s << "(" << p.X << "," << p.Y << ")";
  return s;
}
//------------------------------------------------------------------------------

std::ostream& operator <<(std::ostream &s, const Path &p)
{
  if (p.empty()) return s;
  Path::size_type last = p.size() -1;
  for (Path::size_type i = 0; i < last; i++)
    s << "(" << p[i].X << "," << p[i].Y << "), ";
  s << "(" << p[last].X << "," << p[last].Y << ")\n";
  return s;
}
//------------------------------------------------------------------------------

std::ostream& operator <<(std::ostream &s, const Paths &p)
{
  for (Paths::size_type i = 0; i < p.size(); i++)
    s << p[i];
  s << "\n";
  return s;
}
//------------------------------------------------------------------------------

#ifdef use_deprecated

void OffsetPaths(const Paths &in_polys, Paths &out_polys,
  double delta, JoinType jointype, EndType_ endtype, double limit)
{
  ClipperOffset co(limit, limit);
  co.AddPaths(in_polys, jointype, (EndType)endtype); 
  co.Execute(out_polys, delta);
}
//------------------------------------------------------------------------------

#endif


} //ClipperLib namespace
