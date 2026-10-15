#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> TDS;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
typedef CDT::Point cdtPoint;
typedef CDT::Vertex_handle Vertex_handle;

#include <cassert>
#include <iostream>

#include <algorithm>
#include <map>
using namespace std;

#include "cdt2.h"
#include <cmath> // PATCH(dude-0b)
#include "intersection.h"
#include "polygon.h"

// PATCH(dude-0b): CGAL's edge order and orientation depend on the heap
// layout (the triangulation does not); the edges are oriented from the
// smaller end point and sorted by (x1, y1, x2, y2), a NaN coordinate last.
// The hook (null; tests only) may reorder them first or skip the sort.
typedef pair<ply_vertex *, ply_vertex *> cdt_edge;
void (*g_cdt_edge_test_hook)(vector<cdt_edge> &, bool &) = NULL;
static bool pointLess(const Point2d &a, const Point2d &b) {
  for (int i = 0; i < 2; i++) // both NaN or equal: the next coordinate
    if (std::isnan(a[i]) != std::isnan(b[i]) || (!std::isnan(a[i]) && a[i] != b[i]))
      return std::isnan(b[i]) || a[i] < b[i];
  return false;
}
static bool edgeLess(const cdt_edge &a, const cdt_edge &b) {
  const Point2d &a1 = a.first->getPos(), &b1 = b.first->getPos();
  if (pointLess(a1, b1) || pointLess(b1, a1))
    return pointLess(a1, b1);
  return pointLess(a.second->getPos(), b.second->getPos());
}

bool c_cdt::build(c_dude &ap, c_polygon &sp, c_polygon &initPolygon) {
  CDT cdt;
  map<const ply_vertex *, Vertex_handle> v2cgal;
  map<Vertex_handle, ply_vertex *> cgal2v;

  // register vertices of p
  uint vsize = sp.getSize();
  for (uint i = 0; i < vsize; i++) {
    const Point2d &pt = sp[i]->getPos();
    cdtPoint cdt_pt(pt[0], pt[1]);
    Vertex_handle handler = cdt.push_back(cdt_pt);
    v2cgal[sp[i]] = handler;
    cgal2v[handler] = sp[i];
  }

  // add constraints to CGAL CDT
  for (c_polygon::const_iterator i = sp.begin(); i != sp.end(); i++) {
    ply_vertex *ptr = i->getHead();
    do {
      // add polygon boundary
      ply_vertex *next = ptr->getNext();
      Vertex_handle h1 = v2cgal[ptr];
      Vertex_handle h2 = v2cgal[next];
      cdt.insert_constraint(h1, h2);
      ptr = next;
    } while (ptr != i->getHead());
  }

  // add existing diagonals as additional constraints
  const vector<c_diagonal> &diagonals = ap.getDiagonals();
  for (vector<c_diagonal>::const_iterator i = diagonals.begin();
       i != diagonals.end(); i++) {
    // cout<<"the polygon already has some cuts before delaunay
    // triangulation"<<endl;
    Vertex_handle h1 = v2cgal[i->getV1()];
    Vertex_handle h2 = v2cgal[i->getV2()];
    cdt.insert_constraint(h1, h2);
  }

  // get diagonals back from CDT
  if (!cdt.is_valid()) {
    cerr << "! Warning: GDT failed" << endl;
    return false;
  }

  //
  typedef CDT::Finite_edges_iterator IT;
  vector<cdt_edge> edges; // PATCH(dude-0b): sorted below
  for (IT eit = cdt.finite_edges_begin(); eit != cdt.finite_edges_end();
       ++eit) {
    if (cdt.is_constrained(*eit) == false) // not a constraint
    {
      Vertex_handle v1 = eit->first->vertex(cdt.cw(eit->second));
      Vertex_handle v2 = eit->first->vertex(cdt.ccw(eit->second));

      ply_vertex *pv1 = cgal2v[v1];
      ply_vertex *pv2 = cgal2v[v2];

      if (pv1 == NULL || pv2 == NULL)
        continue;

      if (pointLess(pv2->getPos(), pv1->getPos())) // PATCH(dude-0b)
        swap(pv1, pv2);
      edges.push_back(cdt_edge(pv1, pv2));
      //			//ap.addDiagonal(pv1,pv2,sp);
      //            if( ap.isDiagonal(pv1,pv2,sp) )
      //            {
      //                ap.addDiagonal(pv1,pv2,sp);
      //            }
    }
  }
  bool sort_edges = true; // PATCH(dude-0b)
  if (g_cdt_edge_test_hook != NULL)
    g_cdt_edge_test_hook(edges, sort_edges);
  if (sort_edges)
    stable_sort(edges.begin(), edges.end(), edgeLess);
  for (size_t i = 0; i < edges.size(); i++)
    if (ap.isDiagonal(edges[i].first, edges[i].second, sp, sp))
      ap.addDiagonal(edges[i].first, edges[i].second, sp);

  return true;
}
