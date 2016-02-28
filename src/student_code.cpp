/*
 * Student solution for UC Berkeley Project 2
 *
 * Implemented by ____ on ____.
 *
 */

#include "student_code.h"
#include "mutablePriorityQueue.h"

namespace CGL {

    void BezierPatch::preprocess() {
        // TODO Part 1.
        // TODO If you use the matrix form for Bezier patch evaluation, you will need to
        // TODO compute your matrices based on the 16 control points here. 
        // TODO You will also need to define your matrices
        // TODO as member variables in the "BezierPatch" class.
        // TODO If you use De Casteljau's recursive algorithm, you will not need to do anything here.

    }
    
    Vector3D BezierPatch::lerp(Vector3D v1, Vector3D v2, double t) const{
        return v1*(1-t) + v2*t;
    }

    Vector3D BezierPatch::evaluate(double u, double v) const {
        // TODO Part 1.
        // TODO Returns the 3D point whose parametric coordinates are (u, v) on the Bezier patch.
        // TODO Note that both u and v are within [0, 1].
        Vector3D evalPoints[4][4][4];
        int n = 3;
        for (int r = 0; r <= n; r++) {
          for (int i = 0; i <= n-r; i++) {
            for (int j = 0; j <= n-r; j++) {
              if (r == 0) {
                evalPoints[r][i][j] = controlPoints[i][j];
              } else {
                Vector3D i_i1_j = lerp(evalPoints[r-1][i][j], evalPoints[r-1][i+1][j], u);
                Vector3D i_i1_j1 = lerp(evalPoints[r-1][i][j+1], evalPoints[r-1][i+1][j+1], u);
                evalPoints[r][i][j] = lerp(i_i1_j, i_i1_j1, v);
              }
            }
          }
        } 
        return evalPoints[n][0][0];
    }

    void BezierPatch::add2mesh(Polymesh* mesh) const {
        // TODO Part 1.
        // TODO Tessellate the given Bezier patch into triangles uniformly on a 8x8 grid(8x8x2=128 triangles) in parameter space.
        // TODO You will call your own evaluate function here to compute vertex positions of the tessellated triangles.
        // TODO The "addTriangle" function inherited from the "BezierPatchLoader" class may help you add triangles to the output mesh. 
      int n = 8;
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
          double u = i/(float)n;
          double u1 = (i+1)/(float)n;
          double v = j/(float)n;
          double v1 = (j+1)/(float)n;
          Vector3D uv = evaluate(u,v);
          Vector3D u1v = evaluate(u1,v);
          Vector3D uv1 = evaluate(u,v1);
          Vector3D u1v1 = evaluate(u1,v1);
          addTriangle(mesh, uv, u1v, uv1);
          addTriangle(mesh, u1v1, uv1, u1v);
        }
      }
    }

    Vector3D Vertex::normal(void) const
    // TODO Part 2.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    {
        // TODO Compute and return the area-weighted unit normal.
        Vector3D n(0,0,0);
        HalfedgeCIter h = halfedge();
        HalfedgeCIter h_ori = h;
        do {
            HalfedgeCIter t = h->twin();
            h = t->next();
            Vector3D ei = t->vertex()->position - position;
            Vector3D ej = h->twin()->vertex()->position - position;
            n += cross(ei, ej);
        } while (h != h_ori);
        return n.unit();
    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
        // TODO Part 3.
        // TODO This method should flip the given edge and return an iterator to the flipped edge.
        if ((e0->halfedge()->isBoundary()) || (e0->halfedge()->twin()->isBoundary())) {
           return EdgeIter();
        }
        // abc/dcb -> abd/dca, assuming cb = e0->halfedge
        HalfedgeIter he = e0->halfedge(); 
        HalfedgeIter twin = e0->halfedge()->twin();
        HalfedgeIter bd = he->next();
        HalfedgeIter dc = bd->next();
        HalfedgeIter ca = twin->next();
        HalfedgeIter ab = ca->next();
        HalfedgeIter db = bd->twin();
        HalfedgeIter ac = ca->twin();
        FaceIter f1 = he->face();
        FaceIter f2 = twin->face();
        VertexIter a = ab->vertex();
        VertexIter b = bd->vertex();
        VertexIter c = ca->vertex();
        VertexIter d = dc->vertex();
        
        f1->halfedge() = he;
        f2->halfedge() = twin;
        c->halfedge() = ca;
        b->halfedge() = bd;
        
        he->next() = dc;
        dc->next() = ca;
        ca->next() = he;
        twin->next() = ab;
        ab->next() = bd;
        bd->next() = twin;
        he->vertex() = a;
        twin->vertex() = d;

        ca->face() = f1;
        ac->face() = f1;
        bd->face() = f2;
        db->face() = f2;
        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
        // TODO Part 4.
        // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
        // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
        // abc/dcb split at m,
        // Assume cb is e0->halfedge()
        /*     c                c
         *    /|\              /|\
         *  /f2|f1\          /f2|f1\
         * a   |   d  ===>  a---m---d
         *  \  |  /          \f3|f4/
         *    \|/              \|/
         *     b                b
         */
        if ((e0->halfedge()->isBoundary()) || (e0->halfedge()->twin()->isBoundary())) {
           return VertexIter();
        }
        HalfedgeIter he0 = e0->halfedge();
        HalfedgeIter twin = he0->twin();
        HalfedgeIter bd = he0->next();
        HalfedgeIter dc = bd->next();
        HalfedgeIter ca = twin->next();
        HalfedgeIter ab = ca->next();
        VertexIter a = ab->vertex();
        VertexIter b = bd->vertex();
        VertexIter c = ca->vertex();
        VertexIter d = dc->vertex();
        FaceIter f1 = he0->face();
        FaceIter f2 = twin->face();
        // add 1 vertex, 3 edges, 6 halfedges, 2 faces
        VertexIter m = newVertex();
        m->position = (c->position + b->position)/2;
        EdgeIter ama = newEdge();
        EdgeIter bmb = newEdge();
        EdgeIter dmd = newEdge();
        HalfedgeIter am = newHalfedge();
        HalfedgeIter bm = newHalfedge();
        HalfedgeIter dm = newHalfedge();
        HalfedgeIter ma = newHalfedge();
        HalfedgeIter mb = newHalfedge();
        HalfedgeIter md = newHalfedge();
        FaceIter f3 = newFace();
        FaceIter f4 = newFace();

        m->halfedge() = twin;
        b->halfedge() = bm;
        ama->halfedge() = am;
        bmb->halfedge() = bm;
        dmd->halfedge() = dm;
        f1->halfedge() = he0;
        f2->halfedge() = twin;
        f3->halfedge() = bm;
        f4->halfedge() = mb;
        ab->face() = f3;
        bd->face() = f4;
        
        he0->next() = md;
        twin->vertex() = m;
        am->setNeighbors(twin, ma, a, ama, f2);
        ma->setNeighbors(ab, am, m, ama, f3);
        bm->setNeighbors(ma, mb, b, bmb, f3);
        mb->setNeighbors(bd, bm, m, bmb, f4);
        dm->setNeighbors(mb, md, d, dmd, f4);
        md->setNeighbors(dc, dm, m, dmd, f1);

        ca->next() = am;
        ab->next() = bm;
        bd->next() = dm;
        dc->next() = he0;
        return m;
    }

    void MeshResampler::upsample(HalfedgeMesh& mesh)
    // TODO Part 5.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    {
        // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
        // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
        // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
        // the new subdivided (fine) mesh, which has more elements to traverse.  We will then assign vertex positions in
        // the new mesh based on the values we computed for the original mesh.


        // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
        // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
        // TODO a vertex of the original mesh.


        // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


        // TODO Next, we're going to split every edge in the mesh, in any order.  For future
        // TODO reference, we're also going to store some information about which subdivided
        // TODO edges come from splitting an edge in the original mesh, and which edges are new,
        // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
        // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
        // TODO just split (and the loop will never end!)


        // TODO Now flip any new edge that connects an old and new vertex.


        // TODO Finally, copy the new vertex positions into final Vertex::position.

    }

    // TODO Part 6.
    // TODO There's also some code you'll need to complete in "Shader/frag" file.

}
