/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** @author Jia Pan */

#ifndef FCL_TRAVERSAL_OCTREE_OCTREESOLVER_INL_H
#define FCL_TRAVERSAL_OCTREE_OCTREESOLVER_INL_H

#include "fcl/narrowphase/detail/traversal/octree/octree_solver.h"

#include "fcl/geometry/shape/utility.h"

namespace fcl
{

namespace detail
{

//==============================================================================
template <typename NarrowPhaseSolver>
OcTreeSolver<NarrowPhaseSolver>::OcTreeSolver(
    const NarrowPhaseSolver* solver_)
  : solver(solver_),
    crequest(nullptr),
    drequest(nullptr),
    cresult(nullptr),
    dresult(nullptr)
{
  // Do nothing
}

//==============================================================================
template <typename NarrowPhaseSolver>
void OcTreeSolver<NarrowPhaseSolver>::OcTreeIntersect(
    const OcTree<S>* tree1,
    const OcTree<S>* tree2,
    const Transform3<S>& tf1,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request_,
    CollisionResult<S>& result_) const
{
  crequest = &request_;
  cresult = &result_;

  OcTreeIntersectRecurse(tree1, tree1->getRoot(), tree1->getRootBV(),
                         tree2, tree2->getRoot(), tree2->getRootBV(),
                         tf1, tf2);
}

//==============================================================================
template <typename NarrowPhaseSolver>
void OcTreeSolver<NarrowPhaseSolver>::OcTreeDistance(
    const OcTree<S>* tree1,
    const OcTree<S>* tree2,
    const Transform3<S>& tf1,
    const Transform3<S>& tf2,
    const DistanceRequest<S>& request_,
    DistanceResult<S>& result_) const
{
  drequest = &request_;
  dresult = &result_;

  OcTreeDistanceRecurse(tree1, tree1->getRoot(), tree1->getRootBV(),
                        tree2, tree2->getRoot(), tree2->getRootBV(),
                        tf1, tf2);
}

//==============================================================================
template <typename NarrowPhaseSolver>
template <typename BV>
void OcTreeSolver<NarrowPhaseSolver>::OcTreeMeshIntersect(
    const OcTree<S>* tree1,
    const BVHModel<BV>* tree2,
    const Transform3<S>& tf1,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request_,
    CollisionResult<S>& result_) const
{
  crequest = &request_;
  cresult = &result_;

  OcTreeMeshIntersectRecurse(tree1, tree1->getRoot(), tree1->getRootBV(),
                             tree2, 0,
                             tf1.inverse() * tf2,
                             &tree1->getRegionOfInterest());
}

//==============================================================================
template <typename NarrowPhaseSolver>
template <typename BV>
void OcTreeSolver<NarrowPhaseSolver>::OcTreeMeshDistance(
    const OcTree<S>* tree1,
    const BVHModel<BV>* tree2,
    const Transform3<S>& tf1,
    const Transform3<S>& tf2,
    const DistanceRequest<S>& request_,
    DistanceResult<S>& result_) const
{
  drequest = &request_;
  dresult = &result_;

  OcTreeMeshDistanceRecurse(tree1, tree1->getRoot(), tree1->getRootBV(),
                            tree2, 0,
                            tf1.inverse() * tf2,
                            &tree1->getRegionOfInterest());
}

//==============================================================================
template <typename NarrowPhaseSolver>
template <typename BV>
void OcTreeSolver<NarrowPhaseSolver>::MeshOcTreeIntersect(
    const BVHModel<BV>* tree1,
    const OcTree<S>* tree2,
    const Transform3<S>& tf1,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request_,
    CollisionResult<S>& result_) const

{
  OcTreeMeshIntersect(tree2, tree1, tf2, tf1, request_, result_);
}

//==============================================================================
/// @brief distance between mesh and octree
template <typename NarrowPhaseSolver>
template <typename BV>
void OcTreeSolver<NarrowPhaseSolver>::MeshOcTreeDistance(
    const BVHModel<BV>* tree1,
    const OcTree<S>* tree2,
    const Transform3<S>& tf1,
    const Transform3<S>& tf2,
    const DistanceRequest<S>& request_,
    DistanceResult<S>& result_) const
{
  OcTreeMeshDistance(tree2, tree1, tf2, tf1, request_, result_);
}

//==============================================================================
/// @brief collision between octree and shape
template <typename NarrowPhaseSolver>
template <typename Shape>
void OcTreeSolver<NarrowPhaseSolver>::OcTreeShapeIntersect(
    const OcTree<S>* tree,
    const Shape& s,
    const Transform3<S>& tf1,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request_,
    CollisionResult<S>& result_) const
{
  crequest = &request_;
  cresult = &result_;

  AABB<S> bv2;
  computeBV(s, Transform3<S>::Identity(), bv2);
  OBB<S> obb2;
  convertBV(bv2, tf2, obb2);
  OcTreeShapeIntersectRecurse(tree, tree->getRoot(), tree->getRootBV(),
                              s, obb2,
                              tf1, tf2);

}

//==============================================================================
/// @brief collision between shape and octree
template <typename NarrowPhaseSolver>
template <typename Shape>
void OcTreeSolver<NarrowPhaseSolver>::ShapeOcTreeIntersect(
    const Shape& s,
    const OcTree<S>* tree,
    const Transform3<S>& tf1,
    const Transform3<S>& tf2,
    const CollisionRequest<S>& request_,
    CollisionResult<S>& result_) const
{
  crequest = &request_;
  cresult = &result_;

  AABB<S> bv1;
  computeBV(s, Transform3<S>::Identity(), bv1);
  OBB<S> obb1;
  convertBV(bv1, tf1, obb1);
  OcTreeShapeIntersectRecurse(tree, tree->getRoot(), tree->getRootBV(),
                              s, obb1,
                              tf2, tf1);
}

//==============================================================================
/// @brief distance between octree and shape
template <typename NarrowPhaseSolver>
template <typename Shape>
void OcTreeSolver<NarrowPhaseSolver>::OcTreeShapeDistance(
    const OcTree<S>* tree,
    const Shape& s,
    const Transform3<S>& tf1,
    const Transform3<S>& tf2,
    const DistanceRequest<S>& request_,
    DistanceResult<S>& result_) const
{
  drequest = &request_;
  dresult = &result_;

  AABB<S> aabb2;
  computeBV(s, tf2, aabb2);
  OcTreeShapeDistanceRecurse(tree, tree->getRoot(), tree->getRootBV(),
                             s, aabb2,
                             tf1, tf2);
}

//==============================================================================
/// @brief distance between shape and octree
template <typename NarrowPhaseSolver>
template <typename Shape>
void OcTreeSolver<NarrowPhaseSolver>::ShapeOcTreeDistance(
    const Shape& s,
    const OcTree<S>* tree,
    const Transform3<S>& tf1,
    const Transform3<S>& tf2,
    const DistanceRequest<S>& request_,
    DistanceResult<S>& result_) const
{
  drequest = &request_;
  dresult = &result_;

  AABB<S> aabb1;
  computeBV(s, tf1, aabb1);
  OcTreeShapeDistanceRecurse(tree, tree->getRoot(), tree->getRootBV(),
                             s, aabb1,
                             tf2, tf1);
}

//==============================================================================
template <typename NarrowPhaseSolver>
template <typename Shape>
bool OcTreeSolver<NarrowPhaseSolver>::OcTreeShapeDistanceRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1,
                                const Shape& s, const AABB<S>& aabb2,
                                const Transform3<S>& tf1, const Transform3<S>& tf2) const
{
  if(!tree1->nodeHasChildren(root1))
  {
    if(tree1->isNodeOccupied(root1))
    {
      Box<S> box;
      Transform3<S> box_tf;
      constructBox(bv1, tf1, box, box_tf);

      S dist;
      // NOTE(JS): The closest points are set to zeros in order to suppress the
      // maybe-uninitialized warning. It seems the warnings occur since
      // NarrowPhaseSolver::shapeDistance() conditionally set the closest points.
      // If this wasn't intentional then please remove the initialization of the
      // closest points, and change the function NarrowPhaseSolver::shapeDistance()
      // to always set the closest points.
      Vector3<S> closest_p1 = Vector3<S>::Zero();
      Vector3<S> closest_p2 = Vector3<S>::Zero();
      solver->shapeDistance(box, box_tf, s, tf2, &dist, &closest_p1, &closest_p2);

      dresult->update(dist, tree1, &s, root1 - tree1->getRoot(), DistanceResult<S>::NONE, closest_p1, closest_p2);

      return drequest->isSatisfied(*dresult);
    }
    else
      return false;
  }

  if(!tree1->isNodeOccupied(root1)) return false;

  for(unsigned int i = 0; i < 8; ++i)
  {
    if(tree1->nodeChildExists(root1, i))
    {
      const typename OcTree<S>::OcTreeNode* child = tree1->getNodeChild(root1, i);
      AABB<S> child_bv;
      computeChildBV(bv1, i, child_bv);

      AABB<S> aabb1;
      convertBV(child_bv, tf1, aabb1);
      S d = aabb1.distance(aabb2);
      if(d < dresult->min_distance)
      {
        if(OcTreeShapeDistanceRecurse(tree1, child, child_bv, s, aabb2, tf1, tf2))
          return true;
      }
    }
  }

  return false;
}

//==============================================================================
template <typename NarrowPhaseSolver>
template <typename Shape>
bool OcTreeSolver<NarrowPhaseSolver>::OcTreeShapeIntersectRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1,
                                 const Shape& s, const OBB<S>& obb2,
                                 const Transform3<S>& tf1, const Transform3<S>& tf2) const
{
  if(!root1)
  {
    OBB<S> obb1;
    convertBV(bv1, tf1, obb1);
    if(obb1.overlap(obb2))
    {
      Box<S> box;
      Transform3<S> box_tf;
      constructBox(bv1, tf1, box, box_tf);

      if(solver->shapeIntersect(box, box_tf, s, tf2, nullptr))
      {
        AABB<S> overlap_part;
        AABB<S> aabb1, aabb2;
        computeBV(box, box_tf, aabb1);
        computeBV(s, tf2, aabb2);
        aabb1.overlap(aabb2, overlap_part);
        cresult->addCostSource(CostSource<S>(overlap_part, tree1->getOccupancyThres() * s.cost_density), crequest->num_max_cost_sources);
      }
    }

    return false;
  }
  else if(!tree1->nodeHasChildren(root1))
  {
    if(tree1->isNodeOccupied(root1) && s.isOccupied()) // occupied area
    {
      OBB<S> obb1;
      convertBV(bv1, tf1, obb1);
      if(obb1.overlap(obb2))
      {
        Box<S> box;
        Transform3<S> box_tf;
        constructBox(bv1, tf1, box, box_tf);

        bool is_intersect = false;
        if(!crequest->enable_contact)
        {
          if(solver->shapeIntersect(box, box_tf, s, tf2, nullptr))
          {
            is_intersect = true;
            if(cresult->numContacts() < crequest->num_max_contacts)
              cresult->addContact(Contact<S>(tree1, &s, root1 - tree1->getRoot(), Contact<S>::NONE));
          }
        }
        else
        {
          std::vector<ContactPoint<S>> contacts;
          if(solver->shapeIntersect(box, box_tf, s, tf2, &contacts))
          {
            is_intersect = true;
            if(crequest->num_max_contacts > cresult->numContacts())
            {
              const size_t free_space = crequest->num_max_contacts - cresult->numContacts();
              size_t num_adding_contacts;

              // If the free space is not enough to add all the new contacts, we add contacts in descent order of penetration depth.
              if (free_space < contacts.size())
              {
                std::partial_sort(contacts.begin(), contacts.begin() + free_space, contacts.end(), std::bind(comparePenDepth<S>, std::placeholders::_2, std::placeholders::_1));
                num_adding_contacts = free_space;
              }
              else
              {
                num_adding_contacts = contacts.size();
              }

              for(size_t i = 0; i < num_adding_contacts; ++i)
                cresult->addContact(Contact<S>(tree1, &s, root1 - tree1->getRoot(), Contact<S>::NONE, contacts[i].pos, contacts[i].normal, contacts[i].penetration_depth));
            }
          }
        }

        if(is_intersect && crequest->enable_cost)
        {
          AABB<S> overlap_part;
          AABB<S> aabb1, aabb2;
          computeBV(box, box_tf, aabb1);
          computeBV(s, tf2, aabb2);
          aabb1.overlap(aabb2, overlap_part);
        }

        return crequest->isSatisfied(*cresult);
      }
      else return false;
    }
    else if(!tree1->isNodeFree(root1) && !s.isFree() && crequest->enable_cost) // uncertain area
    {
      OBB<S> obb1;
      convertBV(bv1, tf1, obb1);
      if(obb1.overlap(obb2))
      {
        Box<S> box;
        Transform3<S> box_tf;
        constructBox(bv1, tf1, box, box_tf);

        if(solver->shapeIntersect(box, box_tf, s, tf2, nullptr))
        {
          AABB<S> overlap_part;
          AABB<S> aabb1, aabb2;
          computeBV(box, box_tf, aabb1);
          computeBV(s, tf2, aabb2);
          aabb1.overlap(aabb2, overlap_part);
        }
      }

      return false;
    }
    else // free area
      return false;
  }

  /// stop when 1) bounding boxes of two objects not overlap; OR
  ///           2) at least of one the nodes is free; OR
  ///           2) (two uncertain nodes or one node occupied and one node uncertain) AND cost not required
  if(tree1->isNodeFree(root1) || s.isFree()) return false;
  else if((tree1->isNodeUncertain(root1) || s.isUncertain()) && !crequest->enable_cost) return false;
  else
  {
    OBB<S> obb1;
    convertBV(bv1, tf1, obb1);
    if(!obb1.overlap(obb2)) return false;
  }

  for(unsigned int i = 0; i < 8; ++i)
  {
    if(tree1->nodeChildExists(root1, i))
    {
      const typename OcTree<S>::OcTreeNode* child = tree1->getNodeChild(root1, i);
      AABB<S> child_bv;
      computeChildBV(bv1, i, child_bv);

      if(OcTreeShapeIntersectRecurse(tree1, child, child_bv, s, obb2, tf1, tf2))
        return true;
    }
    else if(!s.isFree() && crequest->enable_cost)
    {
      AABB<S> child_bv;
      computeChildBV(bv1, i, child_bv);

      if(OcTreeShapeIntersectRecurse(tree1, nullptr, child_bv, s, obb2, tf1, tf2))
        return true;
    }
  }

  return false;
}

// Convenience function to find the distance from a cube (represented in aabb1) in I configuration
// to some other bounding volume
template <typename S, typename BV2>
inline S distanceOctomapRSS(const AABB<S>& aabb1, const Vector3<S>& bv1_center,
                            const BV2& bv2, const Transform3<S>& tf2, const Vector3<S>& bv2_center)
{
  static Matrix3<S> axis_yzx = (Matrix3<S>() << 0, 0, 1,
                                                1, 0, 0,
                                                0, 1, 0).finished();
  static Matrix3<S> axis_zxy = (Matrix3<S>() << 0, 1, 0,
                                                0, 0, 1,
                                                1, 0, 0).finished();
  static Matrix3<S> axis_xyz = (Matrix3<S>() << 1, 0, 0,
                                                0, 1, 0,
                                                0, 0, 1).finished();
  // Leverage the fact that octomap cells are cubes and
  // orient the flat face of the RSS at the current other BV
  RSS<S> rss;
  Vector3<S> dir(bv1_center-bv2_center);
  dir[0] = std::abs(dir[0]);
  dir[1] = std::abs(dir[1]);
  dir[2] = std::abs(dir[2]);
  const S x = aabb1.width();
  rss.r = x / 2;
  rss.l[0] = x;
  rss.l[1] = x;
  const bool x_greater_than_y = (dir[0] > dir[1]);
  const bool y_greater_than_z = (dir[1] > dir[2]);
  if (x_greater_than_y && y_greater_than_z)
  {
    rss.axis = axis_yzx;
  }
  else if (!(x_greater_than_y) && (y_greater_than_z))
  {
    rss.axis = axis_zxy;
  }
  else
  {
    rss.axis = axis_xyz;
  }
  rss.setToFromCenter(bv1_center);
  return distanceBV(rss, bv2, tf2);
}

// return -1 if bv1 out of roi, 1 if in, and 0 if on.
template <typename S>
static inline int checkROI(const AABB<S>& bv1, const std::vector<Halfspace<S>>* roi)
{
  Vector3<S> bv1_center = bv1.center();
  Vector3<S> bv1_diag = bv1.max_ - bv1.min_;
  bool all_in = true;

  for (unsigned int i=0; i<roi->size(); ++i)
  {
    const Halfspace<S>& region((*roi)[i]);
    Vector3<S> normal = region.n;
    Vector3<S> n_dot_d(normal[0] * bv1_diag[0], normal[1] * bv1_diag[1], normal[2] * bv1_diag[2]);
    Vector3<S> n_dot_d_abs = n_dot_d.cwiseAbs();
    S bv1_extent = 0.5 * (n_dot_d_abs[0] + n_dot_d_abs[1] + n_dot_d_abs[2]);
    S center_dist = region.signedDistance(bv1_center);
    // If the distance from the center of the AABB in bv1 to the halfspace is
    // bigger than the maximum extent of the AABB, the AABB is outside the
    // boundary of the halfspace. If the negative of the center distance is
    // bigger than the extent, the whole AABB is inside the halfspace.
    // Otherwise, its on the boundary.
    bool out = (center_dist > bv1_extent);
    bool in = (-center_dist > bv1_extent);
    if (out)
    {
      return -1;
    }
    all_in = (all_in && in);
  }
  if (all_in)
  {
    return 1;
  }
  return 0;
}

//==============================================================================
template <typename NarrowPhaseSolver>
template <typename BV>
bool OcTreeSolver<NarrowPhaseSolver>::OcTreeMeshDistanceRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1,
                               const BVHModel<BV>* tree2, int root2,
                               const Transform3<S>& tf2,
                               const std::vector<Halfspace<S>>* roi) const
{
  // First check region of interest.
  if (roi)
  {
    int rv = checkROI<S>(bv1, roi);
    if (rv == -1)
    {
      // this octomap region is entirely out of the region of interest
      return false;
    }
    if (rv == 1)
    {
      // this octomap region is entirely inside the region of interest.
      // There is no need to check any sub-region. Null out the roi for
      // subsequent recursive calls.
      roi = nullptr;
    }
  }

  if(!tree1->nodeHasChildren(root1) && tree2->getBV(root2).isLeaf())
  {
    if(tree1->isNodeOccupied(root1))
    {
      Box<S> box;
      Transform3<S> box_tf;
      constructBox(bv1, Transform3<S>::Identity(), box, box_tf);

      int primitive_id = tree2->getBV(root2).primitiveId();
      const Triangle& tri_id = tree2->tri_indices[primitive_id];
      const Vector3<S>& p1 = tree2->vertices[tri_id[0]];
      const Vector3<S>& p2 = tree2->vertices[tri_id[1]];
      const Vector3<S>& p3 = tree2->vertices[tri_id[2]];

      S dist;
      Vector3<S> closest_p1, closest_p2;
      solver->shapeTriangleDistance(box, box_tf, p1, p2, p3, tf2, &dist, &closest_p1, &closest_p2);
      dresult->primative_distance_calculations++;
      if (dist < dresult->min_distance)
      {
        // only allocate dynamic memory in the case where a new min was found
        std::shared_ptr<Box<S>> box_ptr(new Box<S>(box));
        std::shared_ptr<TriangleP<S>> triangle(new TriangleP<S>(p1, p2, p3));
        dresult->update(dist, tree1, tree2, root1 - tree1->getRoot(), primitive_id,
                        closest_p1, closest_p2, box_ptr, box_tf, triangle, tf2);
      }

      return drequest->isSatisfied(*dresult);
    }
    else
      return false;
  }

  if(!tree1->isNodeOccupied(root1)) return false;

  if(tree1->nodeHasChildren(root1))
  {
    unsigned int nchildren = 0;
    const typename OcTree<S>::OcTreeNode* children[8];
    AABB<S> child_bvs[8];
    S distances[8];
    S min_distance = std::numeric_limits<S>::max();
    S next_min;
    const BV& bv2 = tree2->getBV(root2).bv;
    const Vector3<S> bv2_center(tf2 * bv2.center());
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree1->nodeChildExists(root1, i))
      {
        const typename OcTree<S>::OcTreeNode* child = tree1->getNodeChild(root1, i);
        if(tree1->isNodeOccupied(child))
        {
          children[nchildren] = child;
          computeChildBV(bv1, i, child_bvs[nchildren]);
          distances[nchildren] = distanceOctomapRSS(child_bvs[nchildren], child_bvs[nchildren].center(),
                                                    bv2, tf2, bv2_center);
          dresult->bv_distance_calculations++;
          if (distances[nchildren] < min_distance)
          {
            min_distance = distances[nchildren];
          }
          nchildren++;
        }
      }
    }
    // Visit the octree from closest to furthest and quit early when we have
    // crossed the result min distance
    while(min_distance < dresult->min_distance)
    {
      next_min = std::numeric_limits<S>::max();
      for(unsigned int i = 0; i < nchildren; ++i)
      {
        if(distances[i] == min_distance)
        {
          if(distances[i] < dresult->min_distance)
          {
            // Possible a better result is below, descend
            if(OcTreeMeshDistanceRecurse(tree1, children[i], child_bvs[i], tree2, root2, tf2, roi))
              return true;
          }
          else
          {
            break;
          }
        }
        else if(distances[i] > min_distance)
        {
          if(distances[i] < next_min)
          {
            next_min = distances[i];
          }
        }
        else
        {
          // an already visited spot on a previous iteration
        }
      }
      min_distance = next_min;
    }
  }
  else
  {
    const Vector3<S> bv1_center(bv1.center());
    int children[2] = {
      tree2->getBV(root2).leftChild(),
      tree2->getBV(root2).rightChild()};
    const BV* bv2[2] = {
      &tree2->getBV(children[0]).bv,
      &tree2->getBV(children[1]).bv};
    S d[2] = {
      distanceOctomapRSS(bv1, bv1_center, *bv2[0], tf2, tf2 * bv2[0]->center()),
      distanceOctomapRSS(bv1, bv1_center, *bv2[1], tf2, tf2 * bv2[1]->center())};
    dresult->bv_distance_calculations+=2;
    // Go left first if it is closer, otherwise go right first
    if (d[0] < d[1])
    {
      for (int i=0; i<2; ++i)
      {
        if(d[i] < dresult->min_distance)
        {
          // Because we descend the octree first, there is no need to check the
          // ROI when descending the mesh.
          if(OcTreeMeshDistanceRecurse(tree1, root1, bv1, tree2, children[i], tf2, nullptr))
            return true;
        }
      }
    }
    else
    {
      for (int i=1; i>-1; --i)
      {
        if(d[i] < dresult->min_distance)
        {
          // Because we descend the octree first, there is no need to check the
          // ROI when descending the mesh.
          if(OcTreeMeshDistanceRecurse(tree1, root1, bv1, tree2, children[i], tf2, nullptr))
            return true;
        }
      }
    }
  }

  return false;
}

// Convenience function to set an OBB from an AABB, given I configuration
template <typename S>
static inline void setOBBfromIdentityAABB(const AABB<S>& aabb, OBB<S>& obb)
{
  obb.To.noalias() = aabb.center();
  obb.extent.noalias() = (aabb.max_ - aabb.min_) * 0.5;
  obb.axis.noalias() = Matrix3<S>::Identity();
}

//==============================================================================
template <typename NarrowPhaseSolver>
template <typename BV>
bool OcTreeSolver<NarrowPhaseSolver>::OcTreeMeshIntersectRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1,
                                const BVHModel<BV>* tree2, int root2,
                                const Transform3<S>& tf2,
                                const std::vector<Halfspace<S>>* roi) const
{
  // First check region of interest.
  if (roi)
  {
    int rv = checkROI<S>(bv1, roi);
    if (rv == -1)
    {
      // this octomap region is entirely out of the region of interest
      return false;
    }
    if (rv == 1)
    {
      // this octomap region is entirely inside the region of interest.
      // There is no need to check any sub-region. Null out the roi for
      // subsequent recursive calls.
      roi = nullptr;
    }
  }

  if(!root1)
  {
    if(tree2->getBV(root2).isLeaf())
    {
      OBB<S> obb1, obb2;
      setOBBfromIdentityAABB<S>(bv1, obb1);
      convertBV(tree2->getBV(root2).bv, tf2, obb2);
      if(obb1.overlap(obb2))
      {
        Box<S> box;
        Transform3<S> box_tf;
        constructBox(bv1, Transform3<S>::Identity(), box, box_tf);

        int primitive_id = tree2->getBV(root2).primitiveId();
        const Triangle& tri_id = tree2->tri_indices[primitive_id];
        const Vector3<S>& p1 = tree2->vertices[tri_id[0]];
        const Vector3<S>& p2 = tree2->vertices[tri_id[1]];
        const Vector3<S>& p3 = tree2->vertices[tri_id[2]];

        if(solver->shapeTriangleIntersect(box, box_tf, p1, p2, p3, tf2, nullptr, nullptr, nullptr))
        {
          AABB<S> overlap_part;
          AABB<S> aabb1;
          computeBV(box, box_tf, aabb1);
          AABB<S> aabb2(tf2 * p1, tf2 * p2, tf2 * p3);
          aabb1.overlap(aabb2, overlap_part);
          cresult->addCostSource(CostSource<S>(overlap_part, tree1->getOccupancyThres() * tree2->cost_density), crequest->num_max_cost_sources);
        }
      }

      return false;
    }
    else
    {
      if(OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2, tree2->getBV(root2).leftChild(), tf2, roi))
        return true;

      if(OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2, tree2->getBV(root2).rightChild(), tf2, roi))
        return true;

      return false;
    }
  }
  else if(!tree1->nodeHasChildren(root1) && tree2->getBV(root2).isLeaf())
  {
    if(tree1->isNodeOccupied(root1) && tree2->isOccupied())
    {
      OBB<S> obb1, obb2;
      setOBBfromIdentityAABB<S>(bv1, obb1);
      convertBV(tree2->getBV(root2).bv, tf2, obb2);
      if(obb1.overlap(obb2))
      {
        Box<S> box;
        Transform3<S> box_tf;
        constructBox(bv1, Transform3<S>::Identity(), box, box_tf);

        int primitive_id = tree2->getBV(root2).primitiveId();
        const Triangle& tri_id = tree2->tri_indices[primitive_id];
        const Vector3<S>& p1 = tree2->vertices[tri_id[0]];
        const Vector3<S>& p2 = tree2->vertices[tri_id[1]];
        const Vector3<S>& p3 = tree2->vertices[tri_id[2]];

        bool is_intersect = false;
        if(!crequest->enable_contact)
        {
          if(solver->shapeTriangleIntersect(box, box_tf, p1, p2, p3, tf2, nullptr, nullptr, nullptr))
          {
            is_intersect = true;
            if(cresult->numContacts() < crequest->num_max_contacts)
              cresult->addContact(Contact<S>(tree1, tree2, root1 - tree1->getRoot(), primitive_id));
          }
        }
        else
        {
          Vector3<S> contact;
          S depth;
          Vector3<S> normal;

          if(solver->shapeTriangleIntersect(box, box_tf, p1, p2, p3, tf2, &contact, &depth, &normal))
          {
            is_intersect = true;
            if(cresult->numContacts() < crequest->num_max_contacts)
              cresult->addContact(Contact<S>(tree1, tree2, root1 - tree1->getRoot(), primitive_id, contact, normal, depth));
          }
        }

        if(is_intersect && crequest->enable_cost)
        {
          AABB<S> overlap_part;
          AABB<S> aabb1;
          computeBV(box, box_tf, aabb1);
          AABB<S> aabb2(tf2 * p1, tf2 * p2, tf2 * p3);
          aabb1.overlap(aabb2, overlap_part);
    cresult->addCostSource(CostSource<S>(overlap_part, root1->getOccupancy() * tree2->cost_density), crequest->num_max_cost_sources);
        }

        return crequest->isSatisfied(*cresult);
      }
      else
        return false;
    }
    else if(!tree1->isNodeFree(root1) && !tree2->isFree() && crequest->enable_cost) // uncertain area
    {
      OBB<S> obb1, obb2;
      setOBBfromIdentityAABB<S>(bv1, obb1);
      convertBV(tree2->getBV(root2).bv, tf2, obb2);
      if(obb1.overlap(obb2))
      {
        Box<S> box;
        Transform3<S> box_tf;
        constructBox(bv1, Transform3<S>::Identity(), box, box_tf);

        int primitive_id = tree2->getBV(root2).primitiveId();
        const Triangle& tri_id = tree2->tri_indices[primitive_id];
        const Vector3<S>& p1 = tree2->vertices[tri_id[0]];
        const Vector3<S>& p2 = tree2->vertices[tri_id[1]];
        const Vector3<S>& p3 = tree2->vertices[tri_id[2]];

        if(solver->shapeTriangleIntersect(box, box_tf, p1, p2, p3, tf2, nullptr, nullptr, nullptr))
        {
          AABB<S> overlap_part;
          AABB<S> aabb1;
          computeBV(box, box_tf, aabb1);
          AABB<S> aabb2(tf2 * p1, tf2 * p2, tf2 * p3);
          aabb1.overlap(aabb2, overlap_part);
    cresult->addCostSource(CostSource<S>(overlap_part, root1->getOccupancy() * tree2->cost_density), crequest->num_max_cost_sources);
        }
      }

      return false;
    }
    else // free area
      return false;
  }

  /// stop when 1) bounding boxes of two objects not overlap; OR
  ///           2) at least one of the nodes is free; OR
  ///           2) (two uncertain nodes OR one node occupied and one node uncertain) AND cost not required
  if(tree1->isNodeFree(root1) || tree2->isFree()) return false;
  else if((tree1->isNodeUncertain(root1) || tree2->isUncertain()) && !crequest->enable_cost) return false;
  else
  {
    OBB<S> obb1, obb2;
    setOBBfromIdentityAABB<S>(bv1, obb1);
    convertBV(tree2->getBV(root2).bv, tf2, obb2);
    if(!obb1.overlap(obb2)) return false;
  }

  if(tree1->nodeHasChildren(root1))
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree1->nodeChildExists(root1, i))
      {
        const typename OcTree<S>::OcTreeNode* child = tree1->getNodeChild(root1, i);
        if (!tree1->isNodeFree(child))
        {
          AABB<S> child_bv;
          computeChildBV(bv1, i, child_bv);

          if(OcTreeMeshIntersectRecurse(tree1, child, child_bv, tree2, root2, tf2, roi))
            return true;
        }
      }
else if(!tree2->isFree() && crequest->enable_cost)
      {
        AABB<S> child_bv;
        computeChildBV(bv1, i, child_bv);

        if(OcTreeMeshIntersectRecurse(tree1, nullptr, child_bv, tree2, root2, tf2, roi))
          return true;
      }
    }
  }
  else
  {
    if(OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2, tree2->getBV(root2).leftChild(), tf2, roi))
      return true;

    if(OcTreeMeshIntersectRecurse(tree1, root1, bv1, tree2, tree2->getBV(root2).rightChild(), tf2, roi))
      return true;

  }

  return false;
}

//==============================================================================
template <typename NarrowPhaseSolver>
bool OcTreeSolver<NarrowPhaseSolver>::OcTreeDistanceRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1, const OcTree<S>* tree2, const typename OcTree<S>::OcTreeNode* root2, const AABB<S>& bv2, const Transform3<S>& tf1, const Transform3<S>& tf2) const
{
  if(!tree1->nodeHasChildren(root1) && !tree2->nodeHasChildren(root2))
  {
    if(tree1->isNodeOccupied(root1) && tree2->isNodeOccupied(root2))
    {
      Box<S> box1, box2;
      Transform3<S> box1_tf, box2_tf;
      constructBox(bv1, tf1, box1, box1_tf);
      constructBox(bv2, tf2, box2, box2_tf);

      S dist;
      // NOTE(JS): The closest points are set to zeros in order to suppress the
      // maybe-uninitialized warning. It seems the warnings occur since
      // NarrowPhaseSolver::shapeDistance() conditionally set the closest points.
      // If this wasn't intentional then please remove the initialization of the
      // closest points, and change the function NarrowPhaseSolver::shapeDistance()
      // to always set the closest points.
      Vector3<S> closest_p1 = Vector3<S>::Zero();
      Vector3<S> closest_p2 = Vector3<S>::Zero();
      solver->shapeDistance(box1, box1_tf, box2, box2_tf, &dist, &closest_p1, &closest_p2);

      dresult->update(dist, tree1, tree2, root1 - tree1->getRoot(), root2 - tree2->getRoot(), closest_p1, closest_p2);

      return drequest->isSatisfied(*dresult);
    }
    else
      return false;
  }

  if(!tree1->isNodeOccupied(root1) || !tree2->isNodeOccupied(root2)) return false;

  if(!tree2->nodeHasChildren(root2) || (tree1->nodeHasChildren(root1) && (bv1.size() > bv2.size())))
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree1->nodeChildExists(root1, i))
      {
        const typename OcTree<S>::OcTreeNode* child = tree1->getNodeChild(root1, i);
        AABB<S> child_bv;
        computeChildBV(bv1, i, child_bv);

        S d;
        AABB<S> aabb1, aabb2;
        convertBV(bv1, tf1, aabb1);
        convertBV(bv2, tf2, aabb2);
        d = aabb1.distance(aabb2);

        if(d < dresult->min_distance)
        {

          if(OcTreeDistanceRecurse(tree1, child, child_bv, tree2, root2, bv2, tf1, tf2))
            return true;
        }
      }
    }
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree2->nodeChildExists(root2, i))
      {
        const typename OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB<S> child_bv;
        computeChildBV(bv2, i, child_bv);

        S d;
        AABB<S> aabb1, aabb2;
        convertBV(bv1, tf1, aabb1);
        convertBV(bv2, tf2, aabb2);
        d = aabb1.distance(aabb2);

        if(d < dresult->min_distance)
        {
          if(OcTreeDistanceRecurse(tree1, root1, bv1, tree2, child, child_bv, tf1, tf2))
            return true;
        }
      }
    }
  }

  return false;
}

//==============================================================================
template <typename NarrowPhaseSolver>
bool OcTreeSolver<NarrowPhaseSolver>::OcTreeIntersectRecurse(const OcTree<S>* tree1, const typename OcTree<S>::OcTreeNode* root1, const AABB<S>& bv1, const OcTree<S>* tree2, const typename OcTree<S>::OcTreeNode* root2, const AABB<S>& bv2, const Transform3<S>& tf1, const Transform3<S>& tf2) const
{
  if(!root1 && !root2)
  {
    OBB<S> obb1, obb2;
    convertBV(bv1, tf1, obb1);
    convertBV(bv2, tf2, obb2);

    if(obb1.overlap(obb2))
    {
      Box<S> box1, box2;
      Transform3<S> box1_tf, box2_tf;
      constructBox(bv1, tf1, box1, box1_tf);
      constructBox(bv2, tf2, box2, box2_tf);

      AABB<S> overlap_part;
      AABB<S> aabb1, aabb2;
      computeBV(box1, box1_tf, aabb1);
      computeBV(box2, box2_tf, aabb2);
      aabb1.overlap(aabb2, overlap_part);
      cresult->addCostSource(CostSource<S>(overlap_part, tree1->getOccupancyThres() * tree2->getOccupancyThres()), crequest->num_max_cost_sources);
    }

    return false;
  }
  else if(!root1 && root2)
  {
    if(tree2->nodeHasChildren(root2))
    {
      for(unsigned int i = 0; i < 8; ++i)
      {
        if(tree2->nodeChildExists(root2, i))
        {
          const typename OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
          AABB<S> child_bv;
          computeChildBV(bv2, i, child_bv);
          if(OcTreeIntersectRecurse(tree1, nullptr, bv1, tree2, child, child_bv, tf1, tf2))
            return true;
        }
        else
        {
          AABB<S> child_bv;
          computeChildBV(bv2, i, child_bv);
          if(OcTreeIntersectRecurse(tree1, nullptr, bv1, tree2, nullptr, child_bv, tf1, tf2))
            return true;
        }
      }
    }
    else
    {
      if(OcTreeIntersectRecurse(tree1, nullptr, bv1, tree2, nullptr, bv2, tf1, tf2))
        return true;
    }

    return false;
  }
  else if(root1 && !root2)
  {
    if(tree1->nodeHasChildren(root1))
    {
      for(unsigned int i = 0; i < 8; ++i)
      {
        if(tree1->nodeChildExists(root1, i))
        {
          const typename OcTree<S>::OcTreeNode* child = tree1->getNodeChild(root1, i);
          AABB<S> child_bv;
          computeChildBV(bv1, i,  child_bv);
          if(OcTreeIntersectRecurse(tree1, child, child_bv, tree2, nullptr, bv2, tf1, tf2))
            return true;
        }
        else
        {
          AABB<S> child_bv;
          computeChildBV(bv1, i, child_bv);
          if(OcTreeIntersectRecurse(tree1, nullptr, child_bv, tree2, nullptr, bv2, tf1, tf2))
            return true;
        }
      }
    }
    else
    {
      if(OcTreeIntersectRecurse(tree1, nullptr, bv1, tree2, nullptr, bv2, tf1, tf2))
        return true;
    }

    return false;
  }
  else if(!tree1->nodeHasChildren(root1) && !tree2->nodeHasChildren(root2))
  {
    if(tree1->isNodeOccupied(root1) && tree2->isNodeOccupied(root2)) // occupied area
    {
      bool is_intersect = false;
      if(!crequest->enable_contact)
      {
        OBB<S> obb1, obb2;
        convertBV(bv1, tf1, obb1);
        convertBV(bv2, tf2, obb2);

        if(obb1.overlap(obb2))
        {
          is_intersect = true;
          if(cresult->numContacts() < crequest->num_max_contacts)
            cresult->addContact(Contact<S>(tree1, tree2, root1 - tree1->getRoot(), root2 - tree2->getRoot()));
        }
      }
      else
      {
        Box<S> box1, box2;
        Transform3<S> box1_tf, box2_tf;
        constructBox(bv1, tf1, box1, box1_tf);
        constructBox(bv2, tf2, box2, box2_tf);

        std::vector<ContactPoint<S>> contacts;
        if(solver->shapeIntersect(box1, box1_tf, box2, box2_tf, &contacts))
        {
          is_intersect = true;
          if(crequest->num_max_contacts > cresult->numContacts())
          {
            const size_t free_space = crequest->num_max_contacts - cresult->numContacts();
            size_t num_adding_contacts;

            // If the free space is not enough to add all the new contacts, we add contacts in descent order of penetration depth.
            if (free_space < contacts.size())
            {
              std::partial_sort(contacts.begin(), contacts.begin() + free_space, contacts.end(), std::bind(comparePenDepth<S>, std::placeholders::_2, std::placeholders::_1));
              num_adding_contacts = free_space;
            }
            else
            {
              num_adding_contacts = contacts.size();
            }

            for(size_t i = 0; i < num_adding_contacts; ++i)
              cresult->addContact(Contact<S>(tree1, tree2, root1 - tree1->getRoot(), root2 - tree2->getRoot(), contacts[i].pos, contacts[i].normal, contacts[i].penetration_depth));
          }
        }
      }

      if(is_intersect && crequest->enable_cost)
      {
        Box<S> box1, box2;
        Transform3<S> box1_tf, box2_tf;
        constructBox(bv1, tf1, box1, box1_tf);
        constructBox(bv2, tf2, box2, box2_tf);

        AABB<S> overlap_part;
        AABB<S> aabb1, aabb2;
        computeBV(box1, box1_tf, aabb1);
        computeBV(box2, box2_tf, aabb2);
        aabb1.overlap(aabb2, overlap_part);
        cresult->addCostSource(CostSource<S>(overlap_part, root1->getOccupancy() * root2->getOccupancy()), crequest->num_max_cost_sources);
      }

      return crequest->isSatisfied(*cresult);
    }
    else if(!tree1->isNodeFree(root1) && !tree2->isNodeFree(root2) && crequest->enable_cost) // uncertain area (here means both are uncertain or one uncertain and one occupied)
    {
      OBB<S> obb1, obb2;
      convertBV(bv1, tf1, obb1);
      convertBV(bv2, tf2, obb2);

      if(obb1.overlap(obb2))
      {
        Box<S> box1, box2;
        Transform3<S> box1_tf, box2_tf;
        constructBox(bv1, tf1, box1, box1_tf);
        constructBox(bv2, tf2, box2, box2_tf);

        AABB<S> overlap_part;
        AABB<S> aabb1, aabb2;
        computeBV(box1, box1_tf, aabb1);
        computeBV(box2, box2_tf, aabb2);
        aabb1.overlap(aabb2, overlap_part);
        cresult->addCostSource(CostSource<S>(overlap_part, root1->getOccupancy() * root2->getOccupancy()), crequest->num_max_cost_sources);
      }

      return false;
    }
    else // free area (at least one node is free)
      return false;
  }

  /// stop when 1) bounding boxes of two objects not overlap; OR
  ///           2) at least one of the nodes is free; OR
  ///           2) (two uncertain nodes OR one node occupied and one node uncertain) AND cost not required
  if(tree1->isNodeFree(root1) || tree2->isNodeFree(root2)) return false;
  else if((tree1->isNodeUncertain(root1) || tree2->isNodeUncertain(root2)) && !crequest->enable_cost) return false;
  else
  {
    OBB<S> obb1, obb2;
    convertBV(bv1, tf1, obb1);
    convertBV(bv2, tf2, obb2);
    if(!obb1.overlap(obb2)) return false;
  }

  if(!tree2->nodeHasChildren(root2) || (tree1->nodeHasChildren(root1) && (bv1.size() > bv2.size())))
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree1->nodeChildExists(root1, i))
      {
        const typename OcTree<S>::OcTreeNode* child = tree1->getNodeChild(root1, i);
        AABB<S> child_bv;
        computeChildBV(bv1, i, child_bv);

        if(OcTreeIntersectRecurse(tree1, child, child_bv,
                                  tree2, root2, bv2,
                                  tf1, tf2))
          return true;
      }
      else if(!tree2->isNodeFree(root2) && crequest->enable_cost)
      {
        AABB<S> child_bv;
        computeChildBV(bv1, i, child_bv);

        if(OcTreeIntersectRecurse(tree1, nullptr, child_bv,
                                  tree2, root2, bv2,
                                  tf1, tf2))
          return true;
      }
    }
  }
  else
  {
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree2->nodeChildExists(root2, i))
      {
        const typename OcTree<S>::OcTreeNode* child = tree2->getNodeChild(root2, i);
        AABB<S> child_bv;
        computeChildBV(bv2, i, child_bv);

        if(OcTreeIntersectRecurse(tree1, root1, bv1,
                                  tree2, child, child_bv,
                                  tf1, tf2))
          return true;
      }
      else if(!tree1->isNodeFree(root1) && crequest->enable_cost)
      {
        AABB<S> child_bv;
        computeChildBV(bv2, i, child_bv);

        if(OcTreeIntersectRecurse(tree1, root1, bv1,
                                  tree2, nullptr, child_bv,
                                  tf1, tf2))
          return true;
      }
    }
  }

  return false;
}

} // namespace detail
} // namespace fcl

#endif
