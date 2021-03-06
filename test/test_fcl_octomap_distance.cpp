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

#include <gtest/gtest.h>

#include "fcl/config.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_spatialhash.h"
#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/broadphase/broadphase_SSaP.h"
#include "fcl/broadphase/broadphase_interval_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"

#include "test_fcl_utility.h"

#include "fcl_resources/config.h"

#define NDEBUG 1

using namespace fcl;

/// @brief Octomap distance with an environment with 3 * env_size objects
template <typename S>
void octomap_distance_test(S env_scale, std::size_t env_size, bool use_mesh, bool use_mesh_octomap, double resolution = 0.1);

template<typename BV>
void octomap_distance_test_BVH(std::size_t n, double resolution = 0.1, bool negative_x_roi = false, bool non_negative_x_roi = false);

template <typename S>
void test_octomap_distance()
{
#ifdef NDEBUG
  octomap_distance_test<S>(200, 100, false, false);
  octomap_distance_test<S>(200, 1000, false, false);
#else
  octomap_distance_test<S>(200, 2, false, false, 1.0);
  octomap_distance_test<S>(200, 10, false, false, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_distance)
{
//  test_octomap_distance<float>();
  test_octomap_distance<double>();
}

template <typename S>
void test_octomap_distance_mesh()
{
#ifdef NDEBUG
  octomap_distance_test<S>(200, 100, true, true);
  octomap_distance_test<S>(200, 1000, true, true);
#else
  octomap_distance_test<S>(200, 2, true, true, 1.0);
  octomap_distance_test<S>(200, 5, true, true, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_distance_mesh)
{
//  test_octomap_distance_mesh<float>();
  test_octomap_distance_mesh<double>();
}

template <typename S>
void test_octomap_distance_mesh_octomap_box()
{
#ifdef NDEBUG
  octomap_distance_test<S>(200, 100, true, false);
  octomap_distance_test<S>(200, 1000, true, false);
#else
  octomap_distance_test<S>(200, 2, true, false, 1.0);
  octomap_distance_test<S>(200, 5, true, false, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_distance_mesh_octomap_box)
{
//  test_octomap_distance_mesh_octomap_box<float>();
  test_octomap_distance_mesh_octomap_box<double>();
}

template <typename S>
void test_octomap_bvh_rss_d_distance_rss()
{
#ifdef NDEBUG
  octomap_distance_test_BVH<RSS<S>>(15, 0.1, false, false);
  octomap_distance_test_BVH<RSS<S>>(15, 0.1, true, false);
  octomap_distance_test_BVH<RSS<S>>(15, 0.1, false, true);
#else
  octomap_distance_test_BVH<RSS<S>>(15, 1.0);
  octomap_distance_test_BVH<RSS<S>>(15, 1.0, true, false);
  octomap_distance_test_BVH<RSS<S>>(15, 1.0, false, true);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_bvh_rss_d_distance_rss)
{
//  test_octomap_bvh_rss_d_distance_rss<float>();
  test_octomap_bvh_rss_d_distance_rss<double>();
}

template <typename S>
void test_octomap_bvh_obb_d_distance_obb()
{
#ifdef NDEBUG
  octomap_distance_test_BVH<OBBRSS<S>>(15);
#else
  octomap_distance_test_BVH<OBBRSS<S>>(15, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_bvh_obb_d_distance_obb)
{
//  test_octomap_bvh_obb_d_distance_obb<float>();
  test_octomap_bvh_obb_d_distance_obb<double>();
}

template <typename S>
void test_octomap_bvh_kios_d_distance_kios()
{
#ifdef NDEBUG
  octomap_distance_test_BVH<kIOS<S>>(15);
#else
  octomap_distance_test_BVH<kIOS<S>>(15, 1.0);
#endif
}

GTEST_TEST(FCL_OCTOMAP, test_octomap_bvh_kios_d_distance_kios)
{
//  test_octomap_bvh_kios_d_distance_kios<float>();
  test_octomap_bvh_kios_d_distance_kios<double>();
}

template<typename BV>
void octomap_distance_test_BVH(std::size_t n, double resolution, bool negative_x_roi, bool non_negative_x_roi)
{
  using S = typename BV::S;

  std::vector<Vector3<S>> p1;
  std::vector<Triangle> t1;

  test::loadOBJFile(TEST_RESOURCES_DIR"/env.obj", p1, t1);

  BVHModel<BV>* m1 = new BVHModel<BV>();
  std::shared_ptr<CollisionGeometry<S>> m1_ptr(m1);

  m1->beginModel();
  m1->addSubModel(p1, t1);
  m1->endModel();

  OcTree<S>* tree = new OcTree<S>(std::shared_ptr<octomap::OcTree>(test::generateOcTree(resolution)));
  OcTree<S>* boxes_tree = new OcTree<S>(std::shared_ptr<octomap::OcTree>(test::generateOcTree(resolution, negative_x_roi, non_negative_x_roi)));
  if (negative_x_roi)
  {
    Vector3<S> normal(1.0, 0.0, 0.0);
    Halfspace<S> negative_x(normal, -resolution/2.0);
    tree->addToRegionOfInterest(negative_x);
  }
  if (non_negative_x_roi)
  {
    Vector3<S> normal(-1.0, 0.0, 0.0);
    Halfspace<S> non_negative_x(normal, -resolution/2.0);
    tree->addToRegionOfInterest(non_negative_x);
  }

  std::shared_ptr<CollisionGeometry<S>> tree_ptr(tree);

  aligned_vector<Transform3<S>> transforms;
  S extents[] = {-10, -10, -10, 10, 10, 10};

  test::generateRandomTransforms(extents, transforms, n);
  if (n > 1)
  {
    // Be sure to test identity
    transforms[n - 1] = Transform3<S>::Identity();
    transforms[n / 2 - 1] = Transform3<S>::Identity();
    transforms[n / 2] = Transform3<S>::Identity();
  }

  for(std::size_t i = 0; i < n; ++i)
  {
    Transform3<S> tf1(transforms[i]);
    Transform3<S> tf2(transforms[n-1-i]);

    CollisionObject<S> obj1(m1_ptr, tf1);
    CollisionObject<S> obj2(tree_ptr, tf2);
    test::DistanceData<S> cdata;
    test::DistanceData<S> cdata1b;
    S dist1 = std::numeric_limits<S>::max();
    S dist1b = std::numeric_limits<S>::max();
    test::defaultDistanceFunction(&obj1, &obj2, &cdata, dist1);
    test::defaultDistanceFunction(&obj2, &obj1, &cdata1b, dist1b);
    EXPECT_NEAR(dist1, dist1b, 1e-6);


    std::vector<CollisionObject<S>*> boxes;
    test::generateBoxesFromOctomap(boxes, *boxes_tree);
    for(std::size_t j = 0; j < boxes.size(); ++j)
      boxes[j]->setTransform(tf2 * boxes[j]->getTransform());

    DynamicAABBTreeCollisionManager<S>* manager = new DynamicAABBTreeCollisionManager<S>();
    manager->registerObjects(boxes);
    manager->setup();

    test::DistanceData<S> cdata2;
    manager->distance(&obj1, &cdata2, test::defaultDistanceFunction);
    S dist2 = cdata2.result.min_distance;

    for(std::size_t j = 0; j < boxes.size(); ++j)
    {
      test::DistanceData<S> cdatab;
      S dist = std::numeric_limits<S>::max();
      test::defaultDistanceFunction(&obj1, boxes[j], &cdatab, dist);
      delete boxes[j];
    }
    delete manager;

    EXPECT_NEAR(dist1, dist2, 1e-6);
  }
}

template <typename S>
void octomap_distance_test(S env_scale, std::size_t env_size, bool use_mesh, bool use_mesh_octomap, double resolution)
{
  // srand(1);
  std::vector<CollisionObject<S>*> env;
  if(use_mesh)
    test::generateEnvironmentsMesh(env, env_scale, env_size);
  else
    test::generateEnvironments(env, env_scale, env_size);

  OcTree<S>* tree = new OcTree<S>(std::shared_ptr<const octomap::OcTree>(test::generateOcTree(resolution)));
  CollisionObject<S> tree_obj((std::shared_ptr<CollisionGeometry<S>>(tree)));

  DynamicAABBTreeCollisionManager<S>* manager = new DynamicAABBTreeCollisionManager<S>();
  manager->registerObjects(env);
  manager->setup();

  test::DistanceData<S> cdata;
  test::TStruct t1;
  test::Timer timer1;
  timer1.start();
  manager->octree_as_geometry_collide = false;
  manager->octree_as_geometry_distance = false;
  manager->distance(&tree_obj, &cdata, test::defaultDistanceFunction);
  timer1.stop();
  t1.push_back(timer1.getElapsedTime());


  test::DistanceData<S> cdata3;
  test::TStruct t3;
  test::Timer timer3;
  timer3.start();
  manager->octree_as_geometry_collide = true;
  manager->octree_as_geometry_distance = true;
  manager->distance(&tree_obj, &cdata3, test::defaultDistanceFunction);
  timer3.stop();
  t3.push_back(timer3.getElapsedTime());


  test::TStruct t2;
  test::Timer timer2;
  timer2.start();
  std::vector<CollisionObject<S>*> boxes;
  if(use_mesh_octomap)
    test::generateBoxesFromOctomapMesh(boxes, *tree);
  else
    test::generateBoxesFromOctomap(boxes, *tree);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());

  timer2.start();
  DynamicAABBTreeCollisionManager<S>* manager2 = new DynamicAABBTreeCollisionManager<S>();
  manager2->registerObjects(boxes);
  manager2->setup();
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());


  test::DistanceData<S> cdata2;

  timer2.start();
  manager->distance(manager2, &cdata2, test::defaultDistanceFunction);
  timer2.stop();
  t2.push_back(timer2.getElapsedTime());

  std::cout << cdata.result.min_distance << " " << cdata3.result.min_distance << " " << cdata2.result.min_distance << std::endl;

  if(cdata.result.min_distance < 0)
    EXPECT_LE(cdata2.result.min_distance, 0);
  else
    EXPECT_NEAR(cdata.result.min_distance, cdata2.result.min_distance, 1e-3);

  delete manager;
  delete manager2;
  for(size_t i = 0; i < boxes.size(); ++i)
    delete boxes[i];


  std::cout << "1) octomap overall time: " << t1.overall_time << std::endl;
  std::cout << "1') octomap overall time (as geometry): " << t3.overall_time << std::endl;
  std::cout << "2) boxes overall time: " << t2.overall_time << std::endl;
  std::cout << "  a) to boxes: " << t2.records[0] << std::endl;
  std::cout << "  b) structure init: " << t2.records[1] << std::endl;
  std::cout << "  c) distance: " << t2.records[2] << std::endl;
  std::cout << "Note: octomap may need more collides when using mesh, because octomap collision uses box primitive inside" << std::endl;
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
