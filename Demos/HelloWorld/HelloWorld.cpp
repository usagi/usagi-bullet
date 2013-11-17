/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

// 「うさぎ★ばれっと」プロジェクトによる改変
// https://github.com/usagi/usagi-bullet/commits/master/Demos/HelloWorld/HelloWorld.cpp
// Copyright (c) 2013 Usagi Ito <usagi@WonderRabbitProject.net>

///-----include群の開始-----
#include "btBulletDynamicsCommon.h"
#include <memory>
#include <tuple>
#include <iostream>
///-----include群の終了-----

/// これは基礎的なBullet物理シミュレーションを動作させるHello Worldプログラムだよ

int main()
{
  ///-----初期化の開始-----

  /// デフォルトの衝突設定を行います。ユーザー独自の設定を作る事もできますよ。
  std::unique_ptr<btDefaultCollisionConfiguration> collisionConfiguration(new btDefaultCollisionConfiguration());

  /// デフォルトの衝突ディスパッチャーを使います。 他のディスパッチャーで並行処理にも対応できますよ（→ Extras/BulletMultiThread）
  std::unique_ptr<btCollisionDispatcher> dispatcher(new btCollisionDispatcher(collisionConfiguration.get()));

  /// btDbvtBroadphaseは一般的には良いbroadphaseです。同じ様にしてbtAxis3Sweepも試せますよ。
  std::unique_ptr<btBroadphaseInterface> overlappingPairCache(new btDbvtBroadphase());

  /// デフォルトの制約ソルバー。他のソルバーで並行処理にも対応できますよ（→ Extras/BulletMultiThreaded）
  std::unique_ptr<btSequentialImpulseConstraintSolver> solver(new btSequentialImpulseConstraintSolver);

  /// 動力学の世界を生成します
  std::unique_ptr<btDiscreteDynamicsWorld> dynamicsWorld
  ( new btDiscreteDynamicsWorld
    ( dispatcher.get()
    , overlappingPairCache.get()
    , solver.get()
    , collisionConfiguration.get()
    )
  );

  /// dynamicsWorldに重力を設定します
  dynamicsWorld->setGravity( btVector3(0, -10, 0) );

  ///-----初期化の終了-----

  /// 基礎的な剛体群を生成します
  
  // 衝突形状（btCollisionShape）としてgroundShapeを定義します
  std::unique_ptr<btCollisionShape> groundShape
  ( new btBoxShape
    ( btVector3( btScalar(50.), btScalar(50.), btScalar(50.) )
    )
  );

  // できるだけ、剛体群が衝突形状（シェイプ）を流用する様にしましょう！
  btAlignedObjectArray<btCollisionShape*> collisionShapes;

  // 衝突形状groundShapeをcollisionShapesに追加します
  collisionShapes.push_back( groundShape.get() );

  // 剛体を質量（mass）、動作状態の初期ベクター（motion_transform_origin_vector）、 衝突形状（collision_shape）を元に生成するラムダ式を定義します
  auto create_rigidbody = [&]
  ( btScalar mass
  , const btVector3& motion_transform_origin_vector
  , const std::unique_ptr<btCollisionShape>& collision_shape
  )
  {
    // 動作状態を生成し、初期化し、motion_transform_origin_vectorを適用します
    btTransform motion_transform;
    motion_transform.setIdentity();
    motion_transform.setOrigin(motion_transform_origin_vector);
    
    // 剛体は、もしmassが非ゼロならば動的だし、そうでなければ静的なのだ
    bool isDynamic = (mass != 0.f);
    
    // 物体に働く局所的な慣性を定義します
    btVector3 localInertia(0, 0, 0);
    
    // 剛体が動的な場合には、衝突形状から局所的な慣性を計算します
    if (isDynamic)
      collision_shape->calculateLocalInertia(mass, localInertia);
    
    // motionstateの使用を推奨するよ、なぜなら'active'なオブジェクト群だけとの同期と補間機能を提供してくれるからだ
    std::unique_ptr<btDefaultMotionState> myMotionState( new btDefaultMotionState(motion_transform) );
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState.get(), collision_shape.get(), localInertia);
    std::unique_ptr<btRigidBody> body( new btRigidBody(rbInfo) );
    
    // 物体を動力学の世界へ追加します
    dynamicsWorld->addRigidBody( body.get() );
    
    // ラムダ式の外のスコープにスマートポインターであるmyMotionStateとbodyを移します
    return std::make_tuple( std::move(myMotionState), std::move(body) );
  };
  
  // 静的な剛体を生成して、btDefaultMotionStateとbtRigidBodyのスマートポインターを含むタプルをスコープに維持します
  auto body_1_dtor = create_rigidbody(0., btVector3(0,-56,0), groundShape);

  // 動的な剛体の生成

  // これから生成する動的物体用に衝突形状を生成し、collisionShapesに追加します
  //std::unique_ptr<btCollisionShape> colShape( new btBoxShape( btVector3(1,1,1) ) );
  std::unique_ptr<btCollisionShape> colShape( new btSphereShape( btScalar(1.) ) );
  collisionShapes.push_back( colShape.get() );
  
  /// 動的オブジェクトを生成します
  auto body_2_dtor = create_rigidbody(1.f, btVector3(2, 10, 0), colShape);


  /// このシミュレーションを実行します

  ///-----ステップシミュレーションの開始-----
  for(auto i = 0; i < 100; ++i)
  {
    // dynamicsWorldのシミュレーションステップを全体で1/60秒だけ、最大10分割して進めます
    dynamicsWorld->stepSimulation( 1.f / 60.f, 10 );
    
    // 全てのオブジェクトの位置を表示します
    for ( auto j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; --j )
    {
      // dynamicsWorldからbtCollisionObjectを1つ取り出します
      btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
      // btCollisionObjectをbtRigidBodyへアップキャストします
      btRigidBody* body = btRigidBody::upcast(obj);
      // bodyが取得できており、動作状態も取得できる場合には、
      if ( body && body->getMotionState() )
      {
        // 世界における物体の変形状態を取得して
        btTransform trans;
        body->getMotionState()->getWorldTransform(trans);
        // 取得した変形状態の座標(X,Y,Z)を表示します
        std::cout
          << "world pos = "
          << float(trans.getOrigin().getX()) << ","
          << float(trans.getOrigin().getY()) << ","
          << float(trans.getOrigin().getZ()) << "\n"
          ;
      }
    }
  }
  ///-----ステップシミュレーションの終了-----
}

