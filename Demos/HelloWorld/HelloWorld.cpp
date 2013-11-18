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
//
// 注：もし、hello_world_tへと整理する直前の状態の原作のソース構造に近いmainへのベタ書きのプログラムを見たい場合には、
// commit: 80a2cab70a50f0de077226e123575194168901d7 
// (github: https://github.com/usagi/usagi-bullet/blob/80a2cab70a50f0de077226e123575194168901d7/Demos/HelloWorld/HelloWorld.cpp )
// を確認すると良いでしょう。

///-----include群の開始-----
#include "btBulletDynamicsCommon.h"
#include <memory>
#include <forward_list>
#include <iostream>
///-----include群の終了-----

// これは基礎的なBullet物理シミュレーションを動作させるHello Worldプログラムだよ

/// Bulletによる最低限の物理シミュレーションの世界をまとめたクラス
template
< unsigned STEP_NUMERATOR   = 1
, unsigned STEP_DENOMINATOR = 60
, unsigned STEP_MAX_SUBSTEP = 10
, class COLLISION_CONFIGURATION_T = btDefaultCollisionConfiguration
, class COLLISION_DISPATCHER_T    = btCollisionDispatcher
, class BROADPHASE_INTERFACE_T    = btDbvtBroadphase
, class SOLVER_T                  = btSequentialImpulseConstraintSolver
, class WORLD_T                   = btDiscreteDynamicsWorld
>
struct hello_world_t final
{
  // テンプレート引数からメンバー定数をクラスに定義します
  static constexpr float     step_time         = float(STEP_NUMERATOR) / float(STEP_DENOMINATOR);
  static constexpr unsigned  step_max_substep  = STEP_MAX_SUBSTEP;
  
  // テンプレート引数からメンバー型をクラスに定義します
  using collision_configuration_t = COLLISION_CONFIGURATION_T;
  using collision_dispatcher_t    = COLLISION_DISPATCHER_T;
  using broadphase_interface_t    = BROADPHASE_INTERFACE_T;
  using solver_t                  = SOLVER_T;
  using world_t                   = WORLD_T;
  
  /// hello_world_tを構築します
  hello_world_t()
  { initialize(); }
  
  // 今回はコピーコンストラクターと代入演算子は面倒なので差し当たりdeleteしておきます
  hello_world_t(const hello_world_t&)   = delete;
  hello_world_t(hello_world_t&&)        = delete;
  void operator=(const hello_world_t&)  = delete;
  void operator=(hello_world_t&&)       = delete;
  
  /// 動力学の世界の時間を段階的に進めます
  void step()
  {
    // dynamicsWorldのシミュレーションステップを全体で step_time 秒だけ、最大 step_max_substep 分割して進めます
    world->stepSimulation( step_time, step_max_substep );
  }
  
  /// 物体の動作状態から現在の位置を標準出力します
  void print()
  {
    // 今回は実装上、motion_statesを列挙しても良いのですが、
    // 実際問題、物体を列挙してその物体の動作状態を得る方法を示した方が有用なのでその様な例示にしています
    for ( const auto& body : bodies )
    {
      // 物体の動作状態を取得します
      auto motion_state = body->getMotionState();
      // 丁寧には、動作状態を取得できているか確認しても構いませんが、
      // ここで取得できない様な状況ではそもそも世界のシミュレーションが正常に動作せず異常終了している事でしょう。
      //if(motion_state)
      //{
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
      //}
    }
  }
  
private:
  /// 初期化処理
  void initialize()
  {
    // 動力学の世界を初期化します
    initialize_world();
    // 物体を生成し、世界に放り込みます
    initialize_bodies();
  }
  
  /// 動力学の世界の初期化
  void initialize_world()
  {
    // デフォルトの衝突設定を行います。ユーザー独自の設定を作る事もできますよ。
    collision_configuration.reset(new collision_configuration_t());
    // デフォルトの衝突ディスパッチャーを使います。 他のディスパッチャーで並行処理にも対応できますよ（→ Extras/BulletMultiThread）
    collision_dispatcher.reset(new collision_dispatcher_t(collision_configuration.get()));
    // btDbvtBroadphaseは一般的には良いbroadphaseです。同じ様にしてbtAxis3Sweepも試せますよ。
    overlapping_pair_cache.reset(new broadphase_interface_t());
    // デフォルトの制約ソルバー。他のソルバーで並行処理にも対応できますよ（→ Extras/BulletMultiThreaded）
    solver.reset(new solver_t);
    
    // 動力学の世界を生成します
    std::unique_ptr<world_t> world
    ( new world_t
      ( collision_dispatcher.get()
      , overlapping_pair_cache.get()
      , solver.get()
      , collision_configuration.get()
      )
    );
    
    // 生成した世界に重力を設定します
    world->setGravity( btVector3(0, -10, 0) );
    
    // ローカルスコープのworldオブジェクトの管理をthis->worldへstd::moveしクラススコープに移管します
    this->world = std::move(world);
  }
  
  /// 基礎的な剛体群を生成し、世界への放り込みます
  void initialize_bodies()
  {
    // 静的な剛体の生成
    {
      // 衝突形状（btCollisionShape）としてgroundShapeを定義します
      // なお、できるだけ、剛体群は衝突形状（シェイプ）を使い回せる様にしましょう！
      std::unique_ptr<btCollisionShape> groundShape
      ( new btBoxShape
        ( btVector3( btScalar(50.), btScalar(50.), btScalar(50.) )
        )
      );
      
      // 静的な剛体を生成して、btDefaultMotionStateとbtRigidBodyのスマートポインターを含むタプルをスコープに維持します
      // 質量（mass）を0に設定すると静的な物体を生成できます
      create_rigidbody(0., btVector3(0, -56, 0), groundShape);
      
      // ローカルスコープのcolShapdeオブジェクトの管理をcollision_shapesへstd::moveしクラススコープに移管します
      collision_shapes.emplace_front(std::move(groundShape));
    }
    
    // 動的な剛体の生成
    {
      // これから生成する動的物体用に衝突形状を生成します
      // もし、衝突形状を変更すると動力学の世界の中での挙動ももちろん変化する事でしょう
      //std::unique_ptr<btCollisionShape> colShape( new btBoxShape( btVector3(1, 1, 1) ) );
      std::unique_ptr<btCollisionShape> colShape( new btSphereShape( btScalar(1.) ) );
      
      // 動的な剛体を生成して、btDefaultMotionStateとbtRigidBodyのスマートポインターを含むタプルをスコープに維持します
      // 質量（mass）を0以外に設定すると動的な物体を生成できます
      create_rigidbody(1.f, btVector3(2, 10, 0), colShape);
      
      // ローカルスコープのcolShapdeオブジェクトの管理をcollision_shapesへstd::moveしクラススコープに移管します
      collision_shapes.emplace_front(std::move(colShape));
    }
  }
  
  /// 剛体を質量（mass）、動作状態の初期ベクター（motion_transform_origin_vector）、 衝突形状（collision_shape）を元に生成し、
  /// 動力学の世界へ追加します。
  void create_rigidbody
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
    world->addRigidBody( body.get() );
    
    // ローカルスコープのmyMotionStateオブジェクトの管理をmotion_statesへstd::moveしクラススコープに移管します
    motion_states.emplace_front(std::move(myMotionState));
    // ローカルスコープのbodyオブジェクトの管理をbodiesへstd::moveしクラススコープに移管します
    bodies.emplace_front(std::move(body));
  }
  
  // クラススコープでBulletのオブジェクトを管理するためのスマートポインター群です
  std::unique_ptr<collision_configuration_t>  collision_configuration;
  std::unique_ptr<collision_dispatcher_t>     collision_dispatcher;
  std::unique_ptr<btBroadphaseInterface>      overlapping_pair_cache;
  std::unique_ptr<solver_t>                   solver;
  std::unique_ptr<world_t>                    world;
  
  // クラススコープでBulletのオブジェクト群を管理するためのスマートポインター群の群です
  std::forward_list<std::unique_ptr<btCollisionShape>> collision_shapes;
  std::forward_list<std::unique_ptr<btDefaultMotionState>> motion_states;
  std::forward_list<std::unique_ptr<btRigidBody>> bodies;
};

/// このプログラムのエントリーポイントです
int main()
{
  // hello_world_t<>型を定義します
  hello_world_t<> hello_world;
  
  // hello_worldオブジェクトに対して100回のstepとその都度にprintを呼び出します
  for(auto n = 0; n < 100; ++n)
  {
    hello_world.step();
    hello_world.print();
  }
}
