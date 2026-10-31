// =============================================================================
// Kinema2D —— 一个用于学习的 2D 刚体物理引擎
//
// 使用方式：#include <kinema/Kinema.h>
//
//   kinema::World world;
//   kinema::BodyDef def;
//   def.position = {0, 10};
//   world.CreateBody(def, kinema::CircleShape(0.5f));
//   for (int i = 0; i < 60; ++i) world.Step(1.0f / 60.0f);
// =============================================================================
#pragma once

#include "kinema/Version.h"
#include "kinema/collision/BroadPhase.h"
#include "kinema/collision/Collision.h"
#include "kinema/collision/Shape.h"
#include "kinema/core/Timer.h"
#include "kinema/dynamics/Body.h"
#include "kinema/dynamics/Contact.h"
#include "kinema/dynamics/World.h"
#include "kinema/joints/Joint.h"
#include "kinema/math/AABB.h"
#include "kinema/math/Math.h"
