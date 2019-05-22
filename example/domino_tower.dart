/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

library domino_tower;

import 'dart:math' as math;
import 'package:box2d_flame/box2d.dart';

import 'demo.dart';

class DominoTower extends Demo {
  /** Construct a new DominoTower. */
  DominoTower() : super('Domino tower');

  static const double DOMINO_WIDTH = .2;
  static const double DOMINO_FRICTION = 0.1;
  static const double DOMINO_HEIGHT = 1.0;
  static const int BASE_COUNT = 25;

  /**
   * The density of the dominos under construction. Varies for different parts
   * of the tower.
   */
  double dominoDensity;

  /** Entrypoint. */
  static void main() {
    final DominoTower tower = DominoTower();
    tower.initialize();
    tower.initializeAnimation();
    tower.runAnimation();
  }

  void makeDomino(double x, double y, bool horizontal) {
    final PolygonShape sd = PolygonShape()
      ..setAsBoxXY(.5 * DOMINO_WIDTH, .5 * DOMINO_HEIGHT);
    final FixtureDef fd = FixtureDef()
      ..shape = sd
      ..density = dominoDensity
      ..friction = DOMINO_FRICTION
      ..restitution = 0.65;
    final BodyDef bd = BodyDef()
      ..type = BodyType.DYNAMIC
      ..position = Vector2(x, y)
      ..angle = horizontal ? (math.pi / 2.0) : 0.0;
    final Body myBody = world.createBody(bd);
    myBody.createFixtureFromFixtureDef(fd);
    bodies.add(myBody);
  }

  /**
   * Sets up the dominoes.
   */
  @override
  void initialize() {
    // Create the floor.
    {
      final PolygonShape sd = PolygonShape()..setAsBoxXY(50.0, 10.0);

      final BodyDef bd = BodyDef()..position = Vector2(0.0, -10.0);
      final Body body = world.createBody(bd);
      body.createFixtureFromShape(sd);
      bodies.add(body);
    }

    {
      dominoDensity = 10.0;
      // Make bullet
      final PolygonShape sd = PolygonShape()..setAsBoxXY(.7, .7);
      final FixtureDef fd = FixtureDef()
        ..density = 35.0
        ..shape = sd
        ..friction = 0.0
        ..restitution = 0.85;
      final BodyDef bd = BodyDef()
        ..type = BodyType.DYNAMIC
        ..bullet = true
        ..position = Vector2(30.0, 5.00);
      Body b = world.createBody(bd)
        ..createFixtureFromFixtureDef(fd)
        ..linearVelocity = Vector2(-25.0, -25.0)
        ..angularVelocity = 6.7;
      bodies.add(b);

      fd.density = 25.0;
      bd.position = Vector2(-30.0, 25.0);
      b = world.createBody(bd);
      bodies.add(b);
      b.createFixtureFromFixtureDef(fd);
      b.linearVelocity = Vector2(35.0, -10.0);
      b.angularVelocity = -8.3;
    }

    {
      double currX;
      // Make base
      for (int i = 0; i < BASE_COUNT; ++i) {
        currX =
            i * 1.5 * DOMINO_HEIGHT - (1.5 * DOMINO_HEIGHT * BASE_COUNT / 2);
        makeDomino(currX, DOMINO_HEIGHT / 2.0, false);
        makeDomino(currX, DOMINO_HEIGHT + DOMINO_WIDTH / 2.0, true);
      }
      currX = BASE_COUNT * 1.5 * DOMINO_HEIGHT -
          (1.5 * DOMINO_HEIGHT * BASE_COUNT / 2);

      // Make 'I's
      for (int j = 1; j < BASE_COUNT; ++j) {
        if (j > 3) dominoDensity *= .8;

        // The y at the center of the I structure.
        final double currY =
            DOMINO_HEIGHT * .5 + (DOMINO_HEIGHT + 2 * DOMINO_WIDTH) * .99 * j;

        for (int i = 0; i < BASE_COUNT - j; ++i) {
          currX = i * 1.5 * DOMINO_HEIGHT -
              (1.5 * DOMINO_HEIGHT * (BASE_COUNT - j) / 2);
          dominoDensity *= 2.5;
          if (i == 0) {
            makeDomino(currX - (1.25 * DOMINO_HEIGHT) + .5 * DOMINO_WIDTH,
                currY - DOMINO_WIDTH, false);
          }
          if (i == BASE_COUNT - j - 1) {
            makeDomino(currX + (1.25 * DOMINO_HEIGHT) - .5 * DOMINO_WIDTH,
                currY - DOMINO_WIDTH, false);
          }

          dominoDensity /= 2.5;
          makeDomino(currX, currY, false);
          makeDomino(currX, currY + .5 * (DOMINO_WIDTH + DOMINO_HEIGHT), true);
          makeDomino(currX, currY - .5 * (DOMINO_WIDTH + DOMINO_HEIGHT), true);
        }
      }
    }
  }
}

void main() {
  DominoTower.main();
}
