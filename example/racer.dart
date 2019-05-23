///*******************************************************************************
// * Copyright (c) 2015, Daniel Murphy, Google
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted provided that the following conditions are met:
// *  * Redistributions of source code must retain the above copyright notice,
// *    this list of conditions and the following disclaimer.
// *  * Redistributions in binary form must reproduce the above copyright notice,
// *    this list of conditions and the following disclaimer in the documentation
// *    and/or other materials provided with the distribution.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// * POSSIBILITY OF SUCH DAMAGE.
// ******************************************************************************/

library racer;

import 'dart:html' hide Body;
import 'dart:math';
import 'package:box2d_flame/box2d.dart';
import 'package:box2d_flame/src/math_utils.dart' as math_utils;

import 'demo.dart';

part 'racer/car.dart';

part 'racer/control_state.dart';

part 'racer/ground_area.dart';

part 'racer/tire.dart';

class Racer extends Demo implements ContactListener {
  Racer() : super('Racer', Vector2.zero(), 2.5);

  static void main() {
    final Racer racer = Racer();
    racer.initialize();
    racer.initializeAnimation();
    document.body.nodes
        .add(Element.html('<p>Use the arrow keys to drive the car.</p>'));
    racer.runAnimation();
  }

  @override
  void initialize() {
    _createGround();
    _createBoundary();

    _car = Car(world);
    _controlState = 0;

    // Bind to keyboard events.
    document.onKeyDown.listen(_handleKeyDown);
    document.onKeyUp.listen(_handleKeyUp);

    // Add ourselves as a collision listener.
    world.setContactListener(this);
  }

  @override
  void step(num time) {
    _car.update(time - _lastTime, _controlState);
    _lastTime = time;
    super.step(time);
  }

  // ContactListener overrides.
  @override
  void beginContact(Contact contact) {
    _handleContact(contact, true);
  }

  @override
  void endContact(Contact contact) {
    _handleContact(contact, false);
  }

  @override
  void preSolve(Contact contact, Manifold oldManifold) {}

  @override
  void postSolve(Contact contact, ContactImpulse impulse) {}

  double radians(double deg) => deg * (pi / 180.0);

  void _createGround() {
    final BodyDef def = BodyDef();
    _groundBody = world.createBody(def);
    _groundBody.userData = 'Ground';

    final PolygonShape shape = PolygonShape()
      ..setAsBox(27.0, 21.0, Vector2(-30.0, 30.0), radians(20.0));

    final FixtureDef fixtureDef = FixtureDef()
      ..shape = shape
      ..isSensor = true
      ..userData = GroundArea(0.001, false);

    _groundBody.createFixtureFromFixtureDef(fixtureDef);

    fixtureDef.userData = GroundArea(0.2, false);
    shape.setAsBox(27.0, 15.0, Vector2(20.0, 40.0), radians(-40.0));
    _groundBody.createFixtureFromFixtureDef(fixtureDef);
  }

  void _createBoundary() {
    final BodyDef def = BodyDef();
    final Body boundaryBody = world.createBody(def);
    boundaryBody.userData = 'Boundary';

    final PolygonShape shape = PolygonShape();

    final FixtureDef fixtureDef = FixtureDef();
    fixtureDef.shape = shape;

    const double boundaryX = 150.0;
    const double boundaryY = 100.0;

    shape.setAsEdge(
        Vector2(-boundaryX, -boundaryY), Vector2(boundaryX, -boundaryY));

    boundaryBody.createFixtureFromFixtureDef(fixtureDef);

    shape.setAsEdge(
        Vector2(boundaryX, -boundaryY), Vector2(boundaryX, boundaryY));
    boundaryBody.createFixtureFromFixtureDef(fixtureDef);

    shape.setAsEdge(
        Vector2(boundaryX, boundaryY), Vector2(-boundaryX, boundaryY));
    boundaryBody.createFixtureFromFixtureDef(fixtureDef);

    shape.setAsEdge(
        Vector2(-boundaryX, boundaryY), Vector2(-boundaryX, -boundaryY));
    boundaryBody.createFixtureFromFixtureDef(fixtureDef);
  }

  void _handleKeyDown(KeyboardEvent event) {
    switch (event.keyCode) {
      case 37:
        _controlState |= ControlState.LEFT;
        break;
      case 38:
        _controlState |= ControlState.UP;
        break;
      case 39:
        _controlState |= ControlState.RIGHT;
        break;
      case 40:
        _controlState |= ControlState.DOWN;
        break;
    }
  }

  void _handleKeyUp(KeyboardEvent event) {
    switch (event.keyCode) {
      case 37:
        _controlState &= ~ControlState.LEFT;
        break;
      case 38:
        _controlState &= ~ControlState.UP;
        break;
      case 39:
        _controlState &= ~ControlState.RIGHT;
        break;
      case 40:
        _controlState &= ~ControlState.DOWN;
        break;
    }
  }

  // TODO(unassigned): collision filtering.
  //   Tire with Boundary
  //   Tire with GroundArea
  void _handleContact(Contact contact, bool began) {
    final Object fudA = contact.fixtureA.userData;
    final Object fudB = contact.fixtureB.userData;

    // Check for ground area collision.
    // TODO(unassigned): named parameters instead of swapping order?
    if (fudA is Tire && fudB is GroundArea) {
      _tireVsGroundArea(fudA, fudB, began);
    } else if (fudA is GroundArea && fudB is Tire) {
      _tireVsGroundArea(fudB, fudA, began);
    }
  }

  void _tireVsGroundArea(Tire tire, GroundArea groundArea, bool began) {
    if (began) {
      tire.addGroundArea(groundArea);
    } else {
      tire.removeGroundArea(groundArea);
    }
  }

  int _controlState;
  Body _groundBody;
  Car _car;
  num _lastTime = 0;
}

void main() {
  Racer.main();
}
