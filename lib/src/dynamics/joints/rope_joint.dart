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

part of box2d;

/**
 * A rope joint enforces a maximum distance between two points on two bodies. It has no other
 * effect. Warning: if you attempt to change the maximum length during the simulation you will get
 * some non-physical behavior. A model that would allow you to dynamically modify the length would
 * have some sponginess, so I chose not to implement it that way. See DistanceJoint if you want to
 * dynamically control length.
 *
 * @author Daniel Murphy
 */
class RopeJoint extends Joint {
  RopeJoint(IWorldPool worldPool, RopeJointDef def) : super(worldPool, def) {
    _localAnchorA.setFrom(def.localAnchorA);
    _localAnchorB.setFrom(def.localAnchorB);

    _maxLength = def.maxLength;
  }

  // Solver shared
  final Vector2 _localAnchorA = Vector2.zero();
  final Vector2 _localAnchorB = Vector2.zero();
  double _maxLength = 0.0;
  double _length = 0.0;
  double _impulse = 0.0;

  // Solver temp
  int _indexA = 0;
  int _indexB = 0;
  final Vector2 _u = Vector2.zero();
  final Vector2 _rA = Vector2.zero();
  final Vector2 _rB = Vector2.zero();
  final Vector2 _localCenterA = Vector2.zero();
  final Vector2 _localCenterB = Vector2.zero();
  double _invMassA = 0.0;
  double _invMassB = 0.0;
  double _invIA = 0.0;
  double _invIB = 0.0;
  double _mass = 0.0;
  LimitState _state = LimitState.INACTIVE;

  @override
  void initVelocityConstraints(final SolverData data) {
    _indexA = _bodyA.islandIndex;
    _indexB = _bodyB.islandIndex;
    _localCenterA.setFrom(_bodyA._sweep.localCenter);
    _localCenterB.setFrom(_bodyB._sweep.localCenter);
    _invMassA = _bodyA._invMass;
    _invMassB = _bodyB._invMass;
    _invIA = _bodyA._invI;
    _invIB = _bodyB._invI;

    final Vector2 cA = data.positions[_indexA].c;
    final double aA = data.positions[_indexA].a;
    final Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;

    final Vector2 cB = data.positions[_indexB].c;
    final double aB = data.positions[_indexB].a;
    final Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vector2 temp = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(
        qA,
        temp
          ..setFrom(_localAnchorA)
          ..sub(_localCenterA),
        _rA);
    Rot.mulToOutUnsafe(
        qB,
        temp
          ..setFrom(_localAnchorB)
          ..sub(_localCenterB),
        _rB);

    _u
      ..setFrom(cB)
      ..add(_rB)
      ..sub(cA)
      ..sub(_rA);

    _length = _u.length;

    final double C = _length - _maxLength;
    if (C > 0.0) {
      _state = LimitState.AT_UPPER;
    } else {
      _state = LimitState.INACTIVE;
    }

    if (_length > settings.linearSlop) {
      _u.scale(1.0 / _length);
    } else {
      _u.setZero();
      _mass = 0.0;
      _impulse = 0.0;
      pool.pushRot(2);
      pool.pushVec2(1);
      return;
    }

    // Compute effective mass.
    final double crA = _rA.cross(_u);
    final double crB = _rB.cross(_u);
    final double invMass =
        _invMassA + _invIA * crA * crA + _invMassB + _invIB * crB * crB;

    _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      _impulse *= data.step.dtRatio;

      final double Px = _impulse * _u.x;
      final double Py = _impulse * _u.y;
      vA.x -= _invMassA * Px;
      vA.y -= _invMassA * Py;
      wA -= _invIA * (_rA.x * Py - _rA.y * Px);

      vB.x += _invMassB * Px;
      vB.y += _invMassB * Py;
      wB += _invIB * (_rB.x * Py - _rB.y * Px);
    } else {
      _impulse = 0.0;
    }

    pool.pushRot(2);
    pool.pushVec2(1);

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
  }

  @override
  void solveVelocityConstraints(final SolverData data) {
    final Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    final Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    // Cdot = dot(u, v + cross(w, r))
    final Vector2 vpA = pool.popVec2();
    final Vector2 vpB = pool.popVec2();
    final Vector2 temp = pool.popVec2();

    _rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);

    final double C = _length - _maxLength;
    double Cdot = _u.dot(temp
      ..setFrom(vpB)
      ..sub(vpA));

    // Predictive constraint.
    if (C < 0.0) {
      Cdot += data.step.inverseDt * C;
    }

    double impulse = -_mass * Cdot;
    final double oldImpulse = _impulse;
    _impulse = math.min(0.0, _impulse + impulse);
    impulse = _impulse - oldImpulse;

    final double Px = impulse * _u.x;
    final double Py = impulse * _u.y;
    vA.x -= _invMassA * Px;
    vA.y -= _invMassA * Py;
    wA -= _invIA * (_rA.x * Py - _rA.y * Px);
    vB.x += _invMassB * Px;
    vB.y += _invMassB * Py;
    wB += _invIB * (_rB.x * Py - _rB.y * Px);

    pool.pushVec2(3);

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
  }

  @override
  bool solvePositionConstraints(final SolverData data) {
    final Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    final Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vector2 u = pool.popVec2();
    final Vector2 rA = pool.popVec2();
    final Vector2 rB = pool.popVec2();
    final Vector2 temp = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(
        qA,
        temp
          ..setFrom(_localAnchorA)
          ..sub(_localCenterA),
        rA);
    Rot.mulToOutUnsafe(
        qB,
        temp
          ..setFrom(_localAnchorB)
          ..sub(_localCenterB),
        rB);
    u
      ..setFrom(cB)
      ..add(rB)
      ..sub(cA)
      ..sub(rA);

    final double length = u.normalize();
    double C = length - _maxLength;

    C = math_utils.clampDouble(C, 0.0, settings.maxLinearCorrection);

    final double impulse = -_mass * C;
    final double Px = impulse * u.x;
    final double Py = impulse * u.y;

    cA.x -= _invMassA * Px;
    cA.y -= _invMassA * Py;
    aA -= _invIA * (rA.x * Py - rA.y * Px);
    cB.x += _invMassB * Px;
    cB.y += _invMassB * Py;
    aB += _invIB * (rB.x * Py - rB.y * Px);

    pool.pushRot(2);
    pool.pushVec2(4);

    // data.positions[_indexA].c = cA;
    data.positions[_indexA].a = aA;
    // data.positions[_indexB].c = cB;
    data.positions[_indexB].a = aB;

    return length - _maxLength < settings.linearSlop;
  }

  @override
  void getAnchorA(Vector2 out) {
    _bodyA.getWorldPointToOut(_localAnchorA, out);
  }

  @override
  void getAnchorB(Vector2 out) {
    _bodyB.getWorldPointToOut(_localAnchorB, out);
  }

  @override
  void getReactionForce(double inverseDt, Vector2 out) {
    out
      ..setFrom(_u)
      ..scale(inverseDt)
      ..scale(_impulse);
  }

  @override
  double getReactionTorque(double inverseDt) {
    return 0.0;
  }

  Vector2 getLocalAnchorA() {
    return _localAnchorA;
  }

  Vector2 getLocalAnchorB() {
    return _localAnchorB;
  }

  double getMaxLength() {
    return _maxLength;
  }

  void setMaxLength(double maxLength) {
    _maxLength = maxLength;
  }

  LimitState getLimitState() {
    return _state;
  }
}
