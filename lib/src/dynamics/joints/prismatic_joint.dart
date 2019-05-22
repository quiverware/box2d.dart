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

//Linear constraint (point-to-line)
//d = p2 - p1 = x2 + r2 - x1 - r1
//C = dot(perp, d)
//Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//   = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
//J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
//Angular constraint
//C = a2 - a1 + a_initial
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//
//K = J * invM * JT
//
//J = [-a -s1 a s2]
//  [0  -1  0  1]
//a = perp
//s1 = cross(d + r1, a) = cross(p2 - x1, a)
//s2 = cross(r2, a) = cross(p2 - x2, a)

//Motor/Limit linear constraint
//C = dot(ax1, d)
//Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
//J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

//Block Solver
//We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
//when the mass has poor distribution (leading to large torques about the joint anchor points).
//
//The Jacobian has 3 rows:
//J = [-uT -s1 uT s2] // linear
//  [0   -1   0  1] // angular
//  [-vT -a1 vT a2] // limit
//
//u = perp
//v = axis
//s1 = cross(d + r1, u), s2 = cross(r2, u)
//a1 = cross(d + r1, v), a2 = cross(r2, v)

//M * (v2 - v1) = JT * df
//J * v2 = bias
//
//v2 = v1 + invM * JT * df
//J * (v1 + invM * JT * df) = bias
//K * df = bias - J * v1 = -Cdot
//K = J * invM * JT
//Cdot = J * v1 - bias
//
//Now solve for f2.
//df = f2 - f1
//K * (f2 - f1) = -Cdot
//f2 = invK * (-Cdot) + f1
//
//Clamp accumulated limit impulse.
//lower: f2(3) = max(f2(3), 0)
//upper: f2(3) = min(f2(3), 0)
//
//Solve for correct f2(1:2)
//K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                    = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
//K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
//f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
//Now compute impulse to be applied:
//df = f2 - f1

/**
 * A prismatic joint. This joint provides one degree of freedom: translation along an axis fixed in
 * bodyA. Relative rotation is prevented. You can use a joint limit to restrict the range of motion
 * and a joint motor to drive the motion or to model joint friction.
 *
 * @author Daniel
 */
class PrismaticJoint extends Joint {
  // Solver shared
  final Vector2 _localAnchorA;
  final Vector2 _localAnchorB;
  final Vector2 _localXAxisA;
  final Vector2 _localYAxisA;
  double _referenceAngle;

  // TODO(srdjan): Make fields below private.
  final Vector3 _impulse = Vector3.zero();
  double _motorImpulse = 0.0;
  double _lowerTranslation = 0.0;
  double _upperTranslation = 0.0;
  double _maxMotorForce = 0.0;
  double _motorSpeed = 0.0;
  bool _enableLimit = false;
  bool _enableMotor = false;
  LimitState _limitState;

  // Solver temp
  int _indexA = 0;
  int _indexB = 0;
  final Vector2 _localCenterA = Vector2.zero();
  final Vector2 _localCenterB = Vector2.zero();
  double _invMassA = 0.0;
  double _invMassB = 0.0;
  double _invIA = 0.0;
  double _invIB = 0.0;
  final Vector2 _axis = Vector2.zero();
  final Vector2 _perp = Vector2.zero();
  double _s1 = 0.0, _s2 = 0.0;
  double _a1 = 0.0, _a2 = 0.0;
  final Matrix3 _K = Matrix3.zero();
  double _motorMass =
      0.0; // effective mass for motor/limit translational constraint.

  PrismaticJoint(IWorldPool argWorld, PrismaticJointDef def)
      : _localAnchorA = Vector2.copy(def.localAnchorA),
        _localAnchorB = Vector2.copy(def.localAnchorB),
        _localXAxisA = Vector2.copy(def.localAxisA)..normalize(),
        _localYAxisA = Vector2.zero(),
        super(argWorld, def) {
    _localXAxisA.scaleOrthogonalInto(1.0, _localYAxisA);
    _referenceAngle = def.referenceAngle;

    _lowerTranslation = def.lowerTranslation;
    _upperTranslation = def.upperTranslation;
    _maxMotorForce = def.maxMotorForce;
    _motorSpeed = def.motorSpeed;
    _enableLimit = def.enableLimit;
    _enableMotor = def.enableMotor;
    _limitState = LimitState.INACTIVE;
  }

  Vector2 getLocalAnchorA() {
    return _localAnchorA;
  }

  Vector2 getLocalAnchorB() {
    return _localAnchorB;
  }

  @override
  void getAnchorA(Vector2 argOut) {
    _bodyA.getWorldPointToOut(_localAnchorA, argOut);
  }

  @override
  void getAnchorB(Vector2 argOut) {
    _bodyB.getWorldPointToOut(_localAnchorB, argOut);
  }

  @override
  void getReactionForce(double inv_dt, Vector2 argOut) {
    final Vector2 temp = pool.popVec2();
    temp
      ..setFrom(_axis)
      ..scale(_motorImpulse + _impulse.z);
    argOut
      ..setFrom(_perp)
      ..scale(_impulse.x)
      ..add(temp)
      ..scale(inv_dt);
    pool.pushVec2(1);
  }

  @override
  double getReactionTorque(double inv_dt) {
    return inv_dt * _impulse.y;
  }

  /**
   * Get the current joint translation, usually in meters.
   */
  double getJointSpeed() {
    final Body bA = _bodyA;
    final Body bB = _bodyB;

    final Vector2 temp = pool.popVec2();
    final Vector2 rA = pool.popVec2();
    final Vector2 rB = pool.popVec2();
    final Vector2 p1 = pool.popVec2();
    final Vector2 p2 = pool.popVec2();
    final Vector2 d = pool.popVec2();
    final Vector2 axis = pool.popVec2();
    final Vector2 temp2 = pool.popVec2();
    final Vector2 temp3 = pool.popVec2();

    temp
      ..setFrom(_localAnchorA)
      ..sub(bA._sweep.localCenter);
    Rot.mulToOutUnsafe(bA._transform.q, temp, rA);

    temp
      ..setFrom(_localAnchorB)
      ..sub(bB._sweep.localCenter);
    Rot.mulToOutUnsafe(bB._transform.q, temp, rB);

    p1
      ..setFrom(bA._sweep.c)
      ..add(rA);
    p2
      ..setFrom(bB._sweep.c)
      ..add(rB);

    d
      ..setFrom(p2)
      ..sub(p1);
    Rot.mulToOutUnsafe(bA._transform.q, _localXAxisA, axis);

    final Vector2 vA = bA._linearVelocity;
    final Vector2 vB = bB._linearVelocity;
    final double wA = bA._angularVelocity;
    final double wB = bB._angularVelocity;

    axis.scaleOrthogonalInto(wA, temp);
    rB.scaleOrthogonalInto(wB, temp2);
    rA.scaleOrthogonalInto(wA, temp3);

    temp2
      ..add(vB)
      ..sub(vA)
      ..sub(temp3);
    final double speed = d.dot(temp) + axis.dot(temp2);

    pool.pushVec2(9);

    return speed;
  }

  double getJointTranslation() {
    final Vector2 pA = pool.popVec2(), pB = pool.popVec2(), axis = pool.popVec2();
    _bodyA.getWorldPointToOut(_localAnchorA, pA);
    _bodyB.getWorldPointToOut(_localAnchorB, pB);
    _bodyA.getWorldVectorToOutUnsafe(_localXAxisA, axis);
    pB.sub(pA);
    final double translation = pB.dot(axis);
    pool.pushVec2(3);
    return translation;
  }

  /**
   * Is the joint limit enabled?
   *
   * @return
   */
  bool isLimitEnabled() {
    return _enableLimit;
  }

  /**
   * Enable/disable the joint limit.
   *
   * @param flag
   */
  void enableLimit(bool flag) {
    if (flag != _enableLimit) {
      _bodyA.setAwake(true);
      _bodyB.setAwake(true);
      _enableLimit = flag;
      _impulse.z = 0.0;
    }
  }

  /**
   * Get the lower joint limit, usually in meters.
   *
   * @return
   */
  double getLowerLimit() {
    return _lowerTranslation;
  }

  /**
   * Get the upper joint limit, usually in meters.
   *
   * @return
   */
  double getUpperLimit() {
    return _upperTranslation;
  }

  /**
   * Set the joint limits, usually in meters.
   *
   * @param lower
   * @param upper
   */
  void setLimits(double lower, double upper) {
    assert(lower <= upper);
    if (lower != _lowerTranslation || upper != _upperTranslation) {
      _bodyA.setAwake(true);
      _bodyB.setAwake(true);
      _lowerTranslation = lower;
      _upperTranslation = upper;
      _impulse.z = 0.0;
    }
  }

  /**
   * Is the joint motor enabled?
   *
   * @return
   */
  bool isMotorEnabled() {
    return _enableMotor;
  }

  /**
   * Enable/disable the joint motor.
   *
   * @param flag
   */
  void enableMotor(bool flag) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _enableMotor = flag;
  }

  /**
   * Set the motor speed, usually in meters per second.
   *
   * @param speed
   */
  void setMotorSpeed(double speed) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _motorSpeed = speed;
  }

  /**
   * Get the motor speed, usually in meters per second.
   *
   * @return
   */
  double getMotorSpeed() {
    return _motorSpeed;
  }

  /**
   * Set the maximum motor force, usually in N.
   *
   * @param force
   */
  void setMaxMotorForce(double force) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _maxMotorForce = force;
  }

  /**
   * Get the current motor force, usually in N.
   *
   * @param inv_dt
   * @return
   */
  double getMotorForce(double inv_dt) {
    return _motorImpulse * inv_dt;
  }

  double getMaxMotorForce() {
    return _maxMotorForce;
  }

  double getReferenceAngle() {
    return _referenceAngle;
  }

  Vector2 getLocalAxisA() {
    return _localXAxisA;
  }

  @override
  void initVelocityConstraints(final SolverData data) {
    _indexA = _bodyA._islandIndex;
    _indexB = _bodyB._islandIndex;
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
    final Vector2 d = pool.popVec2();
    final Vector2 temp = pool.popVec2();
    final Vector2 rA = pool.popVec2();
    final Vector2 rB = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(
        qA,
        d
          ..setFrom(_localAnchorA)
          ..sub(_localCenterA),
        rA);
    Rot.mulToOutUnsafe(
        qB,
        d
          ..setFrom(_localAnchorB)
          ..sub(_localCenterB),
        rB);
    d
      ..setFrom(cB)
      ..sub(cA)
      ..add(rB)
      ..sub(rA);

    final double mA = _invMassA, mB = _invMassB;
    final double iA = _invIA, iB = _invIB;

    // Compute motor Jacobian and effective mass.
    {
      Rot.mulToOutUnsafe(qA, _localXAxisA, _axis);
      temp
        ..setFrom(d)
        ..add(rA);
      _a1 = temp.cross(_axis);
      _a2 = rB.cross(_axis);

      _motorMass = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;
      if (_motorMass > 0.0) {
        _motorMass = 1.0 / _motorMass;
      }
    }

    // Prismatic constraint.
    {
      Rot.mulToOutUnsafe(qA, _localYAxisA, _perp);

      temp
        ..setFrom(d)
        ..add(rA);
      _s1 = temp.cross(_perp);
      _s2 = rB.cross(_perp);

      final double k11 = mA + mB + iA * _s1 * _s1 + iB * _s2 * _s2;
      final double k12 = iA * _s1 + iB * _s2;
      final double k13 = iA * _s1 * _a1 + iB * _s2 * _a2;
      double k22 = iA + iB;
      if (k22 == 0.0) {
        // For bodies with fixed rotation.
        k22 = 1.0;
      }
      final double k23 = iA * _a1 + iB * _a2;
      final double k33 = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;

      _K.setValues(k11, k12, k13, k12, k22, k23, k13, k23, k33);
    }

    // Compute motor and limit terms.
    if (_enableLimit) {
      final double jointTranslation = _axis.dot(d);
      if ((_upperTranslation - _lowerTranslation).abs() <
          2.0 * Settings.linearSlop) {
        _limitState = LimitState.EQUAL;
      } else if (jointTranslation <= _lowerTranslation) {
        if (_limitState != LimitState.AT_LOWER) {
          _limitState = LimitState.AT_LOWER;
          _impulse.z = 0.0;
        }
      } else if (jointTranslation >= _upperTranslation) {
        if (_limitState != LimitState.AT_UPPER) {
          _limitState = LimitState.AT_UPPER;
          _impulse.z = 0.0;
        }
      } else {
        _limitState = LimitState.INACTIVE;
        _impulse.z = 0.0;
      }
    } else {
      _limitState = LimitState.INACTIVE;
      _impulse.z = 0.0;
    }

    if (_enableMotor == false) {
      _motorImpulse = 0.0;
    }

    if (data.step.warmStarting) {
      // Account for variable time step.
      _impulse.scale(data.step.dtRatio);
      _motorImpulse *= data.step.dtRatio;

      final Vector2 P = pool.popVec2();
      temp
        ..setFrom(_axis)
        ..scale(_motorImpulse + _impulse.z);
      P
        ..setFrom(_perp)
        ..scale(_impulse.x)
        ..add(temp);

      final double LA =
          _impulse.x * _s1 + _impulse.y + (_motorImpulse + _impulse.z) * _a1;
      final double LB =
          _impulse.x * _s2 + _impulse.y + (_motorImpulse + _impulse.z) * _a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(1);
    } else {
      _impulse.setZero();
      _motorImpulse = 0.0;
    }

    // data.velocities[_indexA].v.set(vA);
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;

    pool.pushRot(2);
    pool.pushVec2(4);
  }

  @override
  void solveVelocityConstraints(final SolverData data) {
    final Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    final Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final double mA = _invMassA, mB = _invMassB;
    final double iA = _invIA, iB = _invIB;

    final Vector2 temp = pool.popVec2();

    // Solve linear motor constraint.
    if (_enableMotor && _limitState != LimitState.EQUAL) {
      temp
        ..setFrom(vB)
        ..sub(vA);
      final double Cdot = _axis.dot(temp) + _a2 * wB - _a1 * wA;
      double impulse = _motorMass * (_motorSpeed - Cdot);
      final double oldImpulse = _motorImpulse;
      final double maxImpulse = data.step.dt * _maxMotorForce;
      _motorImpulse = MathUtils.clampDouble(
          _motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = _motorImpulse - oldImpulse;

      final Vector2 P = pool.popVec2();
      P
        ..setFrom(_axis)
        ..scale(impulse);
      final double LA = impulse * _a1;
      final double LB = impulse * _a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(1);
    }

    final Vector2 Cdot1 = pool.popVec2();
    temp
      ..setFrom(vB)
      ..sub(vA);
    Cdot1.x = _perp.dot(temp) + _s2 * wB - _s1 * wA;
    Cdot1.y = wB - wA;
    // System.out.println(Cdot1);

    if (_enableLimit && _limitState != LimitState.INACTIVE) {
      // Solve prismatic and limit constraint in block form.
      double Cdot2;
      temp
        ..setFrom(vB)
        ..sub(vA);
      Cdot2 = _axis.dot(temp) + _a2 * wB - _a1 * wA;

      final Vector3 Cdot = pool.popVec3();
      Cdot.setValues(Cdot1.x, Cdot1.y, Cdot2);

      final Vector3 f1 = pool.popVec3();
      final Vector3 df = pool.popVec3();

      f1.setFrom(_impulse);
      Matrix3.solve(_K, df, Cdot..negate());

      // Cdot.negateLocal(); not used anymore
      _impulse.add(df);

      if (_limitState == LimitState.AT_LOWER) {
        _impulse.z = Math.max(_impulse.z, 0.0);
      } else if (_limitState == LimitState.AT_UPPER) {
        _impulse.z = Math.min(_impulse.z, 0.0);
      }

      // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) +
      // f1(1:2)
      final Vector2 b = pool.popVec2();
      final Vector2 f2r = pool.popVec2();

      temp
        ..setValues(_K.entry(0, 2), _K.entry(1, 2))
        ..scale(_impulse.z - f1.z);
      b
        ..setFrom(Cdot1)
        ..negate()
        ..sub(temp);

      Matrix3.solve2(_K, f2r, b);
      f2r.add(Vector2(f1.x, f1.y));
      _impulse.x = f2r.x;
      _impulse.y = f2r.y;

      df
        ..setFrom(_impulse)
        ..sub(f1);

      final Vector2 P = pool.popVec2();
      temp
        ..setFrom(_axis)
        ..scale(df.z);
      P
        ..setFrom(_perp)
        ..scale(df.x)
        ..add(temp);

      final double LA = df.x * _s1 + df.y + df.z * _a1;
      final double LB = df.x * _s2 + df.y + df.z * _a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(3);
      pool.pushVec3(3);
    } else {
      // Limit is inactive, just solve the prismatic constraint in block form.
      final Vector2 df = pool.popVec2();
      Matrix3.solve2(_K, df, Cdot1..negate());
      Cdot1.negate();

      _impulse.x += df.x;
      _impulse.y += df.y;

      final Vector2 P = pool.popVec2();
      P
        ..setFrom(_perp)
        ..scale(df.x);
      final double LA = df.x * _s1 + df.y;
      final double LB = df.x * _s2 + df.y;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(2);
    }

    // data.velocities[_indexA].v.set(vA);
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;

    pool.pushVec2(2);
  }

  @override
  bool solvePositionConstraints(final SolverData data) {
    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vector2 rA = pool.popVec2();
    final Vector2 rB = pool.popVec2();
    final Vector2 d = pool.popVec2();
    final Vector2 axis = pool.popVec2();
    final Vector2 perp = pool.popVec2();
    final Vector2 temp = pool.popVec2();
    final Vector2 C1 = pool.popVec2();

    final Vector3 impulse = pool.popVec3();

    final Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    final Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    final double mA = _invMassA, mB = _invMassB;
    final double iA = _invIA, iB = _invIB;

    // Compute fresh Jacobians
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
    d
      ..setFrom(cB)
      ..add(rB)
      ..sub(cA)
      ..sub(rA);

    Rot.mulToOutUnsafe(qA, _localXAxisA, axis);
    final double a1 = (temp
          ..setFrom(d)
          ..add(rA))
        .cross(axis);
    final double a2 = rB.cross(axis);
    Rot.mulToOutUnsafe(qA, _localYAxisA, perp);

    final double s1 = (temp
          ..setFrom(d)
          ..add(rA))
        .cross(perp);
    final double s2 = rB.cross(perp);

    C1.x = perp.dot(d);
    C1.y = aB - aA - _referenceAngle;

    double linearError = C1.x.abs();
    final double angularError = C1.y.abs();

    bool active = false;
    double C2 = 0.0;
    if (_enableLimit) {
      final double translation = axis.dot(d);
      if ((_upperTranslation - _lowerTranslation).abs() <
          2.0 * Settings.linearSlop) {
        // Prevent large angular corrections
        C2 = MathUtils.clampDouble(translation, -Settings.maxLinearCorrection,
            Settings.maxLinearCorrection);
        linearError = Math.max(linearError, translation.abs());
        active = true;
      } else if (translation <= _lowerTranslation) {
        // Prevent large linear corrections and allow some slop.
        C2 = MathUtils.clampDouble(
            translation - _lowerTranslation + Settings.linearSlop,
            -Settings.maxLinearCorrection,
            0.0);
        linearError = Math.max(linearError, _lowerTranslation - translation);
        active = true;
      } else if (translation >= _upperTranslation) {
        // Prevent large linear corrections and allow some slop.
        C2 = MathUtils.clampDouble(
            translation - _upperTranslation - Settings.linearSlop,
            0.0,
            Settings.maxLinearCorrection);
        linearError = Math.max(linearError, translation - _upperTranslation);
        active = true;
      }
    }

    if (active) {
      final double k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
      final double k12 = iA * s1 + iB * s2;
      final double k13 = iA * s1 * a1 + iB * s2 * a2;
      double k22 = iA + iB;
      if (k22 == 0.0) {
        // For fixed rotation
        k22 = 1.0;
      }
      final double k23 = iA * a1 + iB * a2;
      final double k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

      final Matrix3 K = pool.popMat33();
      K.setValues(k11, k12, k13, k12, k22, k23, k13, k23, k33);

      final Vector3 C = pool.popVec3();
      C.x = C1.x;
      C.y = C1.y;
      C.z = C2;

      Matrix3.solve(K, impulse, C..negate());
      pool.pushVec3(1);
      pool.pushMat33(1);
    } else {
      final double k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
      final double k12 = iA * s1 + iB * s2;
      final double k22 = iA + iB == 0.0 ? 1.0 : iA + iB;

      final Matrix2 K = pool.popMat22();

      K.setValues(k11, k12, k12, k22);
      // temp is impulse1
      Matrix2.solve(K, temp, C1..negate());
      C1.negate();

      impulse.x = temp.x;
      impulse.y = temp.y;
      impulse.z = 0.0;

      pool.pushMat22(1);
    }

    final double Px = impulse.x * perp.x + impulse.z * axis.x;
    final double Py = impulse.x * perp.y + impulse.z * axis.y;
    final double LA = impulse.x * s1 + impulse.y + impulse.z * a1;
    final double LB = impulse.x * s2 + impulse.y + impulse.z * a2;

    cA.x -= mA * Px;
    cA.y -= mA * Py;
    aA -= iA * LA;
    cB.x += mB * Px;
    cB.y += mB * Py;
    aB += iB * LB;

    // data.positions[_indexA].c.set(cA);
    data.positions[_indexA].a = aA;
    // data.positions[_indexB].c.set(cB);
    data.positions[_indexB].a = aB;

    pool.pushVec2(7);
    pool.pushVec3(1);
    pool.pushRot(2);

    return linearError <= Settings.linearSlop &&
        angularError <= Settings.angularSlop;
  }
}
