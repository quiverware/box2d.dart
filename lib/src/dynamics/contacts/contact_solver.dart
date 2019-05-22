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

class ContactSolverDef {
  TimeStep step;
  List<Contact> contacts;
  int count = 0;
  List<Position> positions;
  List<Velocity> velocities;
}

class ContactSolver {
  ContactSolver() {
    _positionConstraints =
        List<ContactPositionConstraint>(INITIAL_NUM_CONSTRAINTS);
    _velocityConstraints =
        List<ContactVelocityConstraint>(INITIAL_NUM_CONSTRAINTS);
    for (int i = 0; i < INITIAL_NUM_CONSTRAINTS; i++) {
      _positionConstraints[i] = ContactPositionConstraint();
      _velocityConstraints[i] = ContactVelocityConstraint();
    }
  }

  static const bool DEBUG_SOLVER = false;
  static const double k_errorTol = 1e-3;

  /**
   * For each solver, this is the initial number of constraints in the array, which expands as
   * needed.
   */
  static const int INITIAL_NUM_CONSTRAINTS = 256;

  /**
   * Ensure a reasonable condition number. for the block solver
   */
  static const double k_maxConditionNumber = 100.0;

  TimeStep _step;
  List<Position> _positions;
  List<Velocity> _velocities;
  List<ContactPositionConstraint> _positionConstraints;
  List<ContactVelocityConstraint> _velocityConstraints;
  List<Contact> _contacts;
  int _count = 0;

  void init(ContactSolverDef def) {
    // System.out.println("Initializing contact solver");
    _step = def.step;
    _count = def.count;

    if (_positionConstraints.length < _count) {
      final List<ContactPositionConstraint> old = _positionConstraints;
      _positionConstraints =
          List<ContactPositionConstraint>(math.max(old.length * 2, _count));
      buffer_utils.arraycopy(old, 0, _positionConstraints, 0, old.length);
      for (int i = old.length; i < _positionConstraints.length; i++) {
        _positionConstraints[i] = ContactPositionConstraint();
      }
    }

    if (_velocityConstraints.length < _count) {
      final List<ContactVelocityConstraint> old = _velocityConstraints;
      _velocityConstraints =
          List<ContactVelocityConstraint>(math.max(old.length * 2, _count));
      buffer_utils.arraycopy(old, 0, _velocityConstraints, 0, old.length);
      for (int i = old.length; i < _velocityConstraints.length; i++) {
        _velocityConstraints[i] = ContactVelocityConstraint();
      }
    }

    _positions = def.positions;
    _velocities = def.velocities;
    _contacts = def.contacts;

    for (int i = 0; i < _count; ++i) {
      // System.out.println("contacts: " + _count);
      final Contact contact = _contacts[i];

      final Fixture fixtureA = contact._fixtureA;
      final Fixture fixtureB = contact._fixtureB;
      final Shape shapeA = fixtureA.getShape();
      final Shape shapeB = fixtureB.getShape();
      final double radiusA = shapeA.radius;
      final double radiusB = shapeB.radius;
      final Body bodyA = fixtureA.getBody();
      final Body bodyB = fixtureB.getBody();
      final Manifold manifold = contact._manifold;

      final int pointCount = manifold.pointCount;
      assert(pointCount > 0);

      final ContactVelocityConstraint vc = _velocityConstraints[i]
        ..friction = contact._friction
        ..restitution = contact._restitution
        ..tangentSpeed = contact._tangentSpeed
        ..indexA = bodyA.islandIndex
        ..indexB = bodyB.islandIndex
        ..invMassA = bodyA._invMass
        ..invMassB = bodyB._invMass
        ..invIA = bodyA._invI
        ..invIB = bodyB._invI
        ..contactIndex = i
        ..pointCount = pointCount
        ..K.setZero()
        ..normalMass.setZero();

      final ContactPositionConstraint pc = _positionConstraints[i]
        ..indexA = bodyA.islandIndex
        ..indexB = bodyB.islandIndex
        ..invMassA = bodyA._invMass
        ..invMassB = bodyB._invMass
        ..localCenterA.setFrom(bodyA._sweep.localCenter)
        ..localCenterB.setFrom(bodyB._sweep.localCenter)
        ..invIA = bodyA._invI
        ..invIB = bodyB._invI
        ..localNormal.setFrom(manifold.localNormal)
        ..localPoint.setFrom(manifold.localPoint)
        ..pointCount = pointCount
        ..radiusA = radiusA
        ..radiusB = radiusB
        ..type = manifold.type;

      // System.out.println("contact point count: " + pointCount);
      for (int j = 0; j < pointCount; j++) {
        final ManifoldPoint cp = manifold.points[j];
        final VelocityConstraintPoint vcp = vc.points[j];

        if (_step.warmStarting) {
          // assert(cp.normalImpulse == 0);
          // System.out.println("contact normal impulse: " + cp.normalImpulse);
          vcp.normalImpulse = _step.dtRatio * cp.normalImpulse;
          vcp.tangentImpulse = _step.dtRatio * cp.tangentImpulse;
        } else {
          vcp.normalImpulse = 0.0;
          vcp.tangentImpulse = 0.0;
        }

        vcp.rA.setZero();
        vcp.rB.setZero();
        vcp.normalMass = 0.0;
        vcp.tangentMass = 0.0;
        vcp.velocityBias = 0.0;
        pc.localPoints[j].x = cp.localPoint.x;
        pc.localPoints[j].y = cp.localPoint.y;
      }
    }
  }

  void warmStart() {
    // Warm start.
    for (int i = 0; i < _count; ++i) {
      final ContactVelocityConstraint vc = _velocityConstraints[i];

      final int indexA = vc.indexA;
      final int indexB = vc.indexB;
      final double mA = vc.invMassA;
      final double iA = vc.invIA;
      final double mB = vc.invMassB;
      final double iB = vc.invIB;
      final int pointCount = vc.pointCount;

      final Vector2 vA = _velocities[indexA].v;
      double wA = _velocities[indexA].w;
      final Vector2 vB = _velocities[indexB].v;
      double wB = _velocities[indexB].w;

      final Vector2 normal = vc.normal;
      final double tangentx = 1.0 * normal.y;
      final double tangenty = -1.0 * normal.x;

      for (int j = 0; j < pointCount; ++j) {
        final VelocityConstraintPoint vcp = vc.points[j];
        final double Px =
            tangentx * vcp.tangentImpulse + normal.x * vcp.normalImpulse;
        final double Py =
            tangenty * vcp.tangentImpulse + normal.y * vcp.normalImpulse;

        wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);
        vA.x -= Px * mA;
        vA.y -= Py * mA;
        wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);
        vB.x += Px * mB;
        vB.y += Py * mB;
      }
      _velocities[indexA].w = wA;
      _velocities[indexB].w = wB;
    }
  }

  // djm pooling, and from above
  // TODO(srdjan): make them private.
  final Transform xfA = Transform.zero();
  final Transform xfB = Transform.zero();
  final WorldManifold worldManifold = WorldManifold();

  void initializeVelocityConstraints() {
    // Warm start.
    for (int i = 0; i < _count; ++i) {
      final ContactVelocityConstraint vc = _velocityConstraints[i];
      final ContactPositionConstraint pc = _positionConstraints[i];

      final double radiusA = pc.radiusA;
      final double radiusB = pc.radiusB;
      final Manifold manifold = _contacts[vc.contactIndex]._manifold;

      final int indexA = vc.indexA;
      final int indexB = vc.indexB;

      final double mA = vc.invMassA;
      final double mB = vc.invMassB;
      final double iA = vc.invIA;
      final double iB = vc.invIB;
      final Vector2 localCenterA = pc.localCenterA;
      final Vector2 localCenterB = pc.localCenterB;

      final Vector2 cA = _positions[indexA].c;
      final double aA = _positions[indexA].a;
      final Vector2 vA = _velocities[indexA].v;
      final double wA = _velocities[indexA].w;

      final Vector2 cB = _positions[indexB].c;
      final double aB = _positions[indexB].a;
      final Vector2 vB = _velocities[indexB].v;
      final double wB = _velocities[indexB].w;

      assert(manifold.pointCount > 0);

      final Rot xfAq = xfA.q;
      final Rot xfBq = xfB.q;
      xfAq.setAngle(aA);
      xfBq.setAngle(aB);
      xfA.p.x = cA.x - (xfAq.c * localCenterA.x - xfAq.s * localCenterA.y);
      xfA.p.y = cA.y - (xfAq.s * localCenterA.x + xfAq.c * localCenterA.y);
      xfB.p.x = cB.x - (xfBq.c * localCenterB.x - xfBq.s * localCenterB.y);
      xfB.p.y = cB.y - (xfBq.s * localCenterB.x + xfBq.c * localCenterB.y);

      worldManifold.initialize(manifold, xfA, radiusA, xfB, radiusB);

      final Vector2 vcnormal = vc.normal;
      vcnormal.x = worldManifold.normal.x;
      vcnormal.y = worldManifold.normal.y;

      final int pointCount = vc.pointCount;
      for (int j = 0; j < pointCount; ++j) {
        final VelocityConstraintPoint vcp = vc.points[j];
        final Vector2 wmPj = worldManifold.points[j];
        final Vector2 vcprA = vcp.rA;
        final Vector2 vcprB = vcp.rB;
        vcprA.x = wmPj.x - cA.x;
        vcprA.y = wmPj.y - cA.y;
        vcprB.x = wmPj.x - cB.x;
        vcprB.y = wmPj.y - cB.y;

        final double rnA = vcprA.x * vcnormal.y - vcprA.y * vcnormal.x;
        final double rnB = vcprB.x * vcnormal.y - vcprB.y * vcnormal.x;

        final double kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        vcp.normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0;

        final double tangentx = 1.0 * vcnormal.y;
        final double tangenty = -1.0 * vcnormal.x;

        final double rtA = vcprA.x * tangenty - vcprA.y * tangentx;
        final double rtB = vcprB.x * tangenty - vcprB.y * tangentx;

        final double kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

        vcp.tangentMass = kTangent > 0.0 ? 1.0 / kTangent : 0.0;

        // Setup a velocity bias for restitution.
        vcp.velocityBias = 0.0;
        final double tempx = vB.x + -wB * vcprB.y - vA.x - (-wA * vcprA.y);
        final double tempy = vB.y + wB * vcprB.x - vA.y - (wA * vcprA.x);
        final double vRel = vcnormal.x * tempx + vcnormal.y * tempy;
        if (vRel < -settings.velocityThreshold) {
          vcp.velocityBias = -vc.restitution * vRel;
        }
      }

      // If we have two points, then prepare the block solver.
      if (vc.pointCount == 2) {
        final VelocityConstraintPoint vcp1 = vc.points[0];
        final VelocityConstraintPoint vcp2 = vc.points[1];
        final double rn1A = vcp1.rA.x * vcnormal.y - vcp1.rA.y * vcnormal.x;
        final double rn1B = vcp1.rB.x * vcnormal.y - vcp1.rB.y * vcnormal.x;
        final double rn2A = vcp2.rA.x * vcnormal.y - vcp2.rA.y * vcnormal.x;
        final double rn2B = vcp2.rB.x * vcnormal.y - vcp2.rB.y * vcnormal.x;

        final double k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
        final double k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
        final double k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;
        if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
          // K is safe to invert.
          vc.K.setValues(k11, k12, k12, k22);
          vc.normalMass.setFrom(vc.K);
          vc.normalMass.invert();
        } else {
          // The constraints are redundant, just use one.
          // TODO_ERIN use deepest?
          vc.pointCount = 1;
        }
      }
    }
  }

  void solveVelocityConstraints() {
    for (int i = 0; i < _count; ++i) {
      final ContactVelocityConstraint vc = _velocityConstraints[i];

      final int indexA = vc.indexA;
      final int indexB = vc.indexB;

      final double mA = vc.invMassA;
      final double mB = vc.invMassB;
      final double iA = vc.invIA;
      final double iB = vc.invIB;
      final int pointCount = vc.pointCount;

      final Vector2 vA = _velocities[indexA].v;
      double wA = _velocities[indexA].w;
      final Vector2 vB = _velocities[indexB].v;
      double wB = _velocities[indexB].w;

      final Vector2 normal = vc.normal;
      final double normalx = normal.x;
      final double normaly = normal.y;
      final double tangentx = 1.0 * vc.normal.y;
      final double tangenty = -1.0 * vc.normal.x;
      final double friction = vc.friction;

      assert(pointCount == 1 || pointCount == 2);

      // Solve tangent constraints
      for (int j = 0; j < pointCount; ++j) {
        final VelocityConstraintPoint vcp = vc.points[j];
        final Vector2 a = vcp.rA;
        final double dvx = -wB * vcp.rB.y + vB.x - vA.x + wA * a.y;
        final double dvy = wB * vcp.rB.x + vB.y - vA.y - wA * a.x;

        // Compute tangent force
        final double vt = dvx * tangentx + dvy * tangenty - vc.tangentSpeed;
        double lambda = vcp.tangentMass * (-vt);

        // Clamp the accumulated force
        final double maxFriction = friction * vcp.normalImpulse;
        final double newImpulse = math_utils.clampDouble(
            vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
        lambda = newImpulse - vcp.tangentImpulse;
        vcp.tangentImpulse = newImpulse;

        // Apply contact impulse
        // Vec2 P = lambda * tangent;

        final double Px = tangentx * lambda;
        final double Py = tangenty * lambda;

        // vA -= invMassA * P;
        vA.x -= Px * mA;
        vA.y -= Py * mA;
        wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);

        // vB += invMassB * P;
        vB.x += Px * mB;
        vB.y += Py * mB;
        wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);
      }

      // Solve normal constraints
      if (vc.pointCount == 1) {
        final VelocityConstraintPoint vcp = vc.points[0];

        // Relative velocity at contact
        // Vec2 dv = vB + Cross(wB, vcp.rB) - vA - Cross(wA, vcp.rA);

        final double dvx = -wB * vcp.rB.y + vB.x - vA.x + wA * vcp.rA.y;
        final double dvy = wB * vcp.rB.x + vB.y - vA.y - wA * vcp.rA.x;

        // Compute normal impulse
        final double vn = dvx * normalx + dvy * normaly;
        double lambda = -vcp.normalMass * (vn - vcp.velocityBias);

        // Clamp the accumulated impulse
        final double a = vcp.normalImpulse + lambda;
        final double newImpulse = a > 0.0 ? a : 0.0;
        lambda = newImpulse - vcp.normalImpulse;
        vcp.normalImpulse = newImpulse;

        // Apply contact impulse
        final double Px = normalx * lambda;
        final double Py = normaly * lambda;

        // vA -= invMassA * P;
        vA.x -= Px * mA;
        vA.y -= Py * mA;
        wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);

        // vB += invMassB * P;
        vB.x += Px * mB;
        vB.y += Py * mB;
        wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);
      } else {
        // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on
        // Box2D_Lite).
        // Build the mini LCP for this contact patch
        //
        // vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
        //
        // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
        // b = vn_0 - velocityBias
        //
        // The system is solved using the "Total enumeration method" (s. Murty). The complementary
        // constraint vn_i * x_i
        // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D
        // contact problem the cases
        // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be
        // tested. The first valid
        // solution that satisfies the problem is chosen.
        //
        // In order to account of the accumulated impulse 'a' (because of the iterative nature of
        // the solver which only requires
        // that the accumulated impulse is clamped and not the incremental impulse) we change the
        // impulse variable (x_i).
        //
        // Substitute:
        //
        // x = a + d
        //
        // a := old total impulse
        // x := new total impulse
        // d := incremental impulse
        //
        // For the current iteration we extend the formula for the incremental impulse
        // to compute the new total impulse:
        //
        // vn = A * d + b
        // = A * (x - a) + b
        // = A * x + b - A * a
        // = A * x + b'
        // b' = b - A * a;

        final VelocityConstraintPoint cp1 = vc.points[0];
        final VelocityConstraintPoint cp2 = vc.points[1];
        final Vector2 cp1rA = cp1.rA;
        final Vector2 cp1rB = cp1.rB;
        final Vector2 cp2rA = cp2.rA;
        final Vector2 cp2rB = cp2.rB;
        final double ax = cp1.normalImpulse;
        final double ay = cp2.normalImpulse;

        assert(ax >= 0.0 && ay >= 0.0);
        // Relative velocity at contact
        // Vec2 dv1 = vB + Cross(wB, cp1.rB) - vA - Cross(wA, cp1.rA);
        final double dv1x = -wB * cp1rB.y + vB.x - vA.x + wA * cp1rA.y;
        final double dv1y = wB * cp1rB.x + vB.y - vA.y - wA * cp1rA.x;

        // Vec2 dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);
        final double dv2x = -wB * cp2rB.y + vB.x - vA.x + wA * cp2rA.y;
        final double dv2y = wB * cp2rB.x + vB.y - vA.y - wA * cp2rA.x;

        // Compute normal velocity
        double vn1 = dv1x * normalx + dv1y * normaly;
        double vn2 = dv2x * normalx + dv2y * normaly;

        double bx = vn1 - cp1.velocityBias;
        double by = vn2 - cp2.velocityBias;

        // Compute b'
        final Matrix2 R = vc.K;
        bx -= R.entry(0, 0) * ax + R.entry(0, 1) * ay;
        by -= R.entry(1, 0) * ax + R.entry(1, 1) * ay;

        // final double k_errorTol = 1e-3f;
        // B2_NOT_USED(k_errorTol);
        for (;;) {
          //
          // Case 1: vn = 0
          //
          // 0 = A * x' + b'
          //
          // Solve for x':
          //
          // x' = - inv(A) * b'
          //
          // Vec2 x = - Mul(c.normalMass, b);
          final Matrix2 R1 = vc.normalMass;
          double xx = R1.entry(0, 0) * bx + R1.entry(0, 1) * by;
          double xy = R1.entry(1, 0) * bx + R1.entry(1, 1) * by;
          xx *= -1;
          xy *= -1;

          if (xx >= 0.0 && xy >= 0.0) {
            // Get the incremental impulse
            // Vec2 d = x - a;
            final double dx = xx - ax;
            final double dy = xy - ay;

            // Apply incremental impulse
            // Vec2 P1 = d.x * normal;
            // Vec2 P2 = d.y * normal;
            final double P1x = dx * normalx;
            final double P1y = dx * normaly;
            final double P2x = dy * normalx;
            final double P2y = dy * normaly;

            /*
             * vA -= invMassA * (P1 + P2); wA -= invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA *
                (cp1rA.x * P1y -
                    cp1rA.y * P1x +
                    (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB *
                (cp1rB.x * P1y -
                    cp1rB.y * P1x +
                    (cp2rB.x * P2y - cp2rB.y * P2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            /*
             * #if B2_DEBUG_SOLVER == 1 // Postconditions dv1 = vB + Cross(wB, cp1.rB) - vA -
             * Cross(wA, cp1.rA); dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);
             *
             * // Compute normal velocity vn1 = Dot(dv1, normal); vn2 = Dot(dv2, normal);
             *
             * assert(Abs(vn1 - cp1.velocityBias) < k_errorTol); assert(Abs(vn2 - cp2.velocityBias)
             * < k_errorTol); #endif
             */
            if (DEBUG_SOLVER) {
              // Postconditions
              final Vector2 dv1 = vB + math_utils.crossDblVec2(wB, cp1rB)
                ..sub(vA)
                ..sub(math_utils.crossDblVec2(wA, cp1rA));
              final Vector2 dv2 = vB + math_utils.crossDblVec2(wB, cp2rB)
                ..sub(vA)
                ..sub(math_utils.crossDblVec2(wA, cp2rA));
              // Compute normal velocity
              vn1 = dv1.dot(normal);
              vn2 = dv2.dot(normal);

              assert((vn1 - cp1.velocityBias).abs() < k_errorTol);
              assert((vn2 - cp2.velocityBias).abs() < k_errorTol);
            }
            break;
          }

          //
          // Case 2: vn1 = 0 and x2 = 0
          //
          // 0 = a11 * x1' + a12 * 0 + b1'
          // vn2 = a21 * x1' + a22 * 0 + '
          //
          xx = -cp1.normalMass * bx;
          xy = 0.0;
          vn1 = 0.0;
          vn2 = vc.K.entry(1, 0) * xx + by;

          if (xx >= 0.0 && vn2 >= 0.0) {
            // Get the incremental impulse
            final double dx = xx - ax;
            final double dy = xy - ay;

            // Apply incremental impulse
            // Vec2 P1 = d.x * normal;
            // Vec2 P2 = d.y * normal;
            final double P1x = normalx * dx;
            final double P1y = normaly * dx;
            final double P2x = normalx * dy;
            final double P2y = normaly * dy;

            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA *
                (cp1rA.x * P1y -
                    cp1rA.y * P1x +
                    (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB *
                (cp1rB.x * P1y -
                    cp1rB.y * P1x +
                    (cp2rB.x * P2y - cp2rB.y * P2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            /*
             * #if B2_DEBUG_SOLVER == 1 // Postconditions dv1 = vB + Cross(wB, cp1.rB) - vA -
             * Cross(wA, cp1.rA);
             *
             * // Compute normal velocity vn1 = Dot(dv1, normal);
             *
             * assert(Abs(vn1 - cp1.velocityBias) < k_errorTol); #endif
             */
            if (DEBUG_SOLVER) {
              // Postconditions
              final Vector2 dv1 = vB + math_utils.crossDblVec2(wB, cp1rB)
                ..sub(vA)
                ..sub(math_utils.crossDblVec2(wA, cp1rA));
              // Compute normal velocity
              vn1 = dv1.dot(normal);

              assert((vn1 - cp1.velocityBias).abs() < k_errorTol);
            }
            break;
          }

          //
          // Case 3: wB = 0 and x1 = 0
          //
          // vn1 = a11 * 0 + a12 * x2' + b1'
          // 0 = a21 * 0 + a22 * x2' + '
          //
          xx = 0.0;
          xy = -cp2.normalMass * by;
          vn1 = vc.K.entry(0, 1) * xy + bx;
          vn2 = 0.0;

          if (xy >= 0.0 && vn1 >= 0.0) {
            // Resubstitute for the incremental impulse
            final double dx = xx - ax;
            final double dy = xy - ay;

            // Apply incremental impulse
            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            final double P1x = normalx * dx;
            final double P1y = normaly * dx;
            final double P2x = normalx * dy;
            final double P2y = normaly * dy;

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA *
                (cp1rA.x * P1y -
                    cp1rA.y * P1x +
                    (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB *
                (cp1rB.x * P1y -
                    cp1rB.y * P1x +
                    (cp2rB.x * P2y - cp2rB.y * P2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            /*
             * #if B2_DEBUG_SOLVER == 1 // Postconditions dv2 = vB + Cross(wB, cp2.rB) - vA -
             * Cross(wA, cp2.rA);
             *
             * // Compute normal velocity vn2 = Dot(dv2, normal);
             *
             * assert(Abs(vn2 - cp2.velocityBias) < k_errorTol); #endif
             */
            if (DEBUG_SOLVER) {
              // Postconditions
              final Vector2 dv2 = vB + math_utils.crossDblVec2(wB, cp2rB)
                ..sub(vA)
                ..sub(math_utils.crossDblVec2(wA, cp2rA));
              // Compute normal velocity
              vn2 = dv2.dot(normal);

              assert((vn2 - cp2.velocityBias).abs() < k_errorTol);
            }
            break;
          }

          //
          // Case 4: x1 = 0 and x2 = 0
          //
          // vn1 = b1
          // vn2 = ;
          xx = 0.0;
          xy = 0.0;
          vn1 = bx;
          vn2 = by;

          if (vn1 >= 0.0 && vn2 >= 0.0) {
            // Resubstitute for the incremental impulse
            final double dx = xx - ax;
            final double dy = xy - ay;

            // Apply incremental impulse
            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            final double P1x = normalx * dx;
            final double P1y = normaly * dx;
            final double P2x = normalx * dy;
            final double P2y = normaly * dy;

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA *
                (cp1rA.x * P1y -
                    cp1rA.y * P1x +
                    (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB *
                (cp1rB.x * P1y -
                    cp1rB.y * P1x +
                    (cp2rB.x * P2y - cp2rB.y * P2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            break;
          }

          // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
          break;
        }
      }

      // _velocities[indexA].v.set(vA);
      _velocities[indexA].w = wA;
      // _velocities[indexB].v.set(vB);
      _velocities[indexB].w = wB;
    }
  }

  void storeImpulses() {
    for (int i = 0; i < _count; i++) {
      final ContactVelocityConstraint vc = _velocityConstraints[i];
      final Manifold manifold = _contacts[vc.contactIndex]._manifold;

      for (int j = 0; j < vc.pointCount; j++) {
        manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
        manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
      }
    }
  }

  /*
   * #if 0 // Sequential solver. bool ContactSolver::SolvePositionConstraints(double baumgarte) {
   * double minSeparation = 0.0;
   *
   * for (int i = 0; i < _constraintCount; ++i) { ContactConstraint* c = _constraints + i; Body*
   * bodyA = c.bodyA; Body* bodyB = c.bodyB; double invMassA = bodyA._mass * bodyA._invMass; double
   * invIA = bodyA._mass * bodyA._invI; double invMassB = bodyB._mass * bodyB._invMass; double
   * invIB = bodyB._mass * bodyB._invI;
   *
   * Vec2 normal = c.normal;
   *
   * // Solve normal constraints for (int j = 0; j < c.pointCount; ++j) { ContactConstraintPoint*
   * ccp = c.points + j;
   *
   * Vec2 r1 = Mul(bodyA.GetXForm().R, ccp.localAnchorA - bodyA.GetLocalCenter()); Vec2 r2 =
   * Mul(bodyB.GetXForm().R, ccp.localAnchorB - bodyB.GetLocalCenter());
   *
   * Vec2 p1 = bodyA._sweep.c + r1; Vec2 p2 = bodyB._sweep.c + r2; Vec2 dp = p2 - p1;
   *
   * // Approximate the current separation. double separation = Dot(dp, normal) + ccp.separation;
   *
   * // Track max constraint error. minSeparation = Min(minSeparation, separation);
   *
   * // Prevent large corrections and allow slop. double C = Clamp(baumgarte * (separation +
   * _linearSlop), -_maxLinearCorrection, 0.0);
   *
   * // Compute normal impulse double impulse = -ccp.equalizedMass * C;
   *
   * Vec2 P = impulse * normal;
   *
   * bodyA._sweep.c -= invMassA * P; bodyA._sweep.a -= invIA * Cross(r1, P);
   * bodyA.SynchronizeTransform();
   *
   * bodyB._sweep.c += invMassB * P; bodyB._sweep.a += invIB * Cross(r2, P);
   * bodyB.SynchronizeTransform(); } }
   *
   * // We can't expect minSpeparation >= -_linearSlop because we don't // push the separation above
   * -_linearSlop. return minSeparation >= -1.5f * _linearSlop; }
   */

  final PositionSolverManifold _psolver = PositionSolverManifold();

  /**
   * Sequential solver.
   */
  bool solvePositionConstraints() {
    double minSeparation = 0.0;

    for (int i = 0; i < _count; ++i) {
      final ContactPositionConstraint pc = _positionConstraints[i];

      final int indexA = pc.indexA;
      final int indexB = pc.indexB;

      final double mA = pc.invMassA;
      final double iA = pc.invIA;
      final Vector2 localCenterA = pc.localCenterA;
      final double localCenterAx = localCenterA.x;
      final double localCenterAy = localCenterA.y;
      final double mB = pc.invMassB;
      final double iB = pc.invIB;
      final Vector2 localCenterB = pc.localCenterB;
      final double localCenterBx = localCenterB.x;
      final double localCenterBy = localCenterB.y;
      final int pointCount = pc.pointCount;

      final Vector2 cA = _positions[indexA].c;
      double aA = _positions[indexA].a;
      final Vector2 cB = _positions[indexB].c;
      double aB = _positions[indexB].a;

      // Solve normal constraints
      for (int j = 0; j < pointCount; ++j) {
        final Rot xfAq = xfA.q;
        final Rot xfBq = xfB.q;
        xfAq.setAngle(aA);
        xfBq.setAngle(aB);
        xfA.p.x = cA.x - xfAq.c * localCenterAx + xfAq.s * localCenterAy;
        xfA.p.y = cA.y - xfAq.s * localCenterAx - xfAq.c * localCenterAy;
        xfB.p.x = cB.x - xfBq.c * localCenterBx + xfBq.s * localCenterBy;
        xfB.p.y = cB.y - xfBq.s * localCenterBx - xfBq.c * localCenterBy;

        final PositionSolverManifold psm = _psolver;
        psm.initialize(pc, xfA, xfB, j);
        final Vector2 normal = psm.normal;
        final Vector2 point = psm.point;
        final double separation = psm.separation;

        final double rAx = point.x - cA.x;
        final double rAy = point.y - cA.y;
        final double rBx = point.x - cB.x;
        final double rBy = point.y - cB.y;

        // Track max constraint error.
        minSeparation = math.min(minSeparation, separation);

        // Prevent large corrections and allow slop.
        final double C = math_utils.clampDouble(
            settings.baumgarte * (separation + settings.linearSlop),
            -settings.maxLinearCorrection,
            0.0);

        // Compute the effective mass.
        final double rnA = rAx * normal.y - rAy * normal.x;
        final double rnB = rBx * normal.y - rBy * normal.x;
        final double K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        // Compute normal impulse
        final double impulse = K > 0.0 ? -C / K : 0.0;

        final double Px = normal.x * impulse;
        final double Py = normal.y * impulse;

        cA.x -= Px * mA;
        cA.y -= Py * mA;
        aA -= iA * (rAx * Py - rAy * Px);

        cB.x += Px * mB;
        cB.y += Py * mB;
        aB += iB * (rBx * Py - rBy * Px);
      }

      // _positions[indexA].c.set(cA);
      _positions[indexA].a = aA;

      // _positions[indexB].c.set(cB);
      _positions[indexB].a = aB;
    }

    // We can't expect minSpeparation >= -linearSlop because we don't
    // push the separation above -linearSlop.
    return minSeparation >= -3.0 * settings.linearSlop;
  }

  // Sequential position solver for position constraints.
  bool solveTOIPositionConstraints(int toiIndexA, int toiIndexB) {
    double minSeparation = 0.0;

    for (int i = 0; i < _count; ++i) {
      final ContactPositionConstraint pc = _positionConstraints[i];

      final int indexA = pc.indexA;
      final int indexB = pc.indexB;
      final Vector2 localCenterA = pc.localCenterA;
      final Vector2 localCenterB = pc.localCenterB;
      final double localCenterAx = localCenterA.x;
      final double localCenterAy = localCenterA.y;
      final double localCenterBx = localCenterB.x;
      final double localCenterBy = localCenterB.y;
      final int pointCount = pc.pointCount;

      double mA = 0.0;
      double iA = 0.0;
      if (indexA == toiIndexA || indexA == toiIndexB) {
        mA = pc.invMassA;
        iA = pc.invIA;
      }

      double mB = 0.0;
      double iB = 0.0;
      if (indexB == toiIndexA || indexB == toiIndexB) {
        mB = pc.invMassB;
        iB = pc.invIB;
      }

      final Vector2 cA = _positions[indexA].c;
      double aA = _positions[indexA].a;

      final Vector2 cB = _positions[indexB].c;
      double aB = _positions[indexB].a;

      // Solve normal constraints
      for (int j = 0; j < pointCount; ++j) {
        final Rot xfAq = xfA.q;
        final Rot xfBq = xfB.q;
        xfAq.setAngle(aA);
        xfBq.setAngle(aB);
        xfA.p.x = cA.x - xfAq.c * localCenterAx + xfAq.s * localCenterAy;
        xfA.p.y = cA.y - xfAq.s * localCenterAx - xfAq.c * localCenterAy;
        xfB.p.x = cB.x - xfBq.c * localCenterBx + xfBq.s * localCenterBy;
        xfB.p.y = cB.y - xfBq.s * localCenterBx - xfBq.c * localCenterBy;

        final PositionSolverManifold psm = _psolver;
        psm.initialize(pc, xfA, xfB, j);
        final Vector2 normal = psm.normal;

        final Vector2 point = psm.point;
        final double separation = psm.separation;

        final double rAx = point.x - cA.x;
        final double rAy = point.y - cA.y;
        final double rBx = point.x - cB.x;
        final double rBy = point.y - cB.y;

        // Track max constraint error.
        minSeparation = math.min(minSeparation, separation);

        // Prevent large corrections and allow slop.
        final double C = math_utils.clampDouble(
            settings.toiBaugarte * (separation + settings.linearSlop),
            -settings.maxLinearCorrection,
            0.0);

        // Compute the effective mass.
        final double rnA = rAx * normal.y - rAy * normal.x;
        final double rnB = rBx * normal.y - rBy * normal.x;
        final double K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        // Compute normal impulse
        final double impulse = K > 0.0 ? -C / K : 0.0;

        final double Px = normal.x * impulse;
        final double Py = normal.y * impulse;

        cA.x -= Px * mA;
        cA.y -= Py * mA;
        aA -= iA * (rAx * Py - rAy * Px);

        cB.x += Px * mB;
        cB.y += Py * mB;
        aB += iB * (rBx * Py - rBy * Px);
      }

      // _positions[indexA].c.set(cA);
      _positions[indexA].a = aA;

      // _positions[indexB].c.set(cB);
      _positions[indexB].a = aB;
    }

    // We can't expect minSpeparation >= -_linearSlop because we don't
    // push the separation above -_linearSlop.
    return minSeparation >= -1.5 * settings.linearSlop;
  }
}

class PositionSolverManifold {
  final Vector2 normal = Vector2.zero();
  final Vector2 point = Vector2.zero();
  double separation = 0.0;

  void initialize(
      ContactPositionConstraint pc, Transform xfA, Transform xfB, int index) {
    assert(pc.pointCount > 0);

    final Rot xfAq = xfA.q;
    final Rot xfBq = xfB.q;
    final Vector2 pcLocalPointsI = pc.localPoints[index];
    switch (pc.type) {
      case ManifoldType.CIRCLES:
        // Transform.mulToOutUnsafe(xfA, pc.localPoint, pointA);
        // Transform.mulToOutUnsafe(xfB, pc.localPoints[0], pointB);
        // normal.set(pointB).subLocal(pointA);
        // normal.normalize();
        //
        // point.set(pointA).addLocal(pointB).mulLocal(.5f);
        // temp.set(pointB).subLocal(pointA);
        // separation = Vec2.dot(temp, normal) - pc.radiusA - pc.radiusB;
        final Vector2 plocalPoint = pc.localPoint;
        final Vector2 pLocalPoints0 = pc.localPoints[0];
        final double pointAx =
            (xfAq.c * plocalPoint.x - xfAq.s * plocalPoint.y) + xfA.p.x;
        final double pointAy =
            (xfAq.s * plocalPoint.x + xfAq.c * plocalPoint.y) + xfA.p.y;
        final double pointBx =
            (xfBq.c * pLocalPoints0.x - xfBq.s * pLocalPoints0.y) + xfB.p.x;
        final double pointBy =
            (xfBq.s * pLocalPoints0.x + xfBq.c * pLocalPoints0.y) + xfB.p.y;
        normal.x = pointBx - pointAx;
        normal.y = pointBy - pointAy;
        normal.normalize();

        point.x = (pointAx + pointBx) * .5;
        point.y = (pointAy + pointBy) * .5;
        final double tempx = pointBx - pointAx;
        final double tempy = pointBy - pointAy;
        separation =
            tempx * normal.x + tempy * normal.y - pc.radiusA - pc.radiusB;
        break;

      case ManifoldType.FACE_A:
        // Rot.mulToOutUnsafe(xfAq, pc.localNormal, normal);
        // Transform.mulToOutUnsafe(xfA, pc.localPoint, planePoint);
        //
        // Transform.mulToOutUnsafe(xfB, pc.localPoints[index], clipPoint);
        // temp.set(clipPoint).subLocal(planePoint);
        // separation = Vec2.dot(temp, normal) - pc.radiusA - pc.radiusB;
        // point.set(clipPoint);
        final Vector2 pcLocalNormal = pc.localNormal;
        final Vector2 pcLocalPoint = pc.localPoint;
        normal.x = xfAq.c * pcLocalNormal.x - xfAq.s * pcLocalNormal.y;
        normal.y = xfAq.s * pcLocalNormal.x + xfAq.c * pcLocalNormal.y;
        final double planePointx =
            (xfAq.c * pcLocalPoint.x - xfAq.s * pcLocalPoint.y) + xfA.p.x;
        final double planePointy =
            (xfAq.s * pcLocalPoint.x + xfAq.c * pcLocalPoint.y) + xfA.p.y;

        final double clipPointx =
            (xfBq.c * pcLocalPointsI.x - xfBq.s * pcLocalPointsI.y) + xfB.p.x;
        final double clipPointy =
            (xfBq.s * pcLocalPointsI.x + xfBq.c * pcLocalPointsI.y) + xfB.p.y;
        final double tempx = clipPointx - planePointx;
        final double tempy = clipPointy - planePointy;
        separation =
            tempx * normal.x + tempy * normal.y - pc.radiusA - pc.radiusB;
        point.x = clipPointx;
        point.y = clipPointy;
        break;

      case ManifoldType.FACE_B:
        // Rot.mulToOutUnsafe(xfBq, pc.localNormal, normal);
        // Transform.mulToOutUnsafe(xfB, pc.localPoint, planePoint);
        //
        // Transform.mulToOutUnsafe(xfA, pcLocalPointsI, clipPoint);
        // temp.set(clipPoint).subLocal(planePoint);
        // separation = Vec2.dot(temp, normal) - pc.radiusA - pc.radiusB;
        // point.set(clipPoint);
        //
        // // Ensure normal points from A to B
        // normal.negateLocal();
        final Vector2 pcLocalNormal = pc.localNormal;
        final Vector2 pcLocalPoint = pc.localPoint;
        normal.x = xfBq.c * pcLocalNormal.x - xfBq.s * pcLocalNormal.y;
        normal.y = xfBq.s * pcLocalNormal.x + xfBq.c * pcLocalNormal.y;
        final double planePointx =
            (xfBq.c * pcLocalPoint.x - xfBq.s * pcLocalPoint.y) + xfB.p.x;
        final double planePointy =
            (xfBq.s * pcLocalPoint.x + xfBq.c * pcLocalPoint.y) + xfB.p.y;

        final double clipPointx =
            (xfAq.c * pcLocalPointsI.x - xfAq.s * pcLocalPointsI.y) + xfA.p.x;
        final double clipPointy =
            (xfAq.s * pcLocalPointsI.x + xfAq.c * pcLocalPointsI.y) + xfA.p.y;
        final double tempx = clipPointx - planePointx;
        final double tempy = clipPointy - planePointy;
        separation =
            tempx * normal.x + tempy * normal.y - pc.radiusA - pc.radiusB;
        point.x = clipPointx;
        point.y = clipPointy;
        normal.x *= -1;
        normal.y *= -1;
        break;
    }
  }
}
