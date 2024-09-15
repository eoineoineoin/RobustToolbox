using System;
using System.Numerics;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Systems;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Utility;
using Robust.Shared.ViewVariables;

namespace Robust.Shared.Physics.Collision.Shapes
{
    /// <summary>
    /// A physics shape that represents a circle. The circle cannot be rotated,
    /// and it's origin is always the same as the entity position.
    /// </summary>
    [Serializable, NetSerializable]
    [DataDefinition]
    public sealed partial class PhysShapeCircle : IPhysShape, IEquatable<PhysShapeCircle>
    {
        public int ChildCount => 1;

        public ShapeType ShapeType => ShapeType.Circle;

        private const float DefaultRadius = 0.5f;

        [DataField("radius"), Access(typeof(SharedPhysicsSystem), Friend = AccessPermissions.ReadWriteExecute, Other = AccessPermissions.Read)]
        public float Radius { get; set; } = DefaultRadius;

        [DataField("position"), Access(typeof(SharedPhysicsSystem), Friend = AccessPermissions.ReadWriteExecute, Other = AccessPermissions.Read)]
        public Vector2 Position;

        public PhysShapeCircle()
        {
        }

        public PhysShapeCircle(float radius)
        {
            Radius = radius;
            Position = Vector2.Zero;
        }

        public PhysShapeCircle(float radius, Vector2 position)
        {
            Radius = radius;
            Position = position;
        }

        public float CalculateArea()
        {
            return MathF.PI * Radius * Radius;
        }

        public Box2 ComputeAABB(Transform transform, int childIndex)
        {
            DebugTools.Assert(childIndex == 0);

            var p = transform.Position + Transform.Mul(transform.Quaternion2D, Position);
            return new Box2(p.X - Radius, p.Y - Radius, p.X + Radius, p.Y + Radius);
        }

        public bool CastRay(ref Vector2 rayOrigin, ref Vector2 rayDir, float rayLength, out float fraction)
        {
            fraction = 0.0f;
            if ((Position - rayOrigin).LengthSquared() < Radius * Radius)
            {
                // Starts inside the circle
                return true;
            }

            // Project the center of the circle onto the ray
            var proj = Vector2.Dot(rayDir, Position - rayOrigin);
            if (proj < 0.0f)
            {
                // Ray points away from the circle
                return false;
            }

            // This is the closest point between the circle center and the infinite ray:
            var closestPoint = rayOrigin + rayDir * proj;
            // Solve for the point on the circle
            var intersectTSq = Radius * Radius - (closestPoint - Position).LengthSquared();
            if (intersectTSq <= 0.0f)
            {
                // No real roots -> no intersection
                return false;
            }

            // Have a hit at closestPoint - direction * sqrt(intersectTSq). Calculate the fraction:
            //<todo: redundant calculations here:
            //var intersectionPoint = position + direction * (proj - float.Sqrt(intersectTSq));
            //fraction = (intersectionPoint - position).Length() / length;
            fraction = (proj - float.Sqrt(intersectTSq)) / rayLength;
            return fraction <= 1.0f;
        }

        public Box2 CalcLocalBounds()
        {
            // circle inscribed in box
            return new Box2(
                Position.X - Radius,
                Position.Y - Radius,
                Position.X + Radius,
                Position.Y + Radius);
        }

        public bool Equals(IPhysShape? other)
        {
            if (other is not PhysShapeCircle otherCircle) return false;
            return otherCircle.Equals(this);
        }

        public bool Equals(PhysShapeCircle? other)
        {
            if (ReferenceEquals(null, other))
                return false;
            if (ReferenceEquals(this, other))
                return true;

            return MathHelper.CloseTo(Radius, other.Radius) && Position.EqualsApprox(other.Position);
        }

        public override bool Equals(object? obj)
        {
            return ReferenceEquals(this, obj) || obj is PhysShapeCircle other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Radius, Position);
        }
    }
}
