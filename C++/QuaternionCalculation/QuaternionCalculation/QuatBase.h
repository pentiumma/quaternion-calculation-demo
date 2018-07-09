#pragma once

#define M_PI     3.1415926f
#define D2R(x)  x * M_PI / 180.0f
#define R2D(x)  x * 180.0f / M_PI

//! BVH rotate orders
typedef enum _RotateOrders
{
  RO_XZY, //!< 顺序依次为X轴、 Z轴、 Y轴的旋转值
  RO_YXZ, //!< 顺序依次为Y轴、 X轴、 Z轴的旋转值
  RO_XYZ, //!< 顺序依次为X轴、 Y轴、 Z轴的旋转值
  RO_YZX, //!< 顺序依次为Y轴、 Z轴、 X轴的旋转值
  RO_ZXY, //!< 顺序依次为Z轴、 X轴、 Y轴的旋转值
  RO_ZYX, //!< 顺序依次为Z轴、 Y轴、 X轴的旋转值
  RO_Unknown, //!< Unknown type
}RotateOrders;

class QuatBase
{
public:

  // 四元数转欧拉角
  static inline Vector3f QuatToEulerAngle(RotateOrders rotateOrder, const Quaternionf& quat)
  {
    double w = quat.w();
    double x(0), y(0), z(0);
    int Flag = 1;

    if (rotateOrder == RO_XZY)
    { //t'系
      x = quat.y();
      y = quat.z();
      z = quat.x();
    }
    else if (rotateOrder == RO_YXZ)
    {
      x = quat.z();
      y = quat.x();
      z = quat.y();
    }
    else if (rotateOrder == RO_XYZ)
    {
      x = -quat.z();
      y = quat.y();
      z = quat.x();
      Flag = -1;
    }
    else if (rotateOrder == RO_YZX)
    {
      x = -quat.x();
      y = quat.z();
      z = quat.y();
      Flag = -1;
    }
    else if (rotateOrder == RO_ZXY)
    {
      x = -quat.y();
      y = quat.x();
      z = quat.z();
      Flag = -1;
    }
    else if (rotateOrder == RO_ZYX)
    {
      x = quat.x();
      y = quat.y();
      z = quat.z();
    }
    else {}

    double az = 2 * (w*y - z*x);

    // Gimbal lock
    if (fabs(az) > 0.9999999)
    {
      double res1 = M_PI * 0.5 * az;
      double res0 = atan2(-y*x + w*z, 0.5 - z*z - x*x);
      double res2 = 0.0;
      Vector3f angle(res0, res1, res2);
      return R2D(angle);
    }

    double axy = 2 * (w*z + x*y);
    double axx = 1 - 2 * (y*y + z*z);

    double ayy = 2 * (w*x + y*z) * Flag;
    double ayx = 1 - 2 * (x*x + y*y);

    double res0 = atan2(axy, axx);
    double res2 = atan2(ayy, ayx);
    double res1 = asin(az);

    Vector3f angle(res0, res1, res2);
    angle = R2D(angle);
    return angle;
  }

  static inline Quaternionf EulerToQuat(RotateOrders rotatorOrder, const Vector3f& angle)
  {
    Vector3f a1;
    Vector3f a2;
    Vector3f a3;
    switch (rotatorOrder)
    {
    case RO_XZY:
      a1 = Vector3f::UnitX();
      a2 = Vector3f::UnitZ();
      a3 = Vector3f::UnitY();
      break;
    case RO_YXZ:
      a1 = Vector3f::UnitY();
      a2 = Vector3f::UnitX();
      a3 = Vector3f::UnitZ();
      break;
    case RO_XYZ:
      a1 = Vector3f::UnitX();
      a2 = Vector3f::UnitY();
      a3 = Vector3f::UnitZ();
      break;
    case RO_YZX:
      a1 = Vector3f::UnitY();
      a2 = Vector3f::UnitZ();
      a3 = Vector3f::UnitX();
      break;
    case RO_ZXY:
      a1 = Vector3f::UnitZ();
      a2 = Vector3f::UnitX();
      a3 = Vector3f::UnitY();
      break;
    case RO_ZYX:
      a1 = Vector3f::UnitZ();
      a2 = Vector3f::UnitY();
      a3 = Vector3f::UnitX();
      break;
    case RO_Unknown:
      break;
    default:
      break;
    }

    return AngleAxisf(D2R(angle.x()), a1) *
      AngleAxisf(D2R(angle.y()), a2) *
      AngleAxisf(D2R(angle.z()), a3);
  }

};