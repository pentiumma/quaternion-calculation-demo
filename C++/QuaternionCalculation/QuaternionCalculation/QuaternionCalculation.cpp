#include "stdafx.h"
#include "QuatBase.h"

void PrintVec(const Vector3f& v)
{
  printf("%f, %f, %f\n", v.x(), v.y(), v.z());
}

void PrintQuat(const Quaternionf& q)
{
  printf("%f, %f, %f, %f\n", q.w(), q.x(), q.y(), q.z());
}

void PrintMat(const Matrix3f& M)
{
  printf("\n%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n", M.row(0)[0], M.row(0)[1], M.row(0)[2],
         M.row(1)[0], M.row(1)[1], M.row(1)[2], M.row(2)[0], M.row(2)[1], M.row(2)[2]);
}

int main()
{
  //////////////////////////////////////////////////////////////////////////
  // Quaternion calculation notes
  //////////////////////////////////////////////////////////////////////////

  // Define three coordinate systems (CS):
  // A: x - East, y - North, z - Up
  // B: x - West, y - South, z - Up   (A rotate around its z axis with 180 degree)
  // C: x - Down, y - North, z - East (A rotate around its y axis with 90  degree)

  // 1. Quaternion construction
  // Rotation from A to B:
  Vector3f r1_A(0.0f, 0.0f, 1.0f);  // rotation axis: z axis of A
  float theta1 = D2R(180.0f);  // rotation angle (following right-hand rule)
  // Rotation from A to C:
  Vector3f r2_A(0.0f, 1.0f, 0.0f);  // rotation axis: y axis of A
  float theta2 = D2R(90.0f);   // rotation angle (following right-hand rule)

  Quaternionf q_A_B(cos(theta1 / 2.0f), -sin(theta1 / 2.0f)*r1_A.x(), -sin(theta1 / 2.0f)*r1_A.y(), -sin(theta1 / 2.0f)*r1_A.z());  // denote quaternion from B to A, or A w.r.t. B
  Quaternionf q_A_C(cos(theta2 / 2.0f), -sin(theta2 / 2.0f)*r2_A.x(), -sin(theta2 / 2.0f)*r2_A.y(), -sin(theta2 / 2.0f)*r2_A.z());  // denote quaternion from C to A, or A w.r.t. C
  printf("q_A_B: ");
  PrintQuat(q_A_B);
  printf("q_A_C: ");
  PrintQuat(q_A_C);

  // 2. Vector coordinate transformation
  Vector3f xA_A(1.0f, 0.0f, 0.0f);  // A's x axis in A
  Vector3f xA_B = q_A_B * xA_A;     // A's x axis in B
  Vector3f xA_C = q_A_C * xA_A;     // A's x axis in C
  printf("xA_B: ");
  PrintVec(xA_B);
  printf("xA_C: ");
  PrintVec(xA_C);

  Quaternionf q_B_C = q_A_C * q_A_B.inverse();
  Vector3f xB_B(1.0f, 0.0f, 0.0f);  // B's x axis in B
  Vector3f xB_C = q_B_C * xB_B;     // B's x axis in C
  printf("q_B_C: ");
  PrintQuat(q_B_C);
  printf("xB_C: ");
  PrintVec(xB_C);

  // 3. Quaternion and rotational matrix conversion
  Matrix3f M_A_B_1 = q_A_B.toRotationMatrix();
  Matrix3f M_A_B_2;
  Vector3f yA_B(0.0f, -1.0f, 0.0f);
  Vector3f zA_B(0.0f, 0.0f, 1.0f);
  M_A_B_2.col(0) = xA_B;
  M_A_B_2.col(1) = yA_B;
  M_A_B_2.col(2) = zA_B;
  Matrix3f M_A_B = M_A_B_1;
  printf("M_A_B: ");
  PrintMat(M_A_B_1);
  PrintMat(M_A_B_2);

  Matrix3f M_A_C_1 = q_A_C.toRotationMatrix();
  Matrix3f M_A_C_2;
  Vector3f yA_C(0.0f, 1.0f, 0.0f);
  Vector3f zA_C(-1.0f, 0.0f, 0.0f);
  M_A_C_2.col(0) = xA_C;
  M_A_C_2.col(1) = yA_C;
  M_A_C_2.col(2) = zA_C;
  Matrix3f M_A_C = M_A_C_1;
  printf("M_A_C: ");
  PrintMat(M_A_C_1);
  PrintMat(M_A_C_2);

  Matrix3f M_B_C_1 = M_A_C * M_A_B.conjugate();
  Matrix3f M_B_C_2;
  Vector3f yB_C(0.0f, -1.0f, 0.0f);
  Vector3f zB_C(-1.0f, 0.0f, 0.0f);
  M_B_C_2.col(0) = xB_C;
  M_B_C_2.col(1) = yB_C;
  M_B_C_2.col(2) = zB_C;
  Matrix3f M_B_C = M_B_C_1;
  printf("M_B_C: ");
  PrintMat(M_B_C_1);
  PrintMat(M_B_C_2);

  // 4. Quaternion/Rotational matrix and Euler angle conversion
  // B rotate gamma, beta, alpha in 'ZYX' order to get C
  float alpha = D2R(0.0f);   // rotation angle around x axis
  float beta = D2R(90.0f);   // rotation angle around y axis
  float gamma = D2R(180.0f); // rotation angle around z axis
  Matrix3f M_B_C_3;
  M_B_C_3 = AngleAxisf(gamma, Vector3f::UnitZ())
    * AngleAxisf(beta, Vector3f::UnitY())
    * AngleAxisf(alpha, Vector3f::UnitX());
  printf("M_B_C: ");
  PrintMat(M_B_C);
  PrintMat(M_B_C_3);
  Vector3f euler_angles = M_B_C_3.eulerAngles(2, 1, 0);
  printf("euler angles in ZYX order: \n");
  printf("%f, %f, %f\n", R2D(gamma), R2D(beta), R2D(alpha));
  PrintVec(R2D(euler_angles));
  // Use cutomized quat-euler angle conversion
  Vector3f euler_angles_2(R2D(gamma), R2D(beta), R2D(alpha));
  Quaternionf q_B_C_2 = QuatBase::EulerToQuat(RO_ZYX, euler_angles_2);
  printf("q_B_C: \n");
  PrintQuat(q_B_C_2);
  euler_angles_2 = QuatBase::QuatToEulerAngle(RO_ZYX, q_B_C);
  printf("Euler angles: \n");
  PrintVec(euler_angles_2);

  getchar();
  return 0;
}

