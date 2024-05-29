#ifndef QR_PHYSICS_TRANSFORM_H
#define QR_PHYSICS_TRANSFORM_H

#include "utils/qr_cpptypes.h"
#include "utils/qr_se3.h"

namespace Quadruped {

/**
 * @brief 转换刚体旋转惯量。
 * @param惯量：惯量在惯性体坐标系中的表示，即COM的原点。
 * @param mass：身体质量。
 * @param p：相对于COM的参考点，惯量是相对于该点表示的。
 * @param R：新坐标系相对于身体坐标系的方向，惯量是相对于该坐标系表示的。
 * @return 新的惯量矩阵。
 */
Mat3<float> transformInertia(Mat3<float> inertia, float mass, Vec3<float> p, Mat3<float> R = Mat3<float>::Identity());

}  // Namespace Quadruped

#endif  // QR_PHYSICS_TRANSFORM_H
