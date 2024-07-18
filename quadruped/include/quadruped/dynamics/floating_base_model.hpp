/*! @file FloatingBaseModel.h
 *  @brief 浮动基rigid体模型数据结构的实现
 *
 * 此类存储了 Featherstone 的 "Rigid Body Dynamics Algorithms" 中描述的kinematic树
 * (可从 https://www.springer.com/us/book/9780387743141 下载，需要MIT互联网)
 *
 * 树中包括每个身体的额外“rotor”身体，该rotor固定在父身体上，具有齿轮约束
 * 使用 Jain 的 "Robot and Multibody Dynamics" 第12章中描述的技术来高效地包含它
 * 注意，这个实现仅适用于单个旋转rotor per rigid body的情况。
 * Rotors具有与身体相同的关节类型，但应用了齿轮比率乘数到motion subspace中 浮动基体关联的rotors不执行任何操作
 */

#ifndef LIBBIOMIMETICS_FLOATINGBASEMODEL_H
#define LIBBIOMIMETICS_FLOATINGBASEMODEL_H

#include <eigen3/Eigen/StdVector>

#include "quadruped/dynamics/spatial.hpp"
using namespace spatial;

/*!
 * 浮动基模型的状态（基体和关节）
 */
template <typename T>
struct FBModelState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quat<T> bodyOrientation;
  Vec3<T> bodyPosition;
  SVec<T> bodyVelocity;  // 基体坐标
  DVec<T> q;
  DVec<T> qd;

  /*!
   * 打印基体的位置
   */
  void print() const { printf("position: %.3f %.3f %.3f\n", bodyPosition[0], bodyPosition[1], bodyPosition[2]); }
};

/*!
 * 在刚体浮动基模型上运行articulated body算法的结果
 */
template <typename T>
struct FBModelStateDerivative {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3<T> dBodyPosition;
  SVec<T> dBodyVelocity;
  DVec<T> qdd;
};

/*!
 * 代表浮动基刚体模型的类，包括rotors和地面接触点，无状态概念
 */
template <typename T>
class FloatingBaseModel {
public:
  /*!
   * 使用默认重力初始化浮动基模型
   */
  FloatingBaseModel() : _gravity(0, 0, -9.81) {}
  ~FloatingBaseModel() {}

  void addBase(const SpatialInertia<T>& inertia);
  void addBase(T mass, const Vec3<T>& com, const Mat3<T>& I);
  int addGroundContactPoint(int bodyID, const Vec3<T>& location, bool isFoot = false);
  void addGroundContactBoxPoints(int bodyId, const Vec3<T>& dims);
  int addBody(
      const SpatialInertia<T>& inertia,
      const SpatialInertia<T>& rotorInertia,
      T gearRatio,
      int parent,
      JointType jointType,
      CoordinateAxis jointAxis,
      const Mat6<T>& Xtree,
      const Mat6<T>& Xrot);
  int addBody(
      const MassProperties<T>& inertia,
      const MassProperties<T>& rotorInertia,
      T gearRatio,
      int parent,
      JointType jointType,
      CoordinateAxis jointAxis,
      const Mat6<T>& Xtree,
      const Mat6<T>& Xrot);
  void check();
  T totalRotorMass();
  T totalNonRotorMass();

  /*!
   * 获取父体向量，其中parents[i]是身体i的父体
   * @return 父体向量
   */
  const std::vector<int>& getParentVector() { return _parents; }

  /*!
   * 获取身体空间惯量向量
   * @return 身体空间惯量向量
   */
  const std::vector<SpatialInertia<T>, Eigen::aligned_allocator<SpatialInertia<T>>>& getBodyInertiaVector() {
    return _Ibody;
  }

  /*!
   * 获取rotor空间惯量向量
   * @return rotor空间惯量向量
   */
  const std::vector<SpatialInertia<T>, Eigen::aligned_allocator<SpatialInertia<T>>>& getRotorInertiaVector() {
    return _Irot;
  }

  /*!
   * 设置重力
   */
  void setGravity(Vec3<T>& g) { _gravity = g; }

  /*!
   * 设置contact点的计算标志
   * @param gc_index : contact点的索引
   * @param flag : 是否启用contact计算
   */
  void setContactComputeFlag(size_t gc_index, bool flag) { _compute_contact_info[gc_index] = flag; }

  DMat<T> invContactInertia(const int gc_index, const D6Mat<T>& force_directions);
  T invContactInertia(const int gc_index, const Vec3<T>& force_ics_at_contact);

  T applyTestForce(const int gc_index, const Vec3<T>& force_ics_at_contact, FBModelStateDerivative<T>& dstate_out);

  T applyTestForce(const int gc_index, const Vec3<T>& force_ics_at_contact, DVec<T>& dstate_out);

  void addDynamicsVars(int count);

  void resizeSystemMatricies();

  /*!
   * 更新模拟器的状态，废弃以前的结果
   * @param state : 新的状态
   */
  void setState(const FBModelState<T>& state) {
    _state = state;

    _biasAccelerationsUpToDate = false;
    _compositeInertiasUpToDate = false;

    resetCalculationFlags();
  }

  /*!
   * 将所有以前计算的值标记为无效
   */
  void resetCalculationFlags() {
    _articulatedBodiesUpToDate = false;
    _kinematicsUpToDate = false;
    _forcePropagatorsUpToDate = false;
    _qddEffectsUpToDate = false;
    _accelerationsUpToDate = false;
  }

  /*!
   * 更新模拟器的状态导数，废弃以前的结果。
   * @param dState : 新的状态导数
   */
  void setDState(const FBModelStateDerivative<T>& dState) {
    _dState = dState;
    _accelerationsUpToDate = false;
  }

  Vec3<T> getPosition(const int link_idx, const Vec3<T>& local_pos);
  Vec3<T> getPosition(const int link_idx);

  Mat3<T> getOrientation(const int link_idx);
  Vec3<T> getLinearVelocity(const int link_idx, const Vec3<T>& point);
  Vec3<T> getLinearVelocity(const int link_idx);

  Vec3<T> getLinearAcceleration(const int link_idx, const Vec3<T>& point);
  Vec3<T> getLinearAcceleration(const int link_idx);

  Vec3<T> getAngularVelocity(const int link_idx);
  Vec3<T> getAngularAcceleration(const int link_idx);

  void forwardKinematics();
  void biasAccelerations();
  void compositeInertias();
  void forwardAccelerationKinematics();
  void contactJacobians();

  DVec<T> generalizedGravityForce();
  DVec<T> generalizedCoriolisForce();
  DMat<T> massMatrix();
  DVec<T> inverseDynamics(const FBModelStateDerivative<T>& dState);
  void runABA(const DVec<T>& tau, FBModelStateDerivative<T>& dstate);

  size_t _nDof = 0;
  Vec3<T> _gravity;
  std::vector<int> _parents;
  std::vector<T> _gearRatios;
  std::vector<T> _d, _u;

  std::vector<JointType> _jointTypes;
  std::vector<CoordinateAxis> _jointAxes;
  std::vector<Mat6<T>, Eigen::aligned_allocator<Mat6<T>>> _Xtree, _Xrot;
  std::vector<SpatialInertia<T>, Eigen::aligned_allocator<SpatialInertia<T>>> _Ibody, _Irot;
  std::vector<std::string> _bodyNames;

  size_t _nGroundContact = 0;
  std::vector<size_t> _gcParent;
  std::vector<Vec3<T>> _gcLocation;
  std::vector<uint64_t> _footIndicesGC;

  std::vector<Vec3<T>> _pGC;
  std::vector<Vec3<T>> _vGC;

  std::vector<bool> _compute_contact_info;

  /*!
   * 获取系统的质量矩阵
   */
  const DMat<T>& getMassMatrix() const { return _H; }

  /*!
   * 获取重力项（广义力）
   */
  const DVec<T>& getGravityForce() const { return _G; }

  /*!
   * 获取科里奥利斯项（广义力）
   */
  const DVec<T>& getCoriolisForce() const { return _Cqd; }

  /// BEGIN ALGORITHM SUPPORT VARIABLES
  FBModelState<T> _state;
  FBModelStateDerivative<T> _dState;

  vectorAligned<SVec<T>> _v, _vrot, _a, _arot, _avp, _avprot, _c, _crot, _S, _Srot, _fvp, _fvprot, _ag, _agrot, _f,
      _frot;

  vectorAligned<SVec<T>> _U, _Urot, _Utot, _pA, _pArot;
  vectorAligned<SVec<T>> _externalForces;

  vectorAligned<SpatialInertia<T>> _IC;
  vectorAligned<Mat6<T>> _Xup, _Xa, _Xuprot, _IA, _ChiUp;

  DMat<T> _H, _C;
  DVec<T> _Cqd, _G;

  vectorAligned<D6Mat<T>> _J;
  vectorAligned<SVec<T>> _Jdqd;

  vectorAligned<D3Mat<T>> _Jc;
  vectorAligned<Vec3<T>> _Jcdqd;

  bool _kinematicsUpToDate = false;
  bool _biasAccelerationsUpToDate = false;
  bool _accelerationsUpToDate = false;

  bool _compositeInertiasUpToDate = false;

  void updateArticulatedBodies();
  void updateForcePropagators();
  void udpateQddEffects();

  /*!
   * 将所有外部力设为零
   */
  void resetExternalForces() {
    for (size_t i = 0; i < _nDof; i++) {
      _externalForces[i] = SVec<T>::Zero();
    }
  }

  bool _articulatedBodiesUpToDate = false;
  bool _forcePropagatorsUpToDate = false;
  bool _qddEffectsUpToDate = false;

  DMat<T> _qdd_from_base_accel;
  DMat<T> _qdd_from_subqdd;
  Eigen::ColPivHouseholderQR<Mat6<T>> _invIA5;
};

// #include "dynamics/floating_base_model.hxx"
#endif  // LIBBIOMIMETICS_FLOATINGBASEMODEL_H
