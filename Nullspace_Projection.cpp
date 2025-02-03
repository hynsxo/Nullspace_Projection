#include <iostream>
#include "../DrumRobot/include/eigen-3.4.0/Eigen/Dense"

// Jacobian의 유사 역행렬을 계산하는 함수
Eigen::MatrixXd computePseudoInverse(const Eigen::MatrixXd& J) {
    return J.completeOrthogonalDecomposition().pseudoInverse();
}

// Null-space Projection을 활용한 충돌 회피 속도 계산 함수
Eigen::VectorXd computeJointVelocities(
    const Eigen::MatrixXd& J, 
    const Eigen::VectorXd& q_prev, 
    const Eigen::VectorXd& q_next, 
    const Eigen::Vector3d& pR, 
    const Eigen::Vector3d& pL, 
    double dt) 
{
    Eigen::VectorXd dq(9);  // 9개 관절 속도 벡터

    // 1. 목표 동작 수행을 위한 기본 속도 계산
    Eigen::VectorXd dq_primary = (q_next - q_prev) / dt;

    // 2. 충돌 거리 계산
    double d = (pR - pL).norm();
    double d_min = 0.15;  // 충돌 거리 임계값

    // 3. 충돌 회피 동작
    Eigen::VectorXd dq_avoid(9);
    dq_avoid.setZero();

    if (d < d_min) {
        // 회피 방향 계산 (반대 방향으로 힘 적용)
        Eigen::Vector3d v_avoid = (pR - pL).normalized();
        Eigen::VectorXd dx_avoid(6);
        dx_avoid << v_avoid * (d_min - d), Eigen::Vector3d::Zero(); // x,y,z만 이동
        
        // Jacobian의 유사 역행렬
        Eigen::MatrixXd J_pinv = computePseudoInverse(J);

        // Null-space Projection 행렬
        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(9, 9) - J_pinv * J;
        
        // Null-space에서 충돌 회피 속도 적용
        dq_avoid = N * J.transpose() * dx_avoid;
    }

    // 4. 최종 관절 속도 = 기본 동작 + 충돌 회피 동작
    dq = dq_primary + dq_avoid;
    return dq;
}

int main() {
    // 예제 입력 데이터
    Eigen::MatrixXd J(6, 9);  // Jacobian 행렬
    J.setRandom();  // 랜덤한 Jacobian 값 (실제 로봇 Jacobian 사용 필요)

    Eigen::VectorXd q_prev(9);  // 이전 관절 각도
    Eigen::VectorXd q_next(9);  // 다음 목표 관절 각도
    q_prev.setRandom();
    q_next.setRandom();

    Eigen::Vector3d pR(0.5, 0.2, 0.3); // 오른손 위치
    Eigen::Vector3d pL(0.5, -0.2, 0.3); // 왼손 위치

    double dt = 0.01; // 시간 간격 (10ms)

    Eigen::VectorXd dq = computeJointVelocities(J, q_prev, q_next, pR, pL, dt);
    
    std::cout << "Calculated joint velocities: \n" << dq << std::endl;

    return 0;
}
