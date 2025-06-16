std::pair<int, double> LQRPID::find_nearest_point(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cyaw,
    double x, double y)
{
    int nearest_ind = 0;
    double min_d2 = std::numeric_limits<double>::infinity();

    // 1. 가장 가까운 인덱스 찾기
    for (std::size_t i = 0; i < cx.size(); ++i) {
        double dx = cx[i] - x;
        double dy = cy[i] - y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_d2) {
            min_d2 = d2;
            nearest_ind = i;
        }
    }

    // 2. lateral error 계산 (최근접 기준)
    double e = std::sqrt(min_d2);
    double angle = pi2pi(cyaw[nearest_ind] - std::atan2(cy[nearest_ind] - y, cx[nearest_ind] - x));
    if (angle < 0.0) e = -e;

    // 3. preview horizon 적용
    int preview_horizon = 15;  // 실험적으로 조정: 10 ~ 20
    int target_ind = std::min(nearest_ind + preview_horizon, static_cast<int>(cx.size() - 1));

    return {target_ind, e};  // target point index와 최근접 기준 lateral error 반환
}



auto [ind, e] = find_nearest_point(
    waypoints.x_m, waypoints.y_m, waypoints.psi_rad, x, y);

double psi = waypoints.psi_rad[ind];
double k = waypoints.kappa_radpm[ind];
double th_e = pi2pi(yaw - psi);
