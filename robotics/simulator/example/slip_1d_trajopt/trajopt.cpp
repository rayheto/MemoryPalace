#include "trajopt.hpp"

#include <nlopt.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace slip_trajopt {

namespace {

constexpr double kG = 9.81;

// 决策向量布局（n = 3N+3）：
//   X(k)  = k                   x_k        k=0..N
//   DX(k) = (N+1) + k           dx_k       k=0..N
//   U(k)  = 2(N+1) + k          u_k        k=0..N-1
//   IT    = 2(N+1) + N = 3N+2   T
struct Layout {
    int N;
    int X(int k) const  { return k; }
    int DX(int k) const { return (N + 1) + k; }
    int U(int k) const  { return 2 * (N + 1) + k; }
    int IT() const      { return 3 * N + 2; }
    int n_vars() const  { return 3 * N + 3; }
    // 等式约束布局：
    //   c[0]=x_0-l, c[1]=dx_0-dx_TD, c[2]=x_N-l, c[3]=dx_N-dx_LO_target,
    //   接着 (defect_x_k, defect_v_k) 对 k=0..N-1
    int n_eq() const    { return 4 + 2 * N; }
};

struct UserData {
    Layout      L;
    Params      P;
    double      dx_TD;
    double      dx_LO_target;
};

// f(z,u) 的分量（支撑相，线性动力学）：
static inline double stance_a(double x, double dx, double u, const Params& p) {
    return -kG - (p.kp / p.m) * (x - p.l) - p.kd * dx + u;
}

// ── 目标函数 ────────────────────────────────────────────────────────
// J = h · Σ u_k²         h = T/N         （只罚控制力消耗，按需饱和）
static double objective(const std::vector<double>& xv,
                        std::vector<double>& grad,
                        void* data) {
    auto* D = static_cast<UserData*>(data);
    const Layout& L = D->L;
    const int N = L.N;
    const double T = xv[L.IT()];
    const double h = T / (double)N;

    double sum_u2 = 0.0;
    for (int k = 0; k < N; ++k) {
        double uk = xv[L.U(k)];
        sum_u2 += uk * uk;
    }

    const double J = h * sum_u2;

    if (!grad.empty()) {
        std::fill(grad.begin(), grad.end(), 0.0);
        for (int k = 0; k < N; ++k) {
            grad[L.U(k)] = h * 2.0 * xv[L.U(k)];
        }
        grad[L.IT()] = sum_u2 / (double)N;
    }
    return J;
}

// ── 等式约束（含 Jacobian）─────────────────────────────────────────
// Jacobian 行主序：grad[i * n_vars + j] = ∂c_i / ∂x_j
static void equality_constraints(unsigned m, double* result,
                                 unsigned n_vars, const double* xv,
                                 double* grad, void* data) {
    auto* D = static_cast<UserData*>(data);
    const Layout& L = D->L;
    const int N = L.N;
    const double T = xv[L.IT()];
    const double h = T / (double)N;
    const double alpha = D->P.kp / D->P.m;
    const double kd    = D->P.kd;

    // 清零 Jacobian（稀疏，多数项为 0）
    if (grad) std::memset(grad, 0, sizeof(double) * (size_t)m * n_vars);

    auto J = [&](int i, int j) -> double& {
        return grad[(size_t)i * n_vars + (size_t)j];
    };

    // c0: x_0 - l = 0
    result[0] = xv[L.X(0)] - D->P.l;
    if (grad) J(0, L.X(0)) = 1.0;

    // c1: dx_0 - dx_TD = 0
    result[1] = xv[L.DX(0)] - D->dx_TD;
    if (grad) J(1, L.DX(0)) = 1.0;

    // c2: x_N - l = 0
    result[2] = xv[L.X(N)] - D->P.l;
    if (grad) J(2, L.X(N)) = 1.0;

    // c3: dx_N - dx_LO_target = 0   （指定离地速度 → 指定顶点高度）
    result[3] = xv[L.DX(N)] - D->dx_LO_target;
    if (grad) J(3, L.DX(N)) = 1.0;

    // 配点缺陷 (k=0..N-1)
    for (int k = 0; k < N; ++k) {
        const double xk   = xv[L.X(k)];
        const double xk1  = xv[L.X(k + 1)];
        const double dxk  = xv[L.DX(k)];
        const double dxk1 = xv[L.DX(k + 1)];
        const double uk   = xv[L.U(k)];

        const double ak   = stance_a(xk,  dxk,  uk, D->P);
        const double ak1  = stance_a(xk1, dxk1, uk, D->P);

        const int ix = 4 + 2 * k;       // x defect row
        const int iv = 4 + 2 * k + 1;   // v defect row

        // 位移缺陷: defect_x = x_{k+1} - x_k - (h/2)(dx_k + dx_{k+1})
        result[ix] = xk1 - xk - 0.5 * h * (dxk + dxk1);
        if (grad) {
            J(ix, L.X(k))     += -1.0;
            J(ix, L.X(k + 1)) += +1.0;
            J(ix, L.DX(k))    += -0.5 * h;
            J(ix, L.DX(k + 1))+= -0.5 * h;
            J(ix, L.IT())     += -0.5 * (1.0 / (double)N) * (dxk + dxk1);
        }

        // 速度缺陷: defect_v = dx_{k+1} - dx_k - (h/2)(a_k + a_{k+1})
        result[iv] = dxk1 - dxk - 0.5 * h * (ak + ak1);
        if (grad) {
            // ∂a_k/∂x_k = -alpha, ∂a_k/∂dx_k = -kd, ∂a_k/∂u_k = 1
            // 同理 a_{k+1}
            J(iv, L.X(k))     += -0.5 * h * (-alpha);         // = +0.5*h*alpha
            J(iv, L.X(k + 1)) += -0.5 * h * (-alpha);
            J(iv, L.DX(k))    += -1.0 - 0.5 * h * (-kd);      // = -1 + 0.5*h*kd
            J(iv, L.DX(k + 1))+= +1.0 - 0.5 * h * (-kd);      // = +1 + 0.5*h*kd
            J(iv, L.U(k))     += -0.5 * h * (1.0 + 1.0);      // = -h
            J(iv, L.IT())     += -0.5 * (1.0 / (double)N) * (ak + ak1);
        }
    }
    (void)m;
}

// ── 暖启：sin-dip + 线性插值速度 ───────────────────────────────────
static void warm_start(std::vector<double>& xv, const Layout& L,
                       const Params& P, double dx_TD, double dx_LO_target,
                       double T_init) {
    const int N = L.N;
    const double dip = std::min(0.5 * (P.l - P.l_min), 0.05);
    const double pi  = 3.14159265358979323846;
    for (int k = 0; k <= N; ++k) {
        double r = (double)k / (double)N;
        // x: 从 l 下凹再回到 l
        xv[L.X(k)]  = P.l - dip * std::sin(pi * r);
        // dx: 从 dx_TD 线性插值到 dx_LO_target
        xv[L.DX(k)] = (1.0 - r) * dx_TD + r * dx_LO_target;
    }
    for (int k = 0; k < N; ++k) {
        xv[L.U(k)] = 0.0;
    }
    xv[L.IT()] = T_init;
}

} // anonymous

// ════════════════════════════════════════════════════════════════════
Result solve(const Params& p, double X0, const Options& opt) {
    Result R;

    // 阶段 0：解析自由落体
    if (X0 <= p.l) {
        std::fprintf(stderr, "[slip_trajopt] X0 (%.3f) must be > l (%.3f)\n", X0, p.l);
        return R;
    }
    if (opt.x_apex_target <= p.l) {
        std::fprintf(stderr,
            "[slip_trajopt] x_apex_target (%.3f) must be > l (%.3f)\n",
            opt.x_apex_target, p.l);
        return R;
    }
    const double t_TD         = std::sqrt(2.0 * (X0 - p.l) / kG);
    const double dx_TD        = -kG * t_TD;
    const double dx_LO_target = std::sqrt(2.0 * kG * (opt.x_apex_target - p.l));

    // 阶段 1：配置 NLP
    Layout L; L.N = opt.N;
    const int n_vars = L.n_vars();
    const int n_eq   = L.n_eq();

    UserData ud;
    ud.L            = L;
    ud.P            = p;
    ud.dx_TD        = dx_TD;
    ud.dx_LO_target = dx_LO_target;

    std::vector<double> xv(n_vars, 0.0);
    warm_start(xv, L, p, dx_TD, dx_LO_target, opt.T_init);

    // 边界
    constexpr double INF = std::numeric_limits<double>::infinity();
    std::vector<double> lb(n_vars, -INF), ub(n_vars, +INF);
    for (int k = 0; k <= L.N; ++k) {
        lb[L.X(k)]  = p.l_min;
        ub[L.X(k)]  = p.l;
    }
    for (int k = 0; k < L.N; ++k) {
        lb[L.U(k)] = -opt.u_max;
        ub[L.U(k)] = +opt.u_max;
    }
    lb[L.IT()] = opt.T_lo;
    ub[L.IT()] = opt.T_hi;

    // NLopt 调用
    nlopt::opt solver(nlopt::LD_SLSQP, n_vars);
    solver.set_lower_bounds(lb);
    solver.set_upper_bounds(ub);
    solver.set_min_objective(objective, &ud);
    std::vector<double> tols(n_eq, 1e-8);
    solver.add_equality_mconstraint(equality_constraints, &ud, tols);
    solver.set_xtol_rel(opt.xtol_rel);
    solver.set_ftol_rel(opt.ftol_rel);
    solver.set_maxeval(opt.max_iter);

    double obj = 0.0;
    nlopt::result code = nlopt::FAILURE;
    try {
        code = solver.optimize(xv, obj);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[slip_trajopt] NLopt exception: %s\n", e.what());
        R.nlopt_code = -1000;
        return R;
    }
    R.nlopt_code = (int)code;
    R.obj_value  = obj;
    R.n_eval     = solver.get_numevals();
    // SUCCESS / STOPVAL_REACHED / FTOL_REACHED / XTOL_REACHED / MAXEVAL_REACHED / MAXTIME_REACHED 都视为有效
    R.ok = (code > 0);

    // 提取结果
    R.dx_TD = dx_TD;
    R.t_TD  = t_TD;
    R.T     = xv[L.IT()];
    R.t_LO  = t_TD + R.T;
    R.x.resize(L.N + 1);
    R.dx.resize(L.N + 1);
    R.u.resize(L.N);
    for (int k = 0; k <= L.N; ++k) {
        R.x[k]  = xv[L.X(k)];
        R.dx[k] = xv[L.DX(k)];
    }
    for (int k = 0; k < L.N; ++k) {
        R.u[k] = xv[L.U(k)];
    }
    R.dx_LO  = R.dx[L.N];
    R.t_apex = R.t_LO + R.dx_LO / kG;
    R.x_apex = p.l + R.dx_LO * R.dx_LO / (2.0 * kG);

    // ── 阶段 2：解析飞行（取到稍稍越过 apex，便于看曲线）──
    // ── 拼接三段绘图轨迹 ──
    constexpr int kPhase0Samples = 24;
    constexpr int kPhase2Samples = 32;

    R.traj_t.clear();  R.traj_x.clear();  R.traj_dx.clear();
    R.traj_tu.clear(); R.traj_u.clear();
    R.traj_t.reserve(kPhase0Samples + (L.N + 1) + kPhase2Samples);
    R.traj_x.reserve(R.traj_t.capacity());
    R.traj_dx.reserve(R.traj_t.capacity());

    // 阶段 0
    for (int i = 0; i < kPhase0Samples; ++i) {
        double t = (double)i / (double)(kPhase0Samples - 1) * t_TD;
        double x  = X0 - 0.5 * kG * t * t;
        double v  = -kG * t;
        R.traj_t .push_back((float)t);
        R.traj_x .push_back((float)x);
        R.traj_dx.push_back((float)v);
        R.traj_tu.push_back((float)t);
        R.traj_u .push_back(0.0f);
    }

    // 阶段 1（状态节点）
    const double h = R.T / (double)L.N;
    for (int k = 0; k <= L.N; ++k) {
        double t = R.t_TD + (double)k * h;
        R.traj_t .push_back((float)t);
        R.traj_x .push_back((float)R.x[k]);
        R.traj_dx.push_back((float)R.dx[k]);
    }
    // 阶段 1（控制段中点）—— 控制是分段常值，画在段中点
    for (int k = 0; k < L.N; ++k) {
        double t_mid = R.t_TD + ((double)k + 0.5) * h;
        R.traj_tu.push_back((float)t_mid);
        R.traj_u .push_back((float)R.u[k]);
    }

    // 阶段 2 — 从 t_LO 飞到 t_apex 正好截断（apex 处 dx=0、x=x_apex）
    // 这样 live sim 跑到 traj_t.back() = t_apex 时正好处于「目标顶点状态」，
    // auto-pause 触发后机器人就静止在 target apex 上，不会再因重力坠落。
    for (int i = 1; i <= kPhase2Samples; ++i) {
        double t = R.t_LO + (double)i / (double)kPhase2Samples * (R.t_apex - R.t_LO);
        double dt = t - R.t_LO;
        double x  = p.l + R.dx_LO * dt - 0.5 * kG * dt * dt;
        double v  = R.dx_LO - kG * dt;
        R.traj_t .push_back((float)t);
        R.traj_x .push_back((float)x);
        R.traj_dx.push_back((float)v);
        R.traj_tu.push_back((float)t);
        R.traj_u .push_back(0.0f);
    }

    if (opt.verbose) {
        std::fprintf(stderr,
            "[slip_trajopt] code=%d evals=%d obj=%.4f  T=%.4fs"
            "  dx_TD=%.3f  dx_LO=%.3f(target %.3f)  apex=%.3f(target %.3f)\n",
            R.nlopt_code, R.n_eval, R.obj_value, R.T,
            R.dx_TD, R.dx_LO, dx_LO_target, R.x_apex, opt.x_apex_target);
    }
    return R;
}

} // namespace slip_trajopt
