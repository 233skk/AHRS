/**
 * @file    mag_calibration.h
 * @brief   磁力计椭球拟合校准 — 硬铁 + 软铁 + 比例因子
 *
 * 算法：两步法椭球拟合
 *   Step 1: 球面拟合 |X-C|²=R² → 4×4 线性系统求硬铁偏置 C
 *   Step 2: 中心化后 6 参数二次型拟合 → Cholesky 分解求软铁 W
 *
 * 原理：理想磁力计三轴读数构成原点在 (0,0,0) 的正球面。
 *       实际因硬铁偏移 + 软铁扭曲 + 轴比例不一致 → 斜椭球。
 *       拟合 ellipsoid → 变换回球面 → 得到 B_true = W·(B_raw - C)
 *
 * 使用：
 *   1. MagCalibration calib;
 *   2. 旋转设备采集各方向: calib.addSample(mx, my, mz);  // 建议 300+
 *   3. calib.compute();  // 拟合，成功返回 true
 *   4. calib.apply(mx, my, mz, &mx_cal, &my_cal, &mz_cal);  // 每帧校正
 */

#ifndef MAG_CALIBRATION_H
#define MAG_CALIBRATION_H

#include <cmath>
#include <cstring>
#include <cstdio>

class MagCalibration {
public:
    MagCalibration() { reset(); }

    // ---- 数据采集 ----

    void reset() {
        count_      = 0;
        calibrated_ = false;
        field_norm_ = 1.0f;
    }

    /// 添加原始磁力计采样 (单位任意，保持一致即可, 推荐 Gauss)
    void addSample(float mx, float my, float mz) {
        if (count_ >= MAX_SAMPLES) return;
        samples_[count_][0] = mx;
        samples_[count_][1] = my;
        samples_[count_][2] = mz;
        count_++;
    }

    int sampleCount() const { return count_; }

    // ---- 椭球拟合 ----

    /// 两步法椭球拟合：先球面拟合定中心 → 中心化后 6 参数求软铁
    /// Step 1: 球面拟合 |X-C|² = R² → 硬铁偏置 C
    /// Step 2: 中心化后拟合 X'^T·Q·X' = 1 → Cholesky → 软铁 W
    bool compute() {
        if (count_ < 30) return false;

        // ==== Step 1: 球面拟合求中心 (硬铁偏置) ====
        // 线性化: x²+y²+z² = 2cx·x + 2cy·y + 2cz·z + (R² - |C|²)
        // 未知量: [cx, cy, cz, d] 其中 d = R² - |C|²
        // 这是一个 4×4 线性系统，不受共线性影响

        double S[16] = {0};  // 4×4 正规方程矩阵
        double t[4]  = {0};  // RHS

        for (int i = 0; i < count_; i++) {
            double x = samples_[i][0];
            double y = samples_[i][1];
            double z = samples_[i][2];
            double r2 = x*x + y*y + z*z;

            // 设计矩阵行: [2x, 2y, 2z, 1]
            double row[4] = {2.0*x, 2.0*y, 2.0*z, 1.0};

            for (int r = 0; r < 4; r++) {
                for (int c = 0; c < 4; c++) {
                    S[r*4 + c] += row[r] * row[c];
                }
                t[r] += row[r] * r2;
            }
        }

        // 解 4×4: S · [cx, cy, cz, d]^T = t
        double sol[4];
        if (!solveLinearN(S, t, sol, 4)) return false;

        hard_iron_[0] = (float)sol[0];  // cx
        hard_iron_[1] = (float)sol[1];  // cy
        hard_iron_[2] = (float)sol[2];  // cz
        double R2_est = sol[3] + sol[0]*sol[0] + sol[1]*sol[1] + sol[2]*sol[2];
        if (R2_est <= 0.0) return false;
        field_norm_ = (float)sqrt(R2_est);

        // ==== Step 2: 中心化后 6 参数二次型拟合 ====
        // X' = X - C, 拟合: a·x'² + b·y'² + c·z'² + 2f·x'y' + 2g·x'z' + 2h·y'z' = R²
        // 6 未知数，设计矩阵列: [x'², y'², z'², 2x'y', 2x'z', 2y'z']
        // 没有一次项 → 不受共线性影响，6×6 矩阵条件良好

        double A6[36] = {0};
        double b6[6]  = {0};

        for (int i = 0; i < count_; i++) {
            double x = samples_[i][0] - hard_iron_[0];
            double y = samples_[i][1] - hard_iron_[1];
            double z = samples_[i][2] - hard_iron_[2];

            double row[6] = {x*x, y*y, z*z, 2.0*x*y, 2.0*x*z, 2.0*y*z};

            for (int r = 0; r < 6; r++) {
                for (int c = 0; c < 6; c++) {
                    A6[r*6 + c] += row[r] * row[c];
                }
                b6[r] += row[r] * R2_est;  // RHS = R² (使得校正后 |B|=R)
            }
        }

        double theta[6];
        if (!solveLinearN(A6, b6, theta, 6)) return false;

        double a = theta[0], b_ = theta[1], c = theta[2];
        double f = theta[3], g = theta[4], h = theta[5];

        // 检查正定性
        if (a <= 0.0 || b_ <= 0.0 || c <= 0.0) return false;

        // 构造二次型矩阵 Q = [[a, f, g], [f, b_, h], [g, h, c]]
        // Q 已按 R² 归一化 (因为 RHS = R²)，所以校正后 |B_cal| ≈ R
        double Q[9] = {a, f, g, f, b_, h, g, h, c};

        // Cholesky 分解 Q = L·L^T → 软铁 W = L^T
        double L[9] = {0};
        if (!cholesky3(Q, L)) return false;

        for (int r = 0; r < 3; r++)
            for (int c_ = 0; c_ < 3; c_++)
                soft_iron_[r*3 + c_] = (float)L[c_*3 + r];

        calibrated_ = true;
        return true;
    }

    // ---- 校正参数 ----

    bool isCalibrated() const { return calibrated_; }

    void getHardIron(float& bx, float& by, float& bz) const {
        bx = hard_iron_[0]; by = hard_iron_[1]; bz = hard_iron_[2];
    }

    /// 返回 3×3 软铁校正矩阵 (行主), B_cal = W · (B_raw - C)
    void getSoftIron(float m[9]) const {
        for (int i = 0; i < 9; i++) m[i] = soft_iron_[i];
    }

    /// 对原始采样做完整校正: B_cal = W · (B_raw - C)
    /// 输出单位为 Gauss，magnitude ≈ 本地磁场强度
    void apply(float mx_raw, float my_raw, float mz_raw,
               float& mx_cal, float& my_cal, float& mz_cal) const
    {
        float dx = mx_raw - hard_iron_[0];
        float dy = my_raw - hard_iron_[1];
        float dz = mz_raw - hard_iron_[2];

        // W · (X - C) → 幅值已为 field_norm_ (Gauss)
        mx_cal = soft_iron_[0]*dx + soft_iron_[1]*dy + soft_iron_[2]*dz;
        my_cal = soft_iron_[3]*dx + soft_iron_[4]*dy + soft_iron_[5]*dz;
        mz_cal = soft_iron_[6]*dx + soft_iron_[7]*dy + soft_iron_[8]*dz;
    }

    /// 返回校正后的磁场强度标尺 (Gauss)
    float getFieldNorm() const { return field_norm_; }

    /// 验证校正质量: 对所有采样做校正，输出 magnitude 的均值/标准差
    /// 理想: mean ≈ 本地磁场强度(Gauss), stddev ≪ mean
    void verifyCalibration() const {
        if (count_ == 0) return;
        double sum = 0, sumSq = 0;
        for (int i = 0; i < count_; i++) {
            float cx, cy, cz;
            apply(samples_[i][0], samples_[i][1], samples_[i][2], cx, cy, cz);
            double mag = sqrt((double)cx*cx + (double)cy*cy + (double)cz*cz);
            sum   += mag;
            sumSq += mag * mag;
        }
        double mean = sum / count_;
        double stdv = sqrt(sumSq / count_ - mean * mean);
        printf("\n===== 校正质量验证 =====\n");
        printf("采样数: %d\n", count_);
        printf("校正后磁场均值: %.3f Gauss (%.0f mGa)\n", mean, mean*1000);
        printf("校正后磁场标准差: %.3f Gauss (%.0f mGa)\n", stdv, stdv*1000);
        printf("变异系数: %.1f%%  %s\n", 100.0*stdv/mean,
               (stdv/mean < 0.05) ? "✓ 良好" :
               (stdv/mean < 0.10) ? "△ 可接受" : "✗ 建议重新采集");
    }

    /// 打印校正参数 (人类可读)
    void printParameters() const {
        printf("\n===== 磁力计校准参数 =====\n");
        printf("本地磁场强度: %.3f Gauss (%.0f mGa)\n", field_norm_, field_norm_*1000);
        printf("硬铁偏置 (Gauss):\n");
        printf("  bx = %+.6f\n", hard_iron_[0]);
        printf("  by = %+.6f\n", hard_iron_[1]);
        printf("  bz = %+.6f\n", hard_iron_[2]);
        printf("\n软铁校正矩阵 (3×3, 行主):\n");
        for (int r = 0; r < 3; r++) {
            printf("  ");
            for (int c_ = 0; c_ < 3; c_++) {
                printf("%+10.6f  ", soft_iron_[r*3 + c_]);
            }
            printf("\n");
        }
        printf("\n// C++ 常量 (可直接拷进代码):\n");
        printf("static const float mag_bias[3] = {%.6ff, %.6ff, %.6ff};\n",
               hard_iron_[0], hard_iron_[1], hard_iron_[2]);
        printf("static const float mag_soft[9] = {\n");
        for (int r = 0; r < 3; r++) {
            printf("    %.6ff, %.6ff, %.6ff%s\n",
                   soft_iron_[r*3+0], soft_iron_[r*3+1], soft_iron_[r*3+2],
                   r < 2 ? "," : "");
        }
        printf("};\n");
    }

private:
    static const int MAX_SAMPLES = 2048;

    float samples_[MAX_SAMPLES][3];
    int   count_;
    bool  calibrated_;

    float hard_iron_[3];
    float soft_iron_[9];   // 3×3 行主，B_cal = W·(B_raw - C)，幅值为 field_norm_
    float field_norm_;     // 本地磁场强度 R (Gauss)

    // ---- 3×3 Cholesky 分解 A = L·L^T (A 必须对称正定) ----
    static bool cholesky3(const double A[9], double L[9]) {
        std::memset(L, 0, 9 * sizeof(double));

        L[0] = sqrt(A[0]);
        if (L[0] < 1e-12) return false;

        L[3] = A[3] / L[0];                          // L[1,0] = A[1,0] / L[0,0]
        L[4] = A[4] - L[3]*L[3];
        if (L[4] < 1e-12) return false;
        L[4] = sqrt(L[4]);

        L[6] = A[6] / L[0];                          // L[2,0]
        L[7] = (A[7] - L[6]*L[3]) / L[4];             // L[2,1]
        L[8] = A[8] - L[6]*L[6] - L[7]*L[7];
        if (L[8] < 1e-12) return false;
        L[8] = sqrt(L[8]);

        return true;
    }

    // ---- N×N 高斯消元 + 列主元 (通用) ----
    static bool solveLinearN(const double A[], const double b[], double x[], int n) {
        // 增广矩阵 [A|b], 原地消元
        // 动态分配放栈上，最大支持 9×9 (当前用 4 和 6)
        double aug[9][10];

        for (int r = 0; r < n; r++) {
            for (int c = 0; c < n; c++) {
                aug[r][c] = A[r*n + c];
            }
            aug[r][n] = b[r];
        }

        for (int col = 0; col < n; col++) {
            // 列主元
            int pivot = col;
            double maxVal = fabs(aug[col][col]);
            for (int r = col + 1; r < n; r++) {
                if (fabs(aug[r][col]) > maxVal) {
                    maxVal = fabs(aug[r][col]);
                    pivot = r;
                }
            }
            if (maxVal < 1e-15) return false;

            if (pivot != col) {
                for (int c = 0; c <= n; c++) {
                    double tmp = aug[col][c];
                    aug[col][c] = aug[pivot][c];
                    aug[pivot][c] = tmp;
                }
            }

            // 消去
            double piv = aug[col][col];
            for (int c = col; c <= n; c++) aug[col][c] /= piv;

            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double factor = aug[r][col];
                for (int c = col; c <= n; c++) {
                    aug[r][c] -= factor * aug[col][c];
                }
            }
        }

        for (int r = 0; r < n; r++) {
            x[r] = aug[r][n];
        }

        return true;
    }
};

#endif // MAG_CALIBRATION_H
