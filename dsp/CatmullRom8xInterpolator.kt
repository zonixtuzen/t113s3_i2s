package com.example.sffaudio.dsp

/**
 * 单声道 Catmull-Rom 8x 插值器（v12.5.7.2 新增，用于 Hybrid 分频低频通道）
 *
 * 穿点插值（Interpolating）：C1 连续、零预振铃、紧支集。频响平直，高频无 Droop 滚降。
 *
 * 与 v12.5.4~v12.5.7.1 低频通道使用的非对称 apodizing FIR 不同，本插值器采用纯粹的
 * Catmull-Rom 三次样条基（a = -0.5），配合 4 点滑动历史窗口实现跨 Block 连续（防断层白噪）。
 *
 * 注意：本类为「单声道」插值器，左右声道各持一个实例（与旧版 AsymmetricFirFilter64 用法一致）。
 * 输入采样须已归一化到 [-1.0, 1.0]。
 */
class CatmullRom8xInterpolator {

    // 1x 域历史状态点（用于跨 Block 连续性，防断层白噪）
    // 窗口包含：[x_{n-1}, x_n, x_{n+1}, x_{n+2}]
    private val history = DoubleArray(4)

    /**
     * 重置历史状态。
     * ⚠️ 仅在播放全新歌曲或启动离线 DSF 导出起点时调用一次，绝不要在分块循环内部调用。
     */
    fun reset() {
        history.fill(0.0)
    }

    /**
     * 8x 插值：输入 1x 采样点数组，输出 8x 采样点数组（长度 = 输入的 8 倍）。
     * @param input1x 归一化的 1x 采样点（长度 >= 1）
     * @param output8x 接收 8x 采样点（长度 = input1x.size * 8）
     */
    fun process8xInterpolation(input1x: DoubleArray, output8x: DoubleArray) {
        var outIdx = 0
        for (i in input1x.indices) {
            shiftHistory(input1x[i])
            // 在 [x_n, x_{n+1}] 区间内平滑插值出 8 个点（phase alpha 从 0.0 到 7/8）
            for (phase in 0 until 8) {
                output8x[outIdx++] = interpolateCatmullRom(phase / 8.0)
            }
        }
    }

    // 更新 4-tap 滑动历史窗口
    private inline fun shiftHistory(newSample: Double) {
        history[0] = history[1]
        history[1] = history[2]
        history[2] = history[3]
        history[3] = newSample
    }

    /**
     * Catmull-Rom 三次样条插值核（a = -0.5）
     * C1 连续（一阶导数连续），穿点插值，零预振铃，紧支集。
     */
    private inline fun interpolateCatmullRom(t: Double): Double {
        val t2 = t * t
        val t3 = t2 * t
        return 0.5 * (
            (2.0 * history[1]) +
            (-history[0] + history[2]) * t +
            (2.0 * history[0] - 5.0 * history[1] + 4.0 * history[2] - history[3]) * t2 +
            (-history[0] + 3.0 * history[1] - 3.0 * history[2] + history[3]) * t3
        )
    }
}