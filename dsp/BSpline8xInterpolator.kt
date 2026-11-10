package com.example.sffaudio.dsp

/**
 * 8x PCM 采样率转换器 - 基于三次 B-Spline (Cubic B-Spline) 多相插值
 *
 * 特性：
 * 1. 绝对零预振铃 (Zero Pre-ringing)，具有 C^2 连续的极致时域平滑度。
 * 2. 8 相位 4-tap 预计算系数表，离线转换效率极高，内部零内存分配。
 *
 * 相比传统 Sinc/Windowed-FIR 多相滤波器，B-Spline 是纯粹的单调/凸样条基函数，
 * 在物理上完全不存在前振铃，声学瞬态极为干净，与 PFM 调制器结合时呈现模拟润泽度。
 */
class BSpline8xInterpolator(
    private val mode: SplineMode = SplineMode.CUBIC_BSPLINE
) {

    enum class SplineMode {
        CUBIC_BSPLINE, // 标准三次 B-Spline：天然自带极佳的高频柔化与抗混叠，听感模拟温润
        CATMULL_ROM    // Catmull-Rom 样条：精确经过原始采样点 (Node-passing)，高频解析度略高
    }

    // 预计算 8 个相位、每个相位 4 个 Taps 的权重表 [phase][tap]
    private val polyphaseWeights = Array(8) { DoubleArray(4) }

    init {
        precalculatePolyphaseWeights()
    }

    private fun precalculatePolyphaseWeights() {
        for (phase in 0 until 8) {
            val t = phase / 8.0
            val t2 = t * t
            val t3 = t2 * t

            when (mode) {
                SplineMode.CUBIC_BSPLINE -> {
                    // 三次 B-Spline 基函数 (归一化 DC 增益 = 1.0)
                    polyphaseWeights[phase][0] = (-t3 + 3.0 * t2 - 3.0 * t + 1.0) / 6.0
                    polyphaseWeights[phase][1] = (3.0 * t3 - 6.0 * t2 + 4.0) / 6.0
                    polyphaseWeights[phase][2] = (-3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0) / 6.0
                    polyphaseWeights[phase][3] = t3 / 6.0
                }
                SplineMode.CATMULL_ROM -> {
                    // Catmull-Rom 基函数 (a = -0.5)
                    polyphaseWeights[phase][0] = -0.5 * t3 + t2 - 0.5 * t
                    polyphaseWeights[phase][1] = 1.5 * t3 - 2.5 * t2 + 1.0
                    polyphaseWeights[phase][2] = -1.5 * t3 + 2.0 * t2 + 0.5 * t
                    polyphaseWeights[phase][3] = 0.5 * t3 - 0.5 * t2
                }
            }
        }
    }

    /**
     * 对交织双声道 (Interleaved Stereo) PCM 进行 8 倍升采样
     * @param inPcm1x 输入 1x 采样率 PCM (格式: [L0, R0, L1, R1, ...])
     * @return 8x 升采样后的 PCM 数组 (长度为 inPcm1x.size * 8)
     */
    fun processStereo8x(inPcm1x: DoubleArray): DoubleArray {
        val numFrames = inPcm1x.size / 2
        val outPcm8x = DoubleArray(inPcm1x.size * 8)
        var outIdx = 0

        for (i in 0 until numFrames) {
            // 镜像/钳位边界保护，获取连续 4 个历史/未来控制点 Index
            val i0 = if (i - 1 >= 0) i - 1 else 0
            val i1 = i
            val i2 = if (i + 1 < numFrames) i + 1 else numFrames - 1
            val i3 = if (i + 2 < numFrames) i + 2 else numFrames - 1

            val l0 = inPcm1x[i0 * 2];     val r0 = inPcm1x[i0 * 2 + 1]
            val l1 = inPcm1x[i1 * 2];     val r1 = inPcm1x[i1 * 2 + 1]
            val l2 = inPcm1x[i2 * 2];     val r2 = inPcm1x[i2 * 2 + 1]
            val l3 = inPcm1x[i3 * 2];     val r3 = inPcm1x[i3 * 2 + 1]

            // 展开为 8 个插值点
            for (phase in 0 until 8) {
                val w = polyphaseWeights[phase]

                // 4-Tap 极速乘加
                val outL = w[0] * l0 + w[1] * l1 + w[2] * l2 + w[3] * l3
                val outR = w[0] * r0 + w[1] * r1 + w[2] * r2 + w[3] * r3

                outPcm8x[outIdx++] = outL
                outPcm8x[outIdx++] = outR
            }
        }
        return outPcm8x
    }
}
