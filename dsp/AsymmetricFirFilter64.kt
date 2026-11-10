package com.example.sffaudio.dsp

/**
 * 64 位双精度 (Double/IEEE 754) 高性能非对称 FIR 滤波器
 */
class AsymmetricFirFilter64(private val coeffsDouble: DoubleArray) {

    private val numTaps = coeffsDouble.size
    private val history = DoubleArray(numTaps)
    private var historyIndex = 0

    /**
     * 单采样 64 位 Double 处理
     */
    fun processSample(sample: Double): Double {
        history[historyIndex] = sample

        var accumulator = 0.0
        var currIdx = historyIndex

        for (i in 0 until numTaps) {
            accumulator += history[currIdx] * coeffsDouble[i]
            currIdx--
            if (currIdx < 0) {
                currIdx = numTaps - 1
            }
        }

        historyIndex++
        if (historyIndex >= numTaps) {
            historyIndex = 0
        }

        return accumulator
    }

    /**
     * 8 倍插值 (Zero-Padding) 结合 FIR 滤波处理
     * @param input44k 原始 44.1kHz / 48kHz 的双精度采样块
     * @param output384k 输出 352.8kHz / 384kHz 的双精度采样块 (长度必须是输入的 8 倍)
     */
    fun process8xInterpolation(input44k: DoubleArray, output384k: DoubleArray) {
        var outIdx = 0
        for (i in input44k.indices) {
            // 1. 插入原始采样 (乘以 8.0 补偿零值插值的能量衰减)
            output384k[outIdx++] = processSample(input44k[i] * 8.0)

            // 2. 补 7 个零 (Zero-Padding)
            for (z in 0 until 7) {
                output384k[outIdx++] = processSample(0.0)
            }
        }
    }

    fun reset() {
        history.fill(0.0)
        historyIndex = 0
    }
}
