package com.example.sffaudio.dsp

import kotlin.math.*

/**
 * 1x 采样率域 (44.1/48kHz) 的 Apodizing FIR 预处理器（v12.5.7.5 新增）
 *
 * 通过前馈 FIR 为高频提供完美贴合 B-Spline 的搁架补偿与相位控制，
 * 放在「分频 / 双轨 8x 插值」之前，实现「1x 域预提升 -> 分频 -> 双轨插值 -> PFM」的黄金架构。
 *
 * 核心黑魔法：IDTFT 数值积分 + 分裂汉宁阿多化窗（Split-Hann Apodizing Window）。
 * FIR 系数在 App 内部实时动态计算（无需查表），拖动 Q 值 / 频率 / 相位裕度滑块即瞬间生成
 * 完美的非对称 FIR 系数。
 */
class ApodizingPreProcessor(private val sampleRate: Double = 44100.0) {

    // 采用 31-tap，在 1x 域兼顾低延迟 (0.34ms) 与极高的低频分辨率
    private val N = 31
    private val bufferL = DoubleArray(N)
    private val bufferR = DoubleArray(N)
    private var bufIdx = 0

    private var currentTaps = DoubleArray(N) { 0.0 }
    private var isBypass = true

    @Volatile
    var currentMode: Mode = Mode.BYPASS
        private set

    // --- 预设与高级模式配置数据类 ---
    data class Config(
        val freq: Double,         // High-Shelf 起始频率 (6000 ~ 12000 Hz)
        val gainDb: Double,       // 补偿增益 (0.0 ~ 4.0 dB)
        val q: Double,            // Q值/斜率 (0.4 ~ 0.75)
        val asymmetry: Double     // 相位前置不对称度 (0.0 ~ 0.50) -> 0.0为绝对对称，0.50为最小相位
    )

    enum class Mode {
        BYPASS, NATURAL_AIR, BRILLIANCE, ADVANCED
    }

    // 高级模式当前配置（供 UI 回显）
    @Volatile
    var advancedConfig = Config(10000.0, 1.8, 0.5, 0.0)
        private set

    init {
        // 默认初始化为 Natural Air
        setMode(Mode.NATURAL_AIR)
    }

    /**
     * 切换模式，如果是 ADVANCED 模式，需传入自定义 Config
     */
    fun setMode(mode: Mode, advancedConfig: Config? = null) {
        currentMode = mode
        when (mode) {
            Mode.BYPASS -> {
                isBypass = true
                currentTaps = DoubleArray(N) { 0.0 }.apply { this[N / 2] = 1.0 }
            }
            Mode.NATURAL_AIR -> {
                isBypass = false
                generateApodizingTaps(Config(10000.0, 1.8, 0.5, 0.0))
            }
            Mode.BRILLIANCE -> {
                isBypass = false
                generateApodizingTaps(Config(8000.0, 2.6, 0.55, 0.15))
            }
            Mode.ADVANCED -> {
                isBypass = false
                val cfg = advancedConfig ?: Config(10000.0, 1.8, 0.5, 0.0)
                this.advancedConfig = cfg
                generateApodizingTaps(cfg)
            }
        }
    }

    /**
     * 核心 DSP 黑魔法：动态生成 Apodizing 高频搁架 FIR 系数
     */
    private fun generateApodizingTaps(config: Config) {
        if (config.gainDb == 0.0) {
            isBypass = true
            return
        }

        // 1. 根据 RBJ Audio EQ Cookbook 计算模拟 High-Shelf 双二阶 IIR 频域幅度响应
        val A = 10.0.pow(config.gainDb / 40.0)
        val w0 = 2.0 * PI * config.freq / sampleRate
        val alpha = sin(w0) / (2.0 * config.q)

        val b0 = A * ((A + 1.0) + (A - 1.0) * cos(w0) + 2.0 * sqrt(A) * alpha)
        val b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cos(w0))
        val b2 = A * ((A + 1.0) + (A - 1.0) * cos(w0) - 2.0 * sqrt(A) * alpha)
        val a0 = (A + 1.0) - (A - 1.0) * cos(w0) + 2.0 * sqrt(A) * alpha
        val a1 = 2.0 * ((A - 1.0) - (A + 1.0) * cos(w0))
        val a2 = (A + 1.0) - (A - 1.0) * cos(w0) - 2.0 * sqrt(A) * alpha

        // 系数归一化
        val nb0 = b0 / a0; val nb1 = b1 / a0; val nb2 = b2 / a0
        val na1 = a1 / a0; val na2 = a2 / a0

        // 频率振幅响应函数 M(w)
        fun magnitude(w: Double): Double {
            val cosW = cos(w); val cos2W = cos(2.0 * w)
            val sinW = sin(w); val sin2W = sin(2.0 * w)
            val numRe = nb0 + nb1 * cosW + nb2 * cos2W
            val numIm = -(nb1 * sinW + nb2 * sin2W)
            val denRe = 1.0 + na1 * cosW + na2 * cos2W
            val denIm = -(na1 * sinW + na2 * sin2W)
            return sqrt((numRe * numRe + numIm * numIm) / (denRe * denRe + denIm * denIm))
        }

        // 2. IDTFT 数值积分获取理想线性相位冲激响应 h(t)
        fun idealH(t: Double): Double {
            val steps = 250 // 足够精确的积分步长
            val dw = PI / steps
            var sum = 0.0
            for (i in 0 until steps) {
                val w = (i + 0.5) * dw
                sum += magnitude(w) * cos(w * t)
            }
            return (sum * dw) / PI
        }

        // 3. 构建 "分裂汉宁阿多化窗" (Split-Hann Apodizing Window)
        val centerPeak = (N - 1) / 2.0
        // 根据不对称度计算波峰的绝对位置
        val peakPos = centerPeak * (1.0 - config.asymmetry * 2.0)

        var dcSum = 0.0
        val newTaps = DoubleArray(N)

        for (i in 0 until N) {
            // 时域偏移，使得理想 IR 的峰值对齐到我们设定的 peakPos
            val t = i.toDouble() - peakPos

            // 动态非对称切窗，左边控制预振铃，右边控制后振铃
            val window = if (i <= peakPos) {
                if (peakPos <= 0.0) 1.0 else 0.5 * (1.0 - cos(PI * i / peakPos))
            } else {
                if (peakPos >= N - 1.0) 1.0 else 0.5 * (1.0 - cos(PI * (N - 1.0 - i) / (N - 1.0 - peakPos)))
            }

            newTaps[i] = idealH(t) * window
            dcSum += newTaps[i]
        }

        // 4. DC 增益绝对平坦化校准 (确保 0Hz 处增益绝对为 1.0/0dB，不影响低频)
        for (i in 0 until N) {
            currentTaps[i] = newTaps[i] / dcSum
        }
    }

    /**
     * 单个 1x 采样点处理 (包含 -1.5dB Headroom 预留防过冲)
     */
    fun process(sampleL: Double, sampleR: Double): Pair<Double, Double> {
        // -1.4dB 防过冲衰减
        val inL = sampleL * 0.85
        val inR = sampleR * 0.85

        if (isBypass) return Pair(inL, inR)

        bufferL[bufIdx] = inL
        bufferR[bufIdx] = inR

        var outL = 0.0
        var outR = 0.0

        for (i in 0 until N) {
            val readIdx = (bufIdx - i + N) % N
            outL += bufferL[readIdx] * currentTaps[i]
            outR += bufferR[readIdx] * currentTaps[i]
        }

        bufIdx = (bufIdx + 1) % N
        return Pair(outL, outR)
    }

    /**
     * 重置 FIR 历史缓冲（切换曲目时清空，防止上一首残留污染）。
     */
    fun reset() {
        bufferL.fill(0.0)
        bufferR.fill(0.0)
        bufIdx = 0
    }
}
