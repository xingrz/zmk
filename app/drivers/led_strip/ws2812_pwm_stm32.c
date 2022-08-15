/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2019, Nordic Semiconductor ASA
 * Copyright (c) 2021 Seagate Technology LLC
 * Copyright (c) 2022 Radiotechnical Systems LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT worldsemi_ws2812_pwm

#include <drivers/led_strip.h>

#include <string.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL

#include <logging/log.h>

LOG_MODULE_REGISTER(ws2812_pwm);

#include <zephyr.h>
#include <device.h>
#include <sys/math_extras.h>
#include <sys/util.h>
#include <dt-bindings/led/led.h>
#include <drivers/pwm.h>
#include <devicetree/pwms.h>
#include <stm32_ll_tim.h>
#include <drivers/clock_control/stm32_clock_control.h>
#include <sys_clock.h>

struct pwm_stm32_config {
    /** Timer instance. */
    TIM_TypeDef *timer;
    /** Prescaler. */
    uint32_t prescaler;
    /** Clock configuration. */
    struct stm32_pclken pclken;
    /** pinctrl configurations. */
    const struct soc_gpio_pinctrl *pinctrl;
    /** Number of pinctrl configurations. */
    size_t pinctrl_len;
};

struct pwm {
    const struct device *dev;
    uint32_t channel;
    uint32_t period;
    pwm_flags_t flags;
};

struct ws2812_pwm_cfg {
    const struct pwm pwm;
    uint8_t *px_buf;
    size_t px_buf_size;
    uint8_t num_colors;
    const uint8_t *color_mapping;
    uint16_t t0h_ns;
    uint16_t t1h_ns;
    uint16_t period_ns;
    uint16_t reset_delay;
};

struct ws2812_pwm_data {
    uint32_t pwm_period_cycles;
    uint32_t pwm_t0h_cycles;
    uint32_t pwm_t1h_cycles;
};

/** Series F3, F7, G0, G4, H7, L4, MP1 and WB have up to 6 channels, others up
 *  to 4.
 */
#if defined(CONFIG_SOC_SERIES_STM32F3X) || defined(CONFIG_SOC_SERIES_STM32F7X) ||                  \
    defined(CONFIG_SOC_SERIES_STM32G0X) || defined(CONFIG_SOC_SERIES_STM32G4X) ||                  \
    defined(CONFIG_SOC_SERIES_STM32H7X) || defined(CONFIG_SOC_SERIES_STM32L4X) ||                  \
    defined(CONFIG_SOC_SERIES_STM32MP1X) || defined(CONFIG_SOC_SERIES_STM32WBX)
#define TIMER_HAS_6CH 1
#else
#define TIMER_HAS_6CH 0
#endif

/** Maximum number of timer channels. */
#if TIMER_HAS_6CH
#define TIMER_MAX_CH 6u
#else
#define TIMER_MAX_CH 4u
#endif

/** Channel to LL mapping. */
static const uint32_t ch2ll[TIMER_MAX_CH] = {
    LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4,
#if TIMER_HAS_6CH
    LL_TIM_CHANNEL_CH5, LL_TIM_CHANNEL_CH6
#endif
};

/** Channel to compare set function mapping. */
static void (*const set_timer_compare[TIMER_MAX_CH])(TIM_TypeDef *, uint32_t) = {
    LL_TIM_OC_SetCompareCH1, LL_TIM_OC_SetCompareCH2, LL_TIM_OC_SetCompareCH3,
    LL_TIM_OC_SetCompareCH4,
#if TIMER_HAS_6CH
    LL_TIM_OC_SetCompareCH5, LL_TIM_OC_SetCompareCH6
#endif
};

/** Channel to compare set function mapping. */
static uint32_t (*const is_active_timer_flag[TIMER_MAX_CH])(TIM_TypeDef *) = {
    LL_TIM_IsActiveFlag_CC1, LL_TIM_IsActiveFlag_CC2, LL_TIM_IsActiveFlag_CC3,
    LL_TIM_IsActiveFlag_CC4,
#if TIMER_HAS_6CH
    LL_TIM_IsActiveFlag_CC5, LL_TIM_IsActiveFlag_CC5
#endif
};

/** Channel to compare set function mapping. */
static void (*const clear_timer_flag[TIMER_MAX_CH])(TIM_TypeDef *) = {
    LL_TIM_ClearFlag_CC1, LL_TIM_ClearFlag_CC2, LL_TIM_ClearFlag_CC3, LL_TIM_ClearFlag_CC4,
#if TIMER_HAS_6CH
    LL_TIM_ClearFlag_CC5, LL_TIM_ClearFlag_CC6
#endif
};

/** Channel to compare set function mapping. */
static void (*const generate_timer_event[TIMER_MAX_CH])(TIM_TypeDef *) = {
    LL_TIM_GenerateEvent_CC1, LL_TIM_GenerateEvent_CC1, LL_TIM_GenerateEvent_CC1,
    LL_TIM_GenerateEvent_CC1,
#if TIMER_HAS_6CH
    LL_TIM_GenerateEvent_CC1, LL_TIM_GenerateEvent_CC1
#endif
};

/*
 * Returns true if and only if cfg->px_buf is big enough to convert
 * num_pixels RGB color values into PWM frames.
 */
static inline bool num_pixels_ok(const struct ws2812_pwm_cfg *cfg, size_t num_pixels) {
    size_t nbytes;
    bool overflow;

    overflow = size_mul_overflow(num_pixels, cfg->num_colors, &nbytes);
    return !overflow && (nbytes <= cfg->px_buf_size);
}

/**
 * Obtain LL polarity from PWM flags.
 *
 * @param flags PWM flags.
 *
 * @return LL polarity.
 */
static uint32_t get_polarity(pwm_flags_t flags) {
    if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_NORMAL) {
        return LL_TIM_OCPOLARITY_HIGH;
    }

    return LL_TIM_OCPOLARITY_LOW;
}

static int ws2812_pwm_write(const struct device *dev) {
    unsigned int key;
    const struct ws2812_pwm_cfg *cfg = dev->config;
    struct ws2812_pwm_data *ctx = dev->data;
    const struct pwm_stm32_config *pwm_cfg = cfg->pwm.dev->config;
    TIM_TypeDef *tim = pwm_cfg->timer;
    uint8_t *buf = cfg->px_buf;

    uint32_t channel = ch2ll[cfg->pwm.channel - 1u];

    LL_TIM_OC_InitTypeDef oc_init;
    LL_TIM_OC_StructInit(&oc_init);

    oc_init.OCMode = LL_TIM_OCMODE_PWM1;
    oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
    oc_init.OCPolarity = get_polarity(cfg->pwm.flags);

    if (LL_TIM_OC_Init(tim, channel, &oc_init) != SUCCESS) {
        LOG_ERR("Could not initialize timer channel output");
        return -EIO;
    }

    key = irq_lock();
    LL_TIM_EnableARRPreload(tim);
    LL_TIM_OC_EnablePreload(tim, channel);
    LL_TIM_SetAutoReload(tim, ctx->pwm_period_cycles - 1u);
    LL_TIM_GenerateEvent_UPDATE(tim);
    generate_timer_event[cfg->pwm.channel](tim);
    clear_timer_flag[cfg->pwm.channel](tim);

    for (size_t i = 0; i < cfg->px_buf_size; i++) {
        uint8_t tmp = *buf++;
        for (int j = 0; j < 8; j++) {
            uint32_t value = (tmp & 0x80) ? ctx->pwm_t1h_cycles : ctx->pwm_t0h_cycles;
            set_timer_compare[cfg->pwm.channel - 1u](tim, value);
            while (!is_active_timer_flag[cfg->pwm.channel](tim))
                ;
            clear_timer_flag[cfg->pwm.channel](tim);
            tmp <<= 1;
        }
    }

    set_timer_compare[cfg->pwm.channel - 1u](tim, 0);
    while (!is_active_timer_flag[cfg->pwm.channel](tim))
        ;
    LL_TIM_CC_DisableChannel(tim, channel);
    irq_unlock(key);
    return 0;
}

/*
 * Latch current color values on strip and reset its state machines.
 */
static inline void ws2812_reset_delay(uint16_t delay) { k_usleep(delay); }

static int ws2812_strip_update_rgb(const struct device *dev, struct led_rgb *pixels,
                                   size_t num_pixels) {
    const struct ws2812_pwm_cfg *cfg = dev->config;
    uint8_t *px_buf = cfg->px_buf;
    size_t i;
    int rc;

    if (!num_pixels_ok(cfg, num_pixels)) {
        return -ENOMEM;
    }

    /*
     * Convert pixel data into PWM frames. Each frame has pixel data
     * in color mapping on-wire format (e.g. GRB, GRBW, RGB, etc).
     */
    for (i = 0; i < num_pixels; i++) {
        uint8_t j;

        for (j = 0; j < cfg->num_colors; j++) {
            switch (cfg->color_mapping[j]) {
            /* White channel is not supported by LED strip API. */
            case LED_COLOR_ID_WHITE:
                *px_buf = 0;
                break;
            case LED_COLOR_ID_RED:
                *px_buf = pixels->r;
                break;
            case LED_COLOR_ID_GREEN:
                *px_buf = pixels->g;
                break;
            case LED_COLOR_ID_BLUE:
                *px_buf = pixels->b;
                break;
            default:
                return -EINVAL;
            }
            px_buf++;
        }
        pixels++;
    }

    /*
     * Display the pixel data.
     */
    rc = ws2812_pwm_write(dev);
    ws2812_reset_delay(cfg->reset_delay);

    return rc;
}

static int ws2812_strip_update_channels(const struct device *dev, uint8_t *channels,
                                        size_t num_channels) {
    LOG_ERR("update_channels not implemented");
    return -ENOTSUP;
}

static int ws2812_pwm_init(const struct device *dev) {
    const struct ws2812_pwm_cfg *cfg = dev->config;
    struct ws2812_pwm_data *ctx = dev->data;
    uint8_t i;

    if (!device_is_ready(cfg->pwm.dev)) {
        LOG_ERR("%s: pwm device not ready", cfg->pwm.dev->name);
        return -ENODEV;
    }

    for (i = 0; i < cfg->num_colors; i++) {
        switch (cfg->color_mapping[i]) {
        case LED_COLOR_ID_WHITE:
        case LED_COLOR_ID_RED:
        case LED_COLOR_ID_GREEN:
        case LED_COLOR_ID_BLUE:
            break;
        default:
            LOG_ERR("%s: invalid channel to color mapping."
                    "Check the color-mapping DT property",
                    dev->name);
            return -EINVAL;
        }
    }

    uint64_t cycles_per_sec;
    pwm_get_cycles_per_sec(cfg->pwm.dev, cfg->pwm.channel, &cycles_per_sec);

    ctx->pwm_period_cycles = (uint32_t)((cfg->period_ns * cycles_per_sec) / NSEC_PER_SEC);
    ctx->pwm_t0h_cycles = (uint32_t)((cfg->t0h_ns * cycles_per_sec) / NSEC_PER_SEC);
    ctx->pwm_t1h_cycles = (uint32_t)((cfg->t1h_ns * cycles_per_sec) / NSEC_PER_SEC);

    return 0;
}

static const struct led_strip_driver_api ws2812_pwm_api = {
    .update_rgb = ws2812_strip_update_rgb,
    .update_channels = ws2812_strip_update_channels,
};

#define PWM(idx)                                                                                   \
    {                                                                                              \
        .dev = DEVICE_DT_GET(DT_INST_PWMS_CTLR_BY_IDX(idx, 0)),                                    \
        .channel = DT_INST_PWMS_CHANNEL_BY_IDX(idx, 0),                                            \
        .period = DT_INST_PWMS_PERIOD_BY_IDX(idx, 0), .flags = DT_INST_PWMS_FLAGS_BY_IDX(idx, 0),  \
    }

#define WS2812_NUM_COLORS(idx) (DT_INST_PROP_LEN(idx, color_mapping))
#define WS2812_PWM_NUM_PIXELS(idx) (DT_INST_PROP(idx, chain_length))
#define WS2812_PWM_BUFSZ(idx) (WS2812_NUM_COLORS(idx) * WS2812_PWM_NUM_PIXELS(idx))
#define WS2812_PWM_T0H_NS(idx) (DT_INST_PROP(idx, t0h_ns))
#define WS2812_PWM_T1H_NS(idx) (DT_INST_PROP(idx, t1h_ns))
#define WS2812_PWM_PERIOD_NS(idx) (DT_INST_PROP(idx, period_ns))

/*
 * Retrieve the channel to color mapping (e.g. RGB, BGR, GRB, ...) from the
 * "color-mapping" DT property.
 */
#define WS2812_COLOR_MAPPING(idx)                                                                  \
    static const uint8_t ws2812_pwm_##idx##_color_mapping[] = DT_INST_PROP(idx, color_mapping)

/* Get the latch/reset delay from the "reset-delay" DT property. */
#define WS2812_RESET_DELAY(idx) DT_INST_PROP(idx, reset_delay)

#define WS2812_PWM_DEVICE(idx)                                                                     \
                                                                                                   \
    WS2812_COLOR_MAPPING(idx);                                                                     \
    static uint8_t ws2812_pwm_##idx##_px_buf[WS2812_PWM_BUFSZ(idx)];                               \
                                                                                                   \
    static struct ws2812_pwm_data ws2812_pwm##idx##_data;                                          \
                                                                                                   \
    static const struct ws2812_pwm_cfg ws2812_pwm_##idx##_cfg = {                                  \
        .pwm = PWM(idx),                                                                           \
        .px_buf = ws2812_pwm_##idx##_px_buf,                                                       \
        .px_buf_size = WS2812_PWM_BUFSZ(idx),                                                      \
        .num_colors = WS2812_NUM_COLORS(idx),                                                      \
        .color_mapping = ws2812_pwm_##idx##_color_mapping,                                         \
        .t0h_ns = WS2812_PWM_T0H_NS(idx),                                                          \
        .t1h_ns = WS2812_PWM_T1H_NS(idx),                                                          \
        .period_ns = WS2812_PWM_PERIOD_NS(idx),                                                    \
        .reset_delay = WS2812_RESET_DELAY(idx),                                                    \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(idx, ws2812_pwm_init, NULL, &ws2812_pwm##idx##_data,                     \
                          &ws2812_pwm_##idx##_cfg, POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY,    \
                          &ws2812_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_PWM_DEVICE)
