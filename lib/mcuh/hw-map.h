// included from "mcu.h"

constexpr auto RTC     = io32<0x4000'2800>;
constexpr auto IWDG    = io32<0x4000'3000>;
constexpr auto PWR     = io32<0x4000'7000>;

#if STM32F4

constexpr auto EXTI    = io32<0x4001'3C00>;
constexpr auto GPIO    = io32<0x4002'0000>;
constexpr auto RCC     = io32<0x4002'3800>;
constexpr auto FLASH   = io32<0x4002'3C00>;

#elif STM32L0

constexpr auto EXTI    = io32<0x4001'0400>;
constexpr auto RCC     = io32<0x4002'1000>;
constexpr auto FLASH   = io32<0x4002'2000>;
constexpr auto GPIO    = io32<0x5000'0000>;

#elif STM32L4

constexpr auto EXTI    = io32<0x4001'0400>;
constexpr auto RCC     = io32<0x4002'1000>;
constexpr auto FLASH   = io32<0x4002'2000>;
constexpr auto GPIO    = io32<0x4800'0000>;

#endif

constexpr auto DWT     = io32<0xE000'1000>;
constexpr auto SCB     = io32<0xE000'E000>;
constexpr auto NVIC    = io32<0xE000'E100>;
