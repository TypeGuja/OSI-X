//! Адреса и типизированные представления регистров TMC2209.
//!
//! Каждый регистр, с которым работает драйвер, представлен отдельной
//! структурой с именованными полями и явными методами `to_u32`/`from_u32` —
//! это исключает магические числа и битовые сдвиги в остальном коде.
//! Битовые раскладки соответствуют официальному datasheet TMC2209
//! (Trinamic/Analog Devices, Rev. 1.09).

/// Адреса регистров TMC2209, используемые драйвером.
pub mod address {
    /// Общая конфигурация (R/W).
    pub const GCONF: u8 = 0x00;
    /// Флаги состояния, сбрасываемые записью `1` (R+WC).
    pub const GSTAT: u8 = 0x01;
    /// Счётчик успешных UART-транзакций записи (R).
    pub const IFCNT: u8 = 0x02;
    /// Конфигурация UART-адреса подчинённого устройства (W).
    pub const SLAVECONF: u8 = 0x03;
    /// Состояние входных пинов драйвера (R).
    pub const IOIN: u8 = 0x06;
    /// Заводская калибровка (R/W), используется для чтения `FCLKTRIM`.
    pub const FACTORY_CONF: u8 = 0x07;
    /// Ток удержания/движения и время задержки (W).
    pub const IHOLD_IRUN: u8 = 0x10;
    /// Время до перехода в режим тока удержания после остановки (W).
    pub const TPOWERDOWN: u8 = 0x11;
    /// Измеренный период между шагами (R), основа для порогов `TPWMTHRS`/`TCOOLTHRS`.
    pub const TSTEP: u8 = 0x12;
    /// Порог скорости перехода StealthChop → SpreadCycle (W).
    pub const TPWMTHRS: u8 = 0x13;
    /// Порог скорости включения CoolStep/StallGuard (W).
    pub const TCOOLTHRS: u8 = 0x14;
    /// Целевая скорость в режиме управления по UART без физического STEP (W).
    pub const VACTUAL: u8 = 0x22;
    /// Порог срабатывания StallGuard (W).
    pub const SGTHRS: u8 = 0x40;
    /// Текущее значение нагрузки StallGuard (R).
    pub const SG_RESULT: u8 = 0x41;
    /// Конфигурация CoolStep (W).
    pub const COOLCONF: u8 = 0x42;
    /// Счётчик текущего микрошага (R).
    pub const MSCNT: u8 = 0x6A;
    /// Текущие значения синус/косинус таблицы микрошага (R).
    pub const MSCURACT: u8 = 0x6B;
    /// Конфигурация чоппера: микрошаг, время выключения, гистерезис (R/W).
    pub const CHOPCONF: u8 = 0x6C;
    /// Диагностическое состояние драйвера (R).
    pub const DRV_STATUS: u8 = 0x6F;
    /// Конфигурация StealthChop PWM (R/W).
    pub const PWMCONF: u8 = 0x70;
    /// Текущее значение амплитуды PWM (R).
    pub const PWM_SCALE: u8 = 0x71;
    /// Автоматически подобранные параметры PWM (R).
    pub const PWM_AUTO: u8 = 0x72;
}

/// Разрешение микрошага TMC2209 (поле `MRES` регистра `CHOPCONF`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MicrostepResolution {
    /// 256 микрошагов на полный шаг.
    Full256,
    /// 128 микрошагов на полный шаг.
    Full128,
    /// 64 микрошага на полный шаг.
    Full64,
    /// 32 микрошага на полный шаг.
    Full32,
    /// 16 микрошагов на полный шаг.
    Full16,
    /// 8 микрошагов на полный шаг.
    Full8,
    /// 4 микрошага на полный шаг.
    Full4,
    /// 2 микрошага на полный шаг (полушаг).
    Full2,
    /// Полный шаг (микрошаг выключен).
    FullStep,
}

impl MicrostepResolution {
    /// Значение поля `MRES` (биты 24..=27 регистра `CHOPCONF`).
    #[must_use]
    pub const fn mres_value(self) -> u8 {
        match self {
            Self::Full256 => 0,
            Self::Full128 => 1,
            Self::Full64 => 2,
            Self::Full32 => 3,
            Self::Full16 => 4,
            Self::Full8 => 5,
            Self::Full4 => 6,
            Self::Full2 => 7,
            Self::FullStep => 8,
        }
    }

    /// Количество микрошагов на один полный шаг двигателя.
    #[must_use]
    pub const fn microsteps_per_step(self) -> u16 {
        match self {
            Self::Full256 => 256,
            Self::Full128 => 128,
            Self::Full64 => 64,
            Self::Full32 => 32,
            Self::Full16 => 16,
            Self::Full8 => 8,
            Self::Full4 => 4,
            Self::Full2 => 2,
            Self::FullStep => 1,
        }
    }

    /// Восстанавливает значение из поля `MRES`. Неизвестные значения (9..15,
    /// зарезервированы производителем) трактуются как полный шаг — наиболее
    /// безопасный вариант по умолчанию.
    #[must_use]
    pub const fn from_mres_value(value: u8) -> Self {
        match value {
            0 => Self::Full256,
            1 => Self::Full128,
            2 => Self::Full64,
            3 => Self::Full32,
            4 => Self::Full16,
            5 => Self::Full8,
            6 => Self::Full4,
            7 => Self::Full2,
            _ => Self::FullStep,
        }
    }
}

/// Битовые позиции полей регистра `GCONF`.
mod gconf_bits {
    pub const I_SCALE_ANALOG: u32 = 0;
    pub const INTERNAL_RSENSE: u32 = 1;
    pub const EN_SPREADCYCLE: u32 = 2;
    pub const SHAFT: u32 = 3;
    pub const INDEX_OTPW: u32 = 4;
    pub const INDEX_STEP: u32 = 5;
    pub const PDN_DISABLE: u32 = 6;
    pub const MSTEP_REG_SELECT: u32 = 7;
    pub const MULTISTEP_FILT: u32 = 8;
}

/// Регистр `GCONF` — общая конфигурация драйвера.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GConf {
    /// Использовать внешний опорный ток через вход `VREF` (аналоговое
    /// масштабирование тока) вместо внутреннего источника.
    pub i_scale_analog: bool,
    /// Использовать внутренний измерительный резистор (не применимо к
    /// TMC2209 в дискретном исполнении — всегда `false`).
    pub internal_rsense: bool,
    /// `true` — SpreadCycle, `false` — StealthChop (тихий режим).
    pub en_spreadcycle: bool,
    /// Программная инверсия направления вращения на уровне регистра.
    pub shaft: bool,
    /// Использовать выход `INDEX` для сигнала перегрева.
    pub index_otpw: bool,
    /// Использовать выход `INDEX` для повторения импульсов `STEP`.
    pub index_step: bool,
    /// Отключить внутреннюю подтяжку `PDN_UART` (обязательно для UART-режима).
    pub pdn_disable: bool,
    /// Выбирать микрошаг из регистра `CHOPCONF.MRES`, а не по пинам `MS1`/`MS2`.
    pub mstep_reg_select: bool,
    /// Включить программную фильтрацию частоты STEP для StealthChop.
    pub multistep_filt: bool,
}

impl Default for GConf {
    /// Конфигурация по умолчанию для управления по UART: аппаратная
    /// подтяжка `PDN_UART` отключена, микрошаг выбирается из `CHOPCONF`.
    fn default() -> Self {
        Self {
            i_scale_analog: false,
            internal_rsense: false,
            en_spreadcycle: false,
            shaft: false,
            index_otpw: false,
            index_step: false,
            pdn_disable: true,
            mstep_reg_select: true,
            multistep_filt: true,
        }
    }
}

impl GConf {
    /// Сериализует регистр в 32-битное значение для записи по UART.
    #[must_use]
    pub fn to_u32(self) -> u32 {
        (u32::from(self.i_scale_analog) << gconf_bits::I_SCALE_ANALOG)
            | (u32::from(self.internal_rsense) << gconf_bits::INTERNAL_RSENSE)
            | (u32::from(self.en_spreadcycle) << gconf_bits::EN_SPREADCYCLE)
            | (u32::from(self.shaft) << gconf_bits::SHAFT)
            | (u32::from(self.index_otpw) << gconf_bits::INDEX_OTPW)
            | (u32::from(self.index_step) << gconf_bits::INDEX_STEP)
            | (u32::from(self.pdn_disable) << gconf_bits::PDN_DISABLE)
            | (u32::from(self.mstep_reg_select) << gconf_bits::MSTEP_REG_SELECT)
            | (u32::from(self.multistep_filt) << gconf_bits::MULTISTEP_FILT)
    }

    /// Разбирает 32-битное значение, прочитанное из регистра, в структуру.
    #[must_use]
    pub fn from_u32(value: u32) -> Self {
        let bit = |pos: u32| value & (1 << pos) != 0;
        Self {
            i_scale_analog: bit(gconf_bits::I_SCALE_ANALOG),
            internal_rsense: bit(gconf_bits::INTERNAL_RSENSE),
            en_spreadcycle: bit(gconf_bits::EN_SPREADCYCLE),
            shaft: bit(gconf_bits::SHAFT),
            index_otpw: bit(gconf_bits::INDEX_OTPW),
            index_step: bit(gconf_bits::INDEX_STEP),
            pdn_disable: bit(gconf_bits::PDN_DISABLE),
            mstep_reg_select: bit(gconf_bits::MSTEP_REG_SELECT),
            multistep_filt: bit(gconf_bits::MULTISTEP_FILT),
        }
    }
}

/// Регистр `GSTAT` — флаги состояния, сбрасываемые записью `1` в
/// соответствующий бит.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GStat {
    /// Произошёл сброс драйвера (питание пропадало или сработал POR).
    pub reset: bool,
    /// Драйвер выключен из-за ошибки (перегрев, короткое замыкание).
    pub drv_err: bool,
    /// Заряда накачки заряда (charge pump) было недостаточно.
    pub uv_cp: bool,
}

impl GStat {
    /// Разбирает значение регистра.
    #[must_use]
    pub fn from_u32(value: u32) -> Self {
        Self {
            reset: value & 0b001 != 0,
            drv_err: value & 0b010 != 0,
            uv_cp: value & 0b100 != 0,
        }
    }

    /// Значение для записи, сбрасывающее все три флага.
    #[must_use]
    pub const fn clear_all() -> u32 {
        0b111
    }
}

/// Регистр `IHOLD_IRUN` — ток удержания, ток движения, время нарастания
/// и спада тока при переходе между ними.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IHoldIRun {
    /// Ток удержания в состоянии покоя, шкала `0..=31`.
    pub ihold: u8,
    /// Ток движения, шкала `0..=31`.
    pub irun: u8,
    /// Задержка перехода на ток удержания после остановки, шкала `0..=15`
    /// (единица — примерно `2^IHOLDDELAY` * 2 мс).
    pub iholddelay: u8,
}

impl IHoldIRun {
    /// Максимальное значение шкалы тока (`CS`, current scale).
    pub const MAX_CURRENT_SCALE: u8 = 31;
    /// Максимальное значение `IHOLDDELAY`.
    pub const MAX_HOLD_DELAY: u8 = 15;

    /// Сериализует регистр (`IHOLD` биты 0-4, `IRUN` биты 8-12,
    /// `IHOLDDELAY` биты 16-19).
    #[must_use]
    pub fn to_u32(self) -> u32 {
        (u32::from(self.ihold.min(Self::MAX_CURRENT_SCALE)))
            | (u32::from(self.irun.min(Self::MAX_CURRENT_SCALE)) << 8)
            | (u32::from(self.iholddelay.min(Self::MAX_HOLD_DELAY)) << 16)
    }
}

/// Регистр `CHOPCONF` — конфигурация чоппера и микрошага.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChopConf {
    /// Время выключения off-time, шкала `0..=15` (`0` выключает драйвер).
    pub toff: u8,
    /// Смещение гистерезиса компаратора, шкала `0..=7` (интерпретация
    /// зависит от `chm`; при `chm = false` — `HSTRT` для SpreadCycle).
    pub hstrt: u8,
    /// Конечное значение гистерезиса, шкала `0..=15`.
    pub hend: u8,
    /// Время блокировки компаратора (`TBL`), шкала `0..=3`.
    pub tbl: u8,
    /// Удвоенный диапазон измерения тока (`VSENSE`): `true` — повышенная
    /// чувствительность (меньший ток при том же `CS`).
    pub vsense: bool,
    /// Разрешение микрошага.
    pub mres: MicrostepResolution,
    /// Интерполяция микрошага до 256 (сглаживание вращения независимо от
    /// установленного `MRES`).
    pub intpol: bool,
}

impl Default for ChopConf {
    fn default() -> Self {
        Self {
            toff: 3,
            hstrt: 5,
            hend: 2,
            tbl: 2,
            vsense: false,
            mres: MicrostepResolution::Full16,
            intpol: true,
        }
    }
}

impl ChopConf {
    /// Сериализует регистр в 32-битное значение.
    #[must_use]
    pub fn to_u32(self) -> u32 {
        u32::from(self.toff & 0b1111)
            | (u32::from(self.hstrt & 0b111) << 4)
            | (u32::from(self.hend & 0b1111) << 7)
            | (u32::from(self.tbl & 0b11) << 15)
            | (u32::from(self.vsense) << 17)
            | (u32::from(self.mres.mres_value()) << 24)
            | (u32::from(self.intpol) << 28)
    }

    /// Разбирает 32-битное значение регистра.
    #[must_use]
    pub fn from_u32(value: u32) -> Self {
        Self {
            toff: (value & 0b1111) as u8,
            hstrt: ((value >> 4) & 0b111) as u8,
            hend: ((value >> 7) & 0b1111) as u8,
            tbl: ((value >> 15) & 0b11) as u8,
            vsense: value & (1 << 17) != 0,
            mres: MicrostepResolution::from_mres_value(((value >> 24) & 0b1111) as u8),
            intpol: value & (1 << 28) != 0,
        }
    }
}

/// Регистр `COOLCONF` — конфигурация CoolStep (адаптивное снижение тока).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoolConf {
    /// Нижний порог нагрузки для увеличения тока, шкала `0..=15`
    /// (`0` отключает CoolStep).
    pub semin: u8,
    /// Шаг увеличения тока при превышении верхнего порога, шкала `0..=3`.
    pub seup: u8,
    /// Верхний порог нагрузки для уменьшения тока, шкала `0..=15`.
    pub semax: u8,
    /// Шаг уменьшения тока, шкала `0..=3`.
    pub sedn: u8,
    /// Минимальный ток CoolStep: `false` — 1/2 `IRUN`, `true` — 1/4 `IRUN`.
    pub seimin: bool,
}

impl Default for CoolConf {
    /// CoolStep выключен (`semin = 0`) до явной настройки.
    fn default() -> Self {
        Self {
            semin: 0,
            seup: 0,
            semax: 0,
            sedn: 0,
            seimin: false,
        }
    }
}

impl CoolConf {
    /// Сериализует регистр в 32-битное значение.
    #[must_use]
    pub fn to_u32(self) -> u32 {
        u32::from(self.semin & 0b1111)
            | (u32::from(self.seup & 0b11) << 5)
            | (u32::from(self.semax & 0b1111) << 8)
            | (u32::from(self.sedn & 0b11) << 13)
            | (u32::from(self.seimin) << 15)
    }
}

/// Регистр `PWMCONF` — конфигурация StealthChop PWM.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmConf {
    /// Начальная амплитуда PWM (используется до автонастройки), `0..=255`.
    pub pwm_ofs: u8,
    /// Градиент автонастройки амплитуды PWM, `0..=255`.
    pub pwm_grad: u8,
    /// Делитель частоты PWM, шкала `0..=3`.
    pub pwm_freq: u8,
    /// Включить автоматическую подстройку амплитуды (`PWM_OFS_AUTO`).
    pub pwm_autoscale: bool,
    /// Включить автоматическую подстройку градиента (`PWM_GRAD_AUTO`).
    pub pwm_autograd: bool,
    /// Ограничение амплитуды регулятора автонастройки, `0..=15`.
    pub pwm_reg: u8,
    /// Порог перехода в режим свободного качения на низких скоростях,
    /// шкала `0..=3`.
    pub freewheel: u8,
}

impl Default for PwmConf {
    /// Рекомендованные производителем значения по умолчанию для StealthChop.
    fn default() -> Self {
        Self {
            pwm_ofs: 36,
            pwm_grad: 14,
            pwm_freq: 1,
            pwm_autoscale: true,
            pwm_autograd: true,
            pwm_reg: 8,
            freewheel: 0,
        }
    }
}

impl PwmConf {
    /// Сериализует регистр в 32-битное значение.
    #[must_use]
    pub fn to_u32(self) -> u32 {
        u32::from(self.pwm_ofs)
            | (u32::from(self.pwm_grad) << 8)
            | (u32::from(self.pwm_freq & 0b11) << 16)
            | (u32::from(self.pwm_autoscale) << 18)
            | (u32::from(self.pwm_autograd) << 19)
            | (u32::from(self.freewheel & 0b11) << 20)
            | (u32::from(self.pwm_reg & 0b1111) << 24)
    }
}
