//! Кинематические схемы станка: перевод координат эффектора (мм) в
//! координаты логических осей моторов и обратно.
//!
//! Планировщик ([`crate::motion::planner`]) всегда оперирует координатами
//! эффектора в пространстве принтера (`CartesianPosition`), а генератор
//! шагов — координатами логических осей (`AxisPosition`, отображаются на
//! `AxisId::X`/`Y`/`Z` из `motion.toml` независимо от того, что физически
//! означает каждая ось при данной кинематике). Смена схемы — это замена
//! реализации [`Kinematics`] в точке создания `MotionPlanner`, без изменений
//! в планировщике, генераторе шагов или G-Code.

use crate::error::{AppError, AppResult};

/// Положение эффектора в декартовом пространстве станка, миллиметры.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CartesianPosition {
    /// Координата X.
    pub x: f32,
    /// Координата Y.
    pub y: f32,
    /// Координата Z.
    pub z: f32,
}

/// Положение трёх логических осей моторов, миллиметры (при неевклидовых
/// кинематиках — не координаты в пространстве, а "путь", пройденный каждым
/// мотором в мм-эквиваленте, к которому применяется `steps_per_mm`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisPosition {
    /// Положение мотора, сопоставленного с `AxisId::X`.
    pub a: f32,
    /// Положение мотора, сопоставленного с `AxisId::Y`.
    pub b: f32,
    /// Положение мотора, сопоставленного с `AxisId::Z`.
    pub c: f32,
}

/// Кинематическая схема станка.
pub trait Kinematics: Send {
    /// Переводит координаты эффектора в положения логических осей моторов.
    ///
    /// Может завершиться ошибкой (например, [`AppError::Motion`]), если
    /// запрошенная точка физически недостижима при данной геометрии
    /// (актуально для [`DeltaKinematics`] — точки вне рабочей сферы).
    fn cartesian_to_motor(&self, position: CartesianPosition) -> AppResult<AxisPosition>;

    /// Обратное преобразование: положения моторов → координаты эффектора.
    fn motor_to_cartesian(&self, position: AxisPosition) -> AppResult<CartesianPosition>;

    /// Человекочитаемое имя схемы (для логов и `M115`).
    fn name(&self) -> &'static str;
}

/// Декартова кинематика: оси X/Y/Z независимы, преобразование тождественно.
#[derive(Debug, Clone, Copy, Default)]
pub struct CartesianKinematics;

impl Kinematics for CartesianKinematics {
    fn cartesian_to_motor(&self, position: CartesianPosition) -> AppResult<AxisPosition> {
        Ok(AxisPosition {
            a: position.x,
            b: position.y,
            c: position.z,
        })
    }

    fn motor_to_cartesian(&self, position: AxisPosition) -> AppResult<CartesianPosition> {
        Ok(CartesianPosition {
            x: position.a,
            y: position.b,
            z: position.c,
        })
    }

    fn name(&self) -> &'static str {
        "cartesian"
    }
}

/// Кинематика CoreXY: два мотора (A, B) совместно управляют X и Y через
/// перекрёстную ремённую передачу; Z независима.
///
/// `a = x + y`, `b = x - y`; обратное преобразование — `x = (a+b)/2`,
/// `y = (a-b)/2`.
#[derive(Debug, Clone, Copy, Default)]
pub struct CoreXyKinematics;

impl Kinematics for CoreXyKinematics {
    fn cartesian_to_motor(&self, position: CartesianPosition) -> AppResult<AxisPosition> {
        Ok(AxisPosition {
            a: position.x + position.y,
            b: position.x - position.y,
            c: position.z,
        })
    }

    fn motor_to_cartesian(&self, position: AxisPosition) -> AppResult<CartesianPosition> {
        Ok(CartesianPosition {
            x: (position.a + position.b) / 2.0,
            y: (position.a - position.b) / 2.0,
            z: position.c,
        })
    }

    fn name(&self) -> &'static str {
        "core_xy"
    }
}

/// Кинематика CoreXZ: два мотора (A, B) совместно управляют X и Z; Y
/// независима (используется в станках с фиксированным по высоте порталом).
///
/// `a = x + z`, `b = x - z`; обратное преобразование — `x = (a+b)/2`,
/// `z = (a-b)/2`. Независимая ось Y хранится в поле `c`.
#[derive(Debug, Clone, Copy, Default)]
pub struct CoreXzKinematics;

impl Kinematics for CoreXzKinematics {
    fn cartesian_to_motor(&self, position: CartesianPosition) -> AppResult<AxisPosition> {
        Ok(AxisPosition {
            a: position.x + position.z,
            b: position.x - position.z,
            c: position.y,
        })
    }

    fn motor_to_cartesian(&self, position: AxisPosition) -> AppResult<CartesianPosition> {
        Ok(CartesianPosition {
            x: (position.a + position.b) / 2.0,
            y: position.c,
            z: (position.a - position.b) / 2.0,
        })
    }

    fn name(&self) -> &'static str {
        "core_xz"
    }
}

/// Геометрия дельта-принтера, необходимая для прямой и обратной кинематики.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeltaGeometry {
    /// Длина диагональной штанги (от каретки до эффектора), мм.
    pub diagonal_rod_mm: f32,
    /// Радиус окружности башен (от центра станка до вертикальной
    /// направляющей), мм.
    pub base_radius_mm: f32,
    /// Радиус окружности крепления штанг на эффекторе, мм.
    pub effector_radius_mm: f32,
    /// Индивидуальная угловая коррекция башен A/B/C относительно
    /// номинальных 210°/330°/90°, градусы (калибровочная поправка).
    pub tower_angle_correction_deg: [f32; 3],
}

impl Default for DeltaGeometry {
    fn default() -> Self {
        Self {
            diagonal_rod_mm: 250.0,
            base_radius_mm: 150.0,
            effector_radius_mm: 30.0,
            tower_angle_correction_deg: [0.0, 0.0, 0.0],
        }
    }
}

/// Номинальные углы башен A/B/C (в градусах, против часовой стрелки от
/// положительного направления X), общепринятые для RepRap-дельт.
const NOMINAL_TOWER_ANGLES_DEG: [f32; 3] = [210.0, 330.0, 90.0];

/// Кинематика линейной дельты. Обратная кинематика (координаты эффектора →
/// высота каретки на каждой башне) — точная формула геометрии штанги
/// постоянной длины. Прямая кинематика (высоты кареток → координаты
/// эффектора) решается численно методом Ньютона — это не находится на
/// горячем пути генератора шагов (нужно только для отчёта `M114` и
/// диагностики), поэтому предпочтена устойчивость и простота проверки
/// корректности перед аналитической трилатерацией в закрытой форме.
#[derive(Debug, Clone, Copy)]
pub struct DeltaKinematics {
    geometry: DeltaGeometry,
    tower_xy: [(f32, f32); 3],
}

impl DeltaKinematics {
    /// Создаёт кинематику из геометрии станка, предвычисляя координаты
    /// оснований башен.
    #[must_use]
    pub fn new(geometry: DeltaGeometry) -> Self {
        let delta_radius = geometry.base_radius_mm - geometry.effector_radius_mm;
        let mut tower_xy = [(0.0f32, 0.0f32); 3];
        for i in 0..3 {
            let angle_deg = NOMINAL_TOWER_ANGLES_DEG[i] + geometry.tower_angle_correction_deg[i];
            let angle_rad = angle_deg.to_radians();
            tower_xy[i] = (delta_radius * angle_rad.cos(), delta_radius * angle_rad.sin());
        }
        Self { geometry, tower_xy }
    }

    /// Высота каретки на башне `tower_index` для точки эффектора `xyz`, мм.
    fn carriage_height(&self, tower_index: usize, x: f32, y: f32, z: f32) -> AppResult<f32> {
        let (tx, ty) = self.tower_xy[tower_index];
        let dx = x - tx;
        let dy = y - ty;
        let rod_sq = self.geometry.diagonal_rod_mm * self.geometry.diagonal_rod_mm;
        let horizontal_sq = dx * dx + dy * dy;
        let under_sqrt = rod_sq - horizontal_sq;
        if under_sqrt < 0.0 {
            return Err(AppError::Motion(format!(
                "точка ({x:.2}, {y:.2}, {z:.2}) вне досягаемости дельта-кинематики (башня {tower_index}: штанга не достаёт)"
            )));
        }
        Ok(z + under_sqrt.sqrt())
    }

    /// Невязка между заданными высотами кареток `target` и высотами,
    /// которые давала бы точка `(x, y, z)`.
    fn residual(&self, x: f32, y: f32, z: f32, target: [f32; 3]) -> AppResult<[f32; 3]> {
        let mut out = [0.0f32; 3];
        for i in 0..3 {
            out[i] = self.carriage_height(i, x, y, z)? - target[i];
        }
        Ok(out)
    }
}

impl Kinematics for DeltaKinematics {
    fn cartesian_to_motor(&self, position: CartesianPosition) -> AppResult<AxisPosition> {
        Ok(AxisPosition {
            a: self.carriage_height(0, position.x, position.y, position.z)?,
            b: self.carriage_height(1, position.x, position.y, position.z)?,
            c: self.carriage_height(2, position.x, position.y, position.z)?,
        })
    }

    fn motor_to_cartesian(&self, position: AxisPosition) -> AppResult<CartesianPosition> {
        let target = [position.a, position.b, position.c];

        // Начальное приближение: центр станка по X/Y, высота — среднее
        // значение высот кареток за вычетом половины длины штанги
        // (разумная отправная точка для сходимости Ньютона).
        let mean_height = (target[0] + target[1] + target[2]) / 3.0;
        let (mut x, mut y, mut z) = (0.0f32, 0.0f32, mean_height - self.geometry.diagonal_rod_mm * 0.5);

        const MAX_ITERATIONS: usize = 20;
        const STEP_EPSILON: f32 = 1e-4;
        const CONVERGENCE_MM: f32 = 1e-5;

        for _ in 0..MAX_ITERATIONS {
            let r0 = self.residual(x, y, z, target)?;

            // Численный Якобиан методом конечных разностей (3x3).
            let rx = self.residual(x + STEP_EPSILON, y, z, target)?;
            let ry = self.residual(x, y + STEP_EPSILON, z, target)?;
            let rz = self.residual(x, y, z + STEP_EPSILON, target)?;

            let jacobian = [
                [(rx[0] - r0[0]) / STEP_EPSILON, (ry[0] - r0[0]) / STEP_EPSILON, (rz[0] - r0[0]) / STEP_EPSILON],
                [(rx[1] - r0[1]) / STEP_EPSILON, (ry[1] - r0[1]) / STEP_EPSILON, (rz[1] - r0[1]) / STEP_EPSILON],
                [(rx[2] - r0[2]) / STEP_EPSILON, (ry[2] - r0[2]) / STEP_EPSILON, (rz[2] - r0[2]) / STEP_EPSILON],
            ];

            let delta = solve_3x3(jacobian, r0).ok_or_else(|| {
                AppError::Motion("вырожденный якобиан при решении прямой кинематики дельты".to_string())
            })?;

            x -= delta[0];
            y -= delta[1];
            z -= delta[2];

            if delta[0].abs() < CONVERGENCE_MM && delta[1].abs() < CONVERGENCE_MM && delta[2].abs() < CONVERGENCE_MM {
                break;
            }
        }

        Ok(CartesianPosition { x, y, z })
    }

    fn name(&self) -> &'static str {
        "delta"
    }
}

/// Решает линейную систему `matrix * x = rhs` методом Гаусса с выбором
/// главного элемента. Возвращает `None`, если матрица вырождена.
fn solve_3x3(matrix: [[f32; 3]; 3], rhs: [f32; 3]) -> Option<[f32; 3]> {
    let mut aug = [
        [matrix[0][0], matrix[0][1], matrix[0][2], rhs[0]],
        [matrix[1][0], matrix[1][1], matrix[1][2], rhs[1]],
        [matrix[2][0], matrix[2][1], matrix[2][2], rhs[2]],
    ];

    for col in 0..3 {
        let pivot_row = (col..3).max_by(|&a, &b| aug[a][col].abs().partial_cmp(&aug[b][col].abs()).unwrap())?;
        if aug[pivot_row][col].abs() < 1e-9 {
            return None;
        }
        aug.swap(col, pivot_row);

        for row in 0..3 {
            if row == col {
                continue;
            }
            let factor = aug[row][col] / aug[col][col];
            for k in col..4 {
                aug[row][k] -= factor * aug[col][k];
            }
        }
    }

    Some([aug[0][3] / aug[0][0], aug[1][3] / aug[1][1], aug[2][3] / aug[2][2]])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cartesian_kinematics_is_identity() {
        let k = CartesianKinematics;
        let pos = CartesianPosition { x: 10.0, y: 20.0, z: 5.0 };
        let motor = k.cartesian_to_motor(pos).unwrap();
        assert_eq!((motor.a, motor.b, motor.c), (10.0, 20.0, 5.0));
        let back = k.motor_to_cartesian(motor).unwrap();
        assert_eq!((back.x, back.y, back.z), (10.0, 20.0, 5.0));
    }

    #[test]
    fn core_xy_round_trips() {
        let k = CoreXyKinematics;
        let pos = CartesianPosition { x: 30.0, y: -12.0, z: 7.0 };
        let motor = k.cartesian_to_motor(pos).unwrap();
        let back = k.motor_to_cartesian(motor).unwrap();
        assert!((back.x - pos.x).abs() < 1e-4);
        assert!((back.y - pos.y).abs() < 1e-4);
        assert!((back.z - pos.z).abs() < 1e-4);
    }

    #[test]
    fn core_xz_round_trips() {
        let k = CoreXzKinematics;
        let pos = CartesianPosition { x: 15.0, y: 4.0, z: -6.0 };
        let motor = k.cartesian_to_motor(pos).unwrap();
        let back = k.motor_to_cartesian(motor).unwrap();
        assert!((back.x - pos.x).abs() < 1e-4);
        assert!((back.y - pos.y).abs() < 1e-4);
        assert!((back.z - pos.z).abs() < 1e-4);
    }

    #[test]
    fn delta_inverse_then_forward_round_trips_near_center() {
        let k = DeltaKinematics::new(DeltaGeometry::default());
        let pos = CartesianPosition { x: 5.0, y: -8.0, z: 100.0 };
        let motor = k.cartesian_to_motor(pos).unwrap();
        let back = k.motor_to_cartesian(motor).unwrap();
        assert!((back.x - pos.x).abs() < 1e-2, "x: {} vs {}", back.x, pos.x);
        assert!((back.y - pos.y).abs() < 1e-2, "y: {} vs {}", back.y, pos.y);
        assert!((back.z - pos.z).abs() < 1e-2, "z: {} vs {}", back.z, pos.z);
    }

    #[test]
    fn delta_rejects_point_out_of_reach() {
        let k = DeltaKinematics::new(DeltaGeometry::default());
        let far = CartesianPosition { x: 500.0, y: 500.0, z: 100.0 };
        assert!(k.cartesian_to_motor(far).is_err());
    }
}
