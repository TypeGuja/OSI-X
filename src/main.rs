//! OSIX Firmware — точка входа.
//!
//! Весь функционал вынесен в библиотечные модули (`app`, `board`, `config`,
//! ...); `main` лишь выполняет обязательные для `esp-idf-sys` шаги
//! инициализации рантайма и делегирует управление структуре [`app::App`].

mod app;
mod board;
mod config;
mod drivers;
mod endstops;
mod error;
mod gcode;
mod hal_adapters;
mod hardware_build;
mod logger;
mod motion;
mod network;
mod printer_state;
mod scheduler;
mod sdcard;
mod storage;
mod system;
mod temperature;
mod types;
mod usb;

fn main() {
    // Обязательный вызов для `esp-idf-sys`: патчит некоторые символы libc,
    // необходимые для корректной работы стандартной библиотеки Rust поверх
    // ESP-IDF. Должен быть первой строкой `main`.
    esp_idf_sys::link_patches();

    if let Err(err) = run() {
        // На этом этапе логгер может быть ещё не инициализирован (ошибка
        // могла произойти до `logger::init`), поэтому дублируем сообщение
        // в стандартный вывод.
        eprintln!("критическая ошибка запуска OSIX Firmware: {err}");
        log::error!("критическая ошибка запуска OSIX Firmware: {err}");
        std::process::exit(1);
    }
}

/// Собирает и запускает приложение, пробрасывая ошибки инициализации наверх.
fn run() -> error::AppResult<()> {
    let mut app = app::App::new()?;
    app.run()
}
