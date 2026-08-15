//! The port, and what comes out of it.
//!
//! # Chrome or Edge only
//!
//! Web Serial does not exist in Firefox or Safari. It also requires a secure
//! context, which `localhost` counts as, so no certificate is needed here.
//!
//! The port picker is native browser UI and it only opens in response to a
//! real click. That is why there is a Connect button rather than an attempt to
//! connect on load — a page cannot reach for a serial port on its own, which
//! is the whole point of the permission model.

use std::cell::RefCell;
use std::rc::Rc;

use packet::Packet;
use packet::glam;
use wasm_bindgen::{JsCast, JsValue};
use wasm_bindgen_futures::JsFuture;

use crate::dom::set_status;
use crate::parser::Parser;
use crate::run::Run;

/// Must match the firmware. See `firmware/src/bin/05_serial.rs`.
const BAUD: u32 = 115_200;

/// What the serial task hands to the render loop.
///
/// The two sides run at different rates and neither waits for the other, so
/// this holds the latest of everything rather than a queue — with one
/// deliberate exception, noted on [`run`](Self::run).
#[derive(Default)]
pub struct Link {
    /// The newest orientation. Deliberately only the newest — this is a
    /// display, and drawing an orientation the board has already left would be
    /// worse than skipping it.
    pub orientation: Option<glam::Quat>,
    /// The calibration run, which is the exception: it accumulates.
    ///
    /// It has to be fed here rather than polled from the render loop, because
    /// the packet that ends a run is sent **once** and the board is back to its
    /// 2 Hz idle heartbeat twenty milliseconds later. A render loop that looked
    /// at "the newest calibration packet" sixty times a second would miss the
    /// answer about half the time, and the failure would look like the board
    /// never solving.
    pub run: Run,
    pub good: u32,
    pub bad: u32,
}

/// Open a port the user picked and read from it until it closes.
pub async fn read_from_port(link: Rc<RefCell<Link>>) -> Result<(), JsValue> {
    let window = web_sys::window().ok_or_else(|| JsValue::from_str("no window"))?;
    let serial = window.navigator().serial();

    set_status("choose the ST-LINK's port…");

    // Opens the browser's own port picker. Rejects if the user cancels, which
    // is not an error worth shouting about.
    let port: web_sys::SerialPort = JsFuture::from(serial.request_port()).await?.dyn_into()?;

    let options = web_sys::SerialOptions::new(BAUD);
    JsFuture::from(port.open(&options)).await?;

    set_status("connected, waiting for packets…");

    let reader: web_sys::ReadableStreamDefaultReader = port.readable().get_reader().dyn_into()?;

    let mut parser = Parser::default();

    loop {
        let result = JsFuture::from(reader.read()).await?;

        let done = js_sys::Reflect::get(&result, &JsValue::from_str("done"))?
            .as_bool()
            .unwrap_or(false);
        if done {
            break;
        }

        let value = js_sys::Reflect::get(&result, &JsValue::from_str("value"))?;
        let chunk = js_sys::Uint8Array::new(&value).to_vec();

        // The same wall clock the render loop reads. They are compared against
        // each other, so a monotonic timer in one and a wall clock in the other
        // would subtract unrelated numbers.
        let now = js_sys::Date::now();

        let mut state = link.borrow_mut();
        for decoded in parser.feed(&chunk) {
            match decoded {
                Packet::Orientation(orientation) => {
                    state.orientation = Some(orientation.rotation);
                }
                Packet::Calibration(calibration) => {
                    state.run.push(calibration, now);
                }
            }
        }
        state.good = parser.good;
        state.bad = parser.bad;
    }

    // A run still going when the port shuts is over, and there is no point
    // waiting out the silence timer to conclude it.
    link.borrow_mut().run.disconnected();

    set_status("port closed");
    Ok(())
}
