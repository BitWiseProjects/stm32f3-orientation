//! The other end of the wire.
//!
//! A web page that opens the board's serial port, reads packets out of it, and
//! draws the board where the packets say it is. It is Rust, compiled to
//! WebAssembly, rendering through WebGL2 — and neither the transform that
//! positions the model nor the format of the packets is written here. Both
//! come from crates in `math/`, which the firmware compiles too.
//!
//! ```text
//! trunk serve
//! ```
//!
//! then open <http://localhost:8080> and press Connect.
//!
//! # Where everything is
//!
//! | Module | What it owns |
//! |---|---|
//! | [`serial`] | Opening the port, reading it, and the state it fills |
//! | [`parser`] | Finding whole packets in a stream of bytes |
//! | [`run`] | A calibration run: which view to show, and the cloud so far |
//! | [`scene`] | The camera, the model, the lights, the colours |
//! | [`dom`] | The status line and the Connect button |
//!
//! What a packet *is* lives outside this crate entirely, in `math/packet`, so
//! that the definition the browser reads and the one the firmware writes
//! cannot drift apart.
//!
//! This file is the wiring: build the scene, hang the button off the page,
//! hand control to the render loop.
//!
//! # Joining a stream already in progress
//!
//! The board has been sending packets since it was powered on, so the first
//! bytes that arrive are almost never the start of one. Worse, the ST-LINK
//! buffers what it has been sending while nothing was listening, so a connect
//! typically delivers a lump of stale data and then jumps to live.
//!
//! Neither is a problem to solve, only to expect: the parser hunts for the
//! marker, checks the checksum, and drops anything that fails. Being able to
//! join a stream at an arbitrary point is exactly what the marker is for.
//!
//! # Two views, and the board chooses
//!
//! There is no control on the page for switching to the calibration view. The
//! board announces a run in its packets and the page follows, so pressing the
//! blue USER button is the only action there is — which is what makes it
//! filmable, and what makes older firmware work untouched. Stages 5 and 6 send
//! no calibration packets, so the page simply never leaves the model view.
//!
//! It has to key on the run *state* and not on the packet type: stage 7 has
//! been sending idle calibration packets at 2 Hz since it booted, so anything
//! watching for "a calibration packet" would switch views on connect.

mod dom;
mod parser;
mod run;
mod scene;
mod serial;

use std::cell::RefCell;
use std::rc::Rc;

use packet::glam;
use three_d::{FrameOutput, Window, WindowSettings};
use wasm_bindgen::JsCast;
use wasm_bindgen::closure::Closure;
use wasm_bindgen_futures::spawn_local;

use dom::{set_connect_enabled, set_status};
use run::Mode;
use serial::Link;

fn main() {
    let window = Window::new(WindowSettings {
        title: "stm32f3-orientation".to_string(),
        ..Default::default()
    })
    .unwrap();
    let context = window.gl();

    let mut scene = scene::Scene::new(&context, window.viewport());
    let link = Rc::new(RefCell::new(Link::default()));

    wire_connect_button(link.clone());

    let mut last_reported = String::new();

    window.render_loop(move |frame_input| {
        let mut state = link.borrow_mut();

        // The same wall clock `serial` stamps packets with. This is also the
        // only thing that ends a run the board stopped talking in the middle
        // of, so it runs before anything reads the mode.
        state.run.tick(js_sys::Date::now());

        let orientation = state.orientation.unwrap_or(glam::Quat::IDENTITY);
        let mode = state.run.mode();

        // The run owns the status line whenever it has something to say; the
        // packet counter is what is left the rest of the time.
        let status = state.run.status().unwrap_or_else(|| {
            format!("{} packets · {} dropped", state.good, state.bad)
        });
        if status != last_reported {
            last_reported.clone_from(&status);
            set_status(&status);
        }

        match mode {
            // `Landed` shows the model again, with the answer in the status
            // line. The model snapping true is the payoff of the whole run, so
            // the cloud gets out of its way once there is nothing left to
            // collect.
            Mode::Model | Mode::Landed => {
                drop(state);
                scene.draw(&frame_input, orientation);
            }
            Mode::Calibrating => {
                scene::draw_calibration(&frame_input, state.run.samples());
            }
        }

        FrameOutput::default()
    });
}

/// Hang the port picker off the Connect button, before the render loop takes
/// over and never returns.
fn wire_connect_button(link: Rc<RefCell<Link>>) {
    let document = web_sys::window().unwrap().document().unwrap();
    let button = document.get_element_by_id("connect").unwrap();

    let on_click = Closure::<dyn FnMut()>::new(move || {
        let link = link.clone();
        set_connect_enabled(false);

        spawn_local(async move {
            if let Err(error) = serial::read_from_port(link).await {
                // Cancelling the picker lands here too, so this is a status
                // line rather than anything more dramatic.
                set_status(&format!("not connected — {error:?}"));
            }

            set_connect_enabled(true);
        });
    });

    button
        .add_event_listener_with_callback("click", on_click.as_ref().unchecked_ref())
        .unwrap();

    // The closure has to outlive this scope; the page owns it now.
    on_click.forget();
}
