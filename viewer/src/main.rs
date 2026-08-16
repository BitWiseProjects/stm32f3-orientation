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
//! | [`cloud`] | Where a sample in nanotesla lands on screen |
//! | [`scene`] | The cameras, the model, the point cloud, the lights, the colours |
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
//!
//! The one thing the board does *not* decide is when the calibration view goes
//! away. It stays up after the run so the answer can be seen against the cloud
//! it came from, and Escape is what puts the model back.

mod cloud;
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

use dom::{set_connect_enabled, set_readout, set_status};
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
    let mut calibration = scene::CalibrationScene::new(&context, window.viewport());
    let link = Rc::new(RefCell::new(Link::default()));

    wire_connect_button(link.clone());
    let keys = dom::watch_keys();

    let mut last_reported = String::new();
    let mut last_readout = (None, None);
    let mut last_mode = Mode::Model;

    window.render_loop(move |mut frame_input| {
        let mut state = link.borrow_mut();

        // The same wall clock `serial` stamps packets with. This is also the
        // only thing that ends a run the board stopped talking in the middle
        // of, so it runs before anything reads the mode.
        state.run.tick(js_sys::Date::now());

        let orientation = state.orientation.unwrap_or(glam::Quat::IDENTITY);

        // Escape dismisses a landed run; space compares the corrected cloud
        // against the raw one. Read before the mode is, so a dismissal takes
        // effect on the frame it happened rather than the next one.
        let keys = dom::take_keys(&keys);
        if keys.dismiss {
            state.run.dismiss();
        }
        // Nothing to compare against until an offset was adopted, and a
        // refusal's packet holds the board's *old* one.
        if keys.compare && state.run.adopted_offset().is_some() {
            calibration.set_snap(!calibration.snapped());
        }

        let mode = state.run.mode();

        // A run starting is the one moment everything the view accumulated —
        // scale, camera, snap — has to go, and it is the mode edge rather than
        // an empty sample list, because a run's first packet already has one.
        if mode == Mode::Calibrating && last_mode != Mode::Calibrating {
            calibration.begin();
        }
        // And a run landing on an adopted offset plays the snap by itself. It
        // is the answer arriving; waiting for a keypress to show it would make
        // the payoff of the whole run something you have to know to ask for.
        if mode == Mode::Landed && last_mode == Mode::Calibrating {
            calibration.set_snap(state.run.adopted_offset().is_some());
        }
        last_mode = mode;

        // The run owns the status line whenever it has something to say; the
        // packet counter is what is left the rest of the time.
        let status = state
            .run
            .status()
            .unwrap_or_else(|| format!("{} packets · {} dropped", state.good, state.bad));
        if status != last_reported {
            last_reported.clone_from(&status);
            set_status(&status);
        }

        let readout = (state.run.metrics(), state.run.constant_line());
        if readout != last_readout {
            last_readout = readout;
            set_readout(last_readout.0.as_deref(), last_readout.1.as_deref());
        }

        match mode {
            Mode::Model => {
                drop(state);
                scene.draw(&frame_input, orientation);
            }

            // `Landed` keeps the cloud. The offset is only meaningful as a
            // distance from this cloud to the origin, so the two have to be on
            // screen together — and the snap, which is the payoff of the whole
            // run, has nothing to move if the cloud has already gone.
            Mode::Calibrating | Mode::Landed => {
                calibration.draw(
                    &mut frame_input,
                    state.run.samples(),
                    state.run.latest(),
                    state.run.adopted_offset(),
                );
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
