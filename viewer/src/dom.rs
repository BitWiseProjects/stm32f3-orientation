//! The bits of page this app touches.
//!
//! Everything else about the page lives in `index.html`. Keeping the reaching
//! into the DOM here means the rest of the crate never has to unwrap its way
//! down from `window` to an element.

use std::cell::Cell;
use std::rc::Rc;

use wasm_bindgen::JsCast;
use wasm_bindgen::closure::Closure;

/// Keys pressed since the last frame looked.
///
/// # Why these do not come from the render loop's own events, and why capture
///
/// Two separate problems, and each one alone leaves the keys half working.
///
/// `three-d` only reports key presses that reached the canvas, and the canvas
/// only gets them while it holds focus — which it loses to the first button
/// anyone clicks. Connect is a button. Copy constant is a button. So after the
/// one click this page exists to receive, space went to the focused button
/// instead: pressing Copy again, or reopening the serial port picker.
///
/// Moving to a listener on the page fixes that and immediately hits the other
/// half: winit calls `stopPropagation` on the canvas, so while the canvas *does*
/// have focus a bubbling listener never sees the key at all. Measured in the
/// browser — a capture listener saw `Escape` on the canvas and a bubble
/// listener on `document` saw nothing.
///
/// So: **on the window, in the capture phase**, which runs before the event
/// reaches the canvas and therefore before winit can stop it. That is the only
/// arrangement where both keys work with focus anywhere on the page.
#[derive(Clone, Copy, Default)]
pub struct Keys {
    /// Space: show the corrected cloud against the raw one.
    pub compare: bool,
    /// Escape: put the board back.
    pub dismiss: bool,
}

/// Start watching for the two shortcut keys. The handle is shared with the
/// render loop, which takes what it finds and clears it.
pub fn watch_keys() -> Rc<Cell<Keys>> {
    let keys = Rc::new(Cell::new(Keys::default()));

    let Some(window) = web_sys::window() else {
        return keys;
    };

    let sink = keys.clone();
    let on_key =
        Closure::<dyn FnMut(web_sys::KeyboardEvent)>::new(move |event: web_sys::KeyboardEvent| {
            let mut pending = sink.get();
            match event.key().as_str() {
                " " | "Spacebar" => pending.compare = true,
                "Escape" | "Esc" => pending.dismiss = true,
                _ => return,
            }
            // Space scrolls the page and activates whatever button has focus.
            // Both are the wrong thing here, and stopping them is the other
            // half of the fix.
            event.prevent_default();
            sink.set(pending);
        });

    // The `true` is the capture phase, and it is the whole fix.
    let _ = window.add_event_listener_with_callback_and_bool(
        "keydown",
        on_key.as_ref().unchecked_ref(),
        true,
    );

    // The page owns the closure now.
    on_key.forget();

    keys
}

/// Read the pending presses and clear them, so one press is one action.
pub fn take_keys(keys: &Rc<Cell<Keys>>) -> Keys {
    keys.replace(Keys::default())
}

/// Write the line under the canvas. Silently does nothing if the element is
/// missing, which is the right call — a missing status line is not a reason to
/// stop drawing the board.
pub fn set_status(text: &str) {
    if let Some(element) = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.get_element_by_id("status"))
    {
        element.set_text_content(Some(text));
    }
}

/// The numbers beside the canvas, and the line to paste into the firmware.
///
/// `None` for either hides that part: the readout is not on screen at all
/// between runs, and the constant only appears once there is one worth pasting.
///
/// Text, deliberately, and not drawn into the canvas. Rendering readable text
/// in WebGL means shipping a font atlas and a layout engine to say six numbers
/// the browser already knows how to set — and the constant has to be selectable
/// and copyable, which a canvas cannot do at all.
pub fn set_readout(metrics: Option<&str>, constant: Option<&str>) {
    let Some(document) = web_sys::window().and_then(|w| w.document()) else {
        return;
    };

    if let Some(element) = document.get_element_by_id("metrics") {
        element.set_text_content(metrics);
    }
    if let Some(element) = document.get_element_by_id("constant") {
        element.set_text_content(constant);
    }
    for (id, shown) in [("readout", metrics.is_some()), ("copy", constant.is_some())] {
        if let Some(element) = document.get_element_by_id(id) {
            let _ = element.set_attribute("style", if shown { "" } else { "display: none" });
        }
    }
}

/// Grey the Connect button out while a connection is live.
///
/// One connection at a time — a second port picker opened while the first is
/// still going leaves two readers on one port.
pub fn set_connect_enabled(enabled: bool) {
    if let Some(button) = web_sys::window()
        .and_then(|w| w.document())
        .and_then(|d| d.get_element_by_id("connect"))
    {
        if enabled {
            let _ = button.remove_attribute("disabled");
        } else {
            let _ = button.set_attribute("disabled", "");
        }
    }
}
