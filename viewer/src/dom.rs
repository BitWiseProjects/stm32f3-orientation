//! The two bits of page this app touches.
//!
//! Everything else about the page lives in `index.html`. Keeping the reaching
//! into the DOM here means the rest of the crate never has to unwrap its way
//! down from `window` to an element.

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
