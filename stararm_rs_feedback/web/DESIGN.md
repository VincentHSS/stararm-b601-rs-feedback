# Star Arm 102 interface

The 0.6.0 interface uses white surfaces, black primary type/actions, and blue
selection and connection feedback. Amber and red indicate torque magnitude or
exceptional states. The Fashionstar brand mark retains its supplied colors.

Interaction code in `motion.js` is original native JavaScript. Design references:

- [beUI](https://beui.dev/) — shared selection indicators and tactile buttons
- [Transitions.dev](https://transitions.dev/) — layout resizing, content swaps and panel exits
- [shadcn/ui](https://ui.shadcn.com/) — restrained component hierarchy
- [Rare UI](https://rareui.com/) — interaction reference

No paid component source is bundled. The UI does not require React, a build step,
an external font, a remote icon CDN, or an online component service at runtime.

`icons.svg` contains selected SVG paths from the official Lucide repository:
<https://github.com/lucide-icons/lucide/tree/main/icons>
Its ISC/MIT notices are in `vendor/LICENSE-lucide.txt`.
The supplied Fashionstar mark has its own notice in `brand/NOTICE.txt`.

Motion uses a consistent 160–420 ms timing range and a common ease-out curve.
The active tab and metric indicator retain their identity while moving and resizing.
Full-view navigation uses a Web Animations layout morph that keeps real controls
interactive while the outgoing panel exits. Fine-grained content replacement can
use the native View Transition API, with the layout morph as its fallback. Interrupted transitions
clean up outgoing inert panels and preserve the latest requested view.
Accordions, logs, notices, labels, loading surfaces and button content have entry
and exit lifecycles. Buttons scale slightly on hover and press. Tooltip surfaces
appear on pointer or keyboard focus. The selected chart supports pointer inspection
and left/right arrow keys; Escape dismisses the readout.

Live state text only animates when it changes. Torque numbers remain the latest
available reported values rather than animated numerical estimates. Invalidation
immediately clears graph values; it does not wait for an exit animation. The 3D
pose, label layout, camera reset and material color changes interpolate visually
without changing any robot command or control-loop behavior.

All motion primitives respect `prefers-reduced-motion`. Tab navigation supports
arrow keys and Home/End; outgoing and hidden controls do not accept input.
