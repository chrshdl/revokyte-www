# www.revokyte.com

Built output for the [Revokyte](https://www.revokyte.com) site, published
here by CI. There is no source in this repo: every file is generated, and
the branch is replaced wholesale on each deploy, so pull requests and
issues against it have nothing to act on. The source is private.

The one hand written file is `.github/workflows/pages.yml`, which hands
the output to GitHub Pages. It arrives with each deploy like everything
else, so editing it here does not survive.

`shiftlights.js` is the exception worth knowing about. It is a JavaScript
port of the instrument cluster's shift light ECU, derived from
[chrshdl/revokyte](https://github.com/chrshdl/revokyte) and covered by
GPL-3.0-or-later. It is served unminified so that the corresponding source
stays available, as that license requires. The rest of the site is
© 2025-2026 Christian Hedel, all rights reserved.
