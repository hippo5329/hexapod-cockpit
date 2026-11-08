#!/bin/sh
# Adds the Hexapod tab (hexa-panels.js) to an installed linorobot2-cockpit web UI.
#   install_panels.sh <cockpit web/frontend dir> [camera URL]
# Copies hexa-panels.js beside cockpit's index.html, loads it before app-core.js (its tab must exist when
# cockpit binds the tab buttons) and, with a camera URL, writes hexa-site.js with the panel's default.
# Idempotent; cockpit's own files are otherwise untouched. Run where the frontend is (in its container).
set -eu
dst=${1:?usage: install_panels.sh <cockpit web/frontend dir> [camera URL]}
here=$(cd "$(dirname "$0")" && pwd)
[ -f "$dst/index.html" ] && grep -q 'src="app-core.js"' "$dst/index.html" || { echo "no cockpit index.html in $dst" >&2; exit 1; }
cp "$here/hexa-panels.js" "$dst/hexa-panels.js"
if [ -n "${2:-}" ]; then
  printf 'window.HEXA_CAMERA_URL = %s;\n' "\"$(printf %s "$2" | sed 's/[\\"]/\\&/g')\"" > "$dst/hexa-site.js"
else
  : > "$dst/hexa-site.js"
fi
if ! grep -q 'src="hexa-panels.js"' "$dst/index.html"; then
  sed -i 's|<script src="app-core.js"></script>|<script src="hexa-site.js"></script>\n<script src="hexa-panels.js"></script>\n&|' "$dst/index.html"
fi
grep -c 'src="hexa-panels.js"' "$dst/index.html"
