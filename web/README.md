# Hexapod tab for the cockpit web UI

`hexa-panels.js` adds a **Hexapod** tab to linorobot2-cockpit's web UI with the miniHexa's own sensors:

- **Battery**: `/battery` (`sensor_msgs/BatteryState`), with its voltage coloured against 7.0 V (end the walk)
  and 6.8 V (the floor at rest), and a 5-minute chart.
- **Sonar**: `/sonar` (`sensor_msgs/Range`), with its range, a bar to the sensor's max range and a 5-minute chart.
- **Camera**: the camera board's `/capture` snapshots, or its `:81/stream` MJPEG. The stream serves **one
  client**: while the tab holds it, `camera_bridge` and other readers cannot have it. Leaving the tab or
  pressing Stop releases it. A snapshot takes about 1 s over the board's Wi-Fi, so snapshots run at about 1/s
  whatever rate is picked. The live stream runs at the board's `fps` setting.

Battery and sonar come over rosbridge on port 9090 of the page's host, the same as the Map Viewer. The tab
keeps its own socket and opens it only while the tab is shown.

Install into the cockpit container's frontend:

    docker cp web <container>:/tmp/hexa-web
    docker exec <container> sh /tmp/hexa-web/install_panels.sh /ws/web/frontend http://<camera ip>

The second argument is optional and sets the camera URL the tab starts with. A `?cam=<url>` on the page
overrides it, and the tab remembers the last URL typed. The script is idempotent, and it changes only one place
in cockpit's `index.html`: it adds two `<script>` lines before `app-core.js`.
