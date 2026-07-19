# RAFCON web GUI

Browser-based viewer and execution controller for RAFCON. It connects to a running headless core
over the websocket network layer (`rafcon.network`) and renders state machines on a WebGL canvas
(Pixi.js) with smooth, effectively unlimited deep-zoom into nested state machines and libraries.

## Usage

```bash
# start the core with the websocket server and the web GUI
rafcon_core --server 9999 --web-port 8880

# then open
http://localhost:8880
```

`--web-port` implies `--server`. Alternatively enable both in the core `config.yaml` via
`NETWORK_SERVER_ENABLED`/`WEB_SERVER_ENABLED`.

State machines are opened from the browser: the **Libraries** tab lists everything under the
core's `LIBRARY_PATHS` (double-click a library to open it as a state machine), and the path
field below it opens any state machine folder on the core's file system. `-o <path>` on the
command line preloads one at startup.

## Multiple windows

Every browser window is an independent client; the URL decides what it shows:

- `/` — the full GUI
- `/?sm=<id>` — the full GUI pinned to one state machine (pop-out button on a state machine tab)
- `/?panel=logs|history|globals` — a single detached panel (pop-out button on the bottom panel tabs)

All windows stay in sync through the websocket server. The concurrent client limit is the
`NETWORK_MAX_CLIENTS` config value (default 10).

The web GUI is a viewer with execution control (start/pause/stop, all step modes, run-to/from
selected state): state machine editing is not supported over the network layer.

## Layout

- `app/` — Vue 3 + TypeScript + Tailwind + Pixi.js sources (Vite project)
- `dist/` — built static assets, served by `rafcon.network.web_server.WebServer`; committed so
  pip-installed users do not need node/npm

## Development

```bash
cd source/rafcon/web/app
npm install
npm run dev        # dev server on :5173; set VITE_WS_URL=ws://localhost:9999 or use ?ws=...
npm run test       # vitest (canvas math incl. the deep-zoom rebase invariant)
npm run typecheck
npm run build      # writes ../dist — commit the result
```

## How deep-zoom works (short)

The canvas never stores one absolute zoom factor. The camera is a pair
`(anchor state, f64 similarity transform local to that anchor)`; while zooming, the anchor is
re-rooted down/up the state tree (`src/canvas/camera.ts`) and every state's local space is
normalized to a canonical size (`src/canvas/model.ts`). All numbers reaching the GPU stay in
f32-safe ranges at any nesting depth; per-node level-of-detail culling keeps the visible node
count bounded.
