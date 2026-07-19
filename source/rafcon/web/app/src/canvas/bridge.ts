// Bridge between Vue components and the (non-reactive) Pixi scene.
// The SceneManager registers itself here; components call the exported helpers.

export interface CanvasBridge {
  zoomToStatePath(path: string): void
}

let bridge: CanvasBridge | null = null

export function registerCanvasBridge(instance: CanvasBridge | null): void {
  bridge = instance
}

export function zoomToStatePath(path: string): void {
  bridge?.zoomToStatePath(path)
}
