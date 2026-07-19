import { describe, expect, it } from 'vitest'
import { applyToPoint, compose, invert, scaleAround, translation } from '../transform'

describe('transform algebra', () => {
  it('compose applies right-to-left', () => {
    const move = translation(10, 0)
    const scale = { s: 2, tx: 0, ty: 0 }
    // scale after move: p -> 2 * (p + 10)
    expect(applyToPoint(compose(scale, move), 1, 0)).toEqual([22, 0])
    // move after scale: p -> 2p + 10
    expect(applyToPoint(compose(move, scale), 1, 0)).toEqual([12, 0])
  })

  it('invert round-trips', () => {
    const sim = { s: 3.7, tx: -12.3, ty: 99 }
    const [x, y] = applyToPoint(compose(sim, invert(sim)), 5, -8)
    expect(x).toBeCloseTo(5, 10)
    expect(y).toBeCloseTo(-8, 10)
  })

  it('scaleAround keeps the fixed point', () => {
    const sim = scaleAround(4, 100, 50)
    expect(applyToPoint(sim, 100, 50)).toEqual([100, 50])
    expect(applyToPoint(sim, 101, 50)[0]).toBeCloseTo(104)
  })
})
