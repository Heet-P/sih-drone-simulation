"""Synthesise the promo soundtrack from the cue list the page exports.

  python render.py events     # writes out/events.json
  python3 sound.py            # writes out/soundtrack.wav (48 kHz stereo)

Everything is generated here (no samples): a 120 BPM pulse in A minor that
drops on the first wipe, plus UI sound effects placed on the exact visual cues.
"""
import json
import wave
from pathlib import Path

import numpy as np
from scipy.signal import butter, fftconvolve, sosfilt

SR = 48000
HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
rng = np.random.default_rng(3)


def t_axis(d):
    return np.arange(int(d * SR)) / SR


def lp(x, f, order=2):
    return sosfilt(butter(order, f, "low", fs=SR, output="sos"), x)


def hp(x, f, order=2):
    return sosfilt(butter(order, f, "high", fs=SR, output="sos"), x)


def bp(x, lo, hi, order=2):
    return sosfilt(butter(order, [lo, hi], "band", fs=SR, output="sos"), x)


def env(n, a, d, curve=6.0):
    """Linear attack a (s), exponential decay over d (s)."""
    t = np.arange(n) / SR
    e = np.exp(-curve * np.maximum(t - a, 0) / max(d, 1e-4))
    if a > 0:
        e *= np.clip(t / a, 0, 1)
    return e


def saw(freq, t, detune=0.0):
    ph = (freq * (1 + detune) * t) % 1.0
    return 2 * ph - 1


def noise(n):
    return rng.standard_normal(n)


def place(bus, x, t0, gain=1.0):
    i = int(t0 * SR)
    if i >= len(bus):
        return
    j = min(len(bus), i + len(x))
    if i < 0:
        x, i = x[-i:], 0
        j = min(len(bus), len(x))
    bus[i:j] += gain * x[: j - i]


# ------------------------------------------------------------------ voices
def kick(v=1.0):
    t = t_axis(0.45)
    f = 44 + 110 * np.exp(-t * 32)
    body = np.sin(2 * np.pi * np.cumsum(f) / SR) * env(len(t), 0.002, 0.42, 5)
    click = hp(noise(len(t)), 2500) * env(len(t), 0, 0.006, 6) * 0.25
    return (body + click) * v


def hat(v=1.0):
    t = t_axis(0.08)
    return hp(noise(len(t)), 7000, 4) * env(len(t), 0, 0.05, 6) * 0.22 * v


def clap(v=1.0):
    t = t_axis(0.25)
    n = bp(noise(len(t)), 900, 4200)
    e = env(len(t), 0.001, 0.16, 6)
    for k in (0.008, 0.017):  # smeared onsets
        i = int(k * SR)
        e[i:] += 0.6 * env(len(t) - i, 0.001, 0.12, 7)
    return n * e * 0.28 * v


def bass_note(freq, d, v=1.0):
    t = t_axis(d)
    x = saw(freq, t) + 0.6 * np.sin(2 * np.pi * freq * t)
    x = lp(x, 380, 2)
    return x * env(len(t), 0.004, d * 0.9, 3.5) * 0.5 * v


def pad_chord(freqs, d, v=1.0):
    t = t_axis(d)
    x = np.zeros(len(t))
    for f in freqs:
        for dt in (-0.004, 0.0, 0.0045):
            x += saw(f, t, dt)
    x = lp(x / (len(freqs) * 3), 1400, 2)
    a = np.clip(t / 0.35, 0, 1) * np.clip((d - t) / 0.4, 0, 1)
    return x * a * 0.5 * v


def tick(v=1.0):
    t = t_axis(0.05)
    x = np.sin(2 * np.pi * 2300 * t) * env(len(t), 0, 0.025, 6)
    x += hp(noise(len(t)), 5000) * env(len(t), 0, 0.004, 6) * 0.4
    return x * 0.22 * v


def ping(v=1.0):
    t = t_axis(0.9)
    x = (np.sin(2 * np.pi * 1318.5 * t) + 0.45 * np.sin(2 * np.pi * 1975.5 * t) + 0.2 * np.sin(2 * np.pi * 2637 * t))
    return x * env(len(t), 0.002, 0.7, 6) * 0.2 * v


def confirm(v=1.0):
    out = np.zeros(int(0.9 * SR))
    for k, f in enumerate((659.3, 880.0, 1318.5)):
        t = t_axis(0.7)
        n = (np.sin(2 * np.pi * f * t) + 0.3 * np.sin(4 * np.pi * f * t)) * env(len(t), 0.002, 0.55, 6)
        place(out, n, k * 0.07)
    return out * 0.17 * v


def buzz(v=1.0):
    t = t_axis(0.22)
    x = np.sign(np.sin(2 * np.pi * 116 * t)) * 0.6 + np.sign(np.sin(2 * np.pi * 122 * t)) * 0.4
    return lp(x, 900) * env(len(t), 0.003, 0.18, 4) * 0.16 * v


def snap(v=1.0):
    t = t_axis(0.12)
    x = bp(noise(len(t)), 1500, 7000) * env(len(t), 0, 0.03, 6)
    x += np.sin(2 * np.pi * 180 * t) * env(len(t), 0, 0.06, 6) * 0.6
    return x * 0.3 * v


def pop(v=1.0):
    t = t_axis(0.25)
    f = 300 + 700 * np.exp(-t * 30)
    x = np.sin(2 * np.pi * np.cumsum(f) / SR) * env(len(t), 0.001, 0.15, 6)
    return x * 0.25 * v


def slam(v=1.0):
    t = t_axis(0.6)
    f = 38 + 90 * np.exp(-t * 20)
    body = np.sin(2 * np.pi * np.cumsum(f) / SR) * env(len(t), 0.002, 0.5, 5)
    crack = bp(noise(len(t)), 600, 6000) * env(len(t), 0, 0.05, 6) * 0.5
    return (body * 0.9 + crack) * 0.55 * v


def impact(v=1.0):
    t = t_axis(2.6)
    f = 32 + 70 * np.exp(-t * 9)
    boom = np.sin(2 * np.pi * np.cumsum(f) / SR) * env(len(t), 0.003, 2.2, 4)
    air = lp(noise(len(t)), 3000) * env(len(t), 0.002, 1.2, 5) * 0.35
    return (boom + air) * 0.7 * v


def stamp(v=1.0):
    t = t_axis(0.3)
    body = np.sin(2 * np.pi * 95 * t) * env(len(t), 0.001, 0.12, 6)
    hit = lp(noise(len(t)), 2500) * env(len(t), 0, 0.03, 6) * 0.8
    return (body + hit) * 0.42 * v


def thud(v=1.0):
    t = t_axis(0.5)
    f = 55 + 60 * np.exp(-t * 25)
    return np.sin(2 * np.pi * np.cumsum(f) / SR) * env(len(t), 0.002, 0.35, 5) * 0.5 * v


def switch(v=1.0):
    out = np.zeros(int(0.12 * SR))
    place(out, tick(1.2), 0)
    place(out, tick(0.8), 0.05)
    return out * v


def filtered_sweep(dur, f0, f1, shape, v):
    """Noise through a band-pass whose centre glides f0 -> f1 (block-wise)."""
    n = int(dur * SR)
    x = noise(n)
    y = np.zeros(n)
    blk = 1024
    for i in range(0, n, blk):
        u = i / n
        fc = f0 * (f1 / f0) ** u
        seg = x[max(0, i - 2048): i + blk]
        s = bp(seg, max(40, fc * 0.6), min(SR / 2 - 100, fc * 1.6))
        y[i: i + blk] = s[-len(y[i: i + blk]):]
    return y * shape(np.linspace(0, 1, n)) * v


def whoosh(v=1.0):
    d = 0.75
    shape = lambda u: np.sin(np.pi * u) ** 2.2
    up = filtered_sweep(d, 400, 5000, shape, 1.0)
    return up * 0.32 * v


def swish(v=1.0):
    return filtered_sweep(0.45, 800, 6000, lambda u: np.sin(np.pi * u) ** 2, 1.0) * 0.18 * v


def sweep(v=1.0):
    return filtered_sweep(1.0, 300, 2400, lambda u: np.sin(np.pi * u) ** 1.5, 1.0) * 0.14 * v


def riser(dur=1.3, v=1.0):
    n = int(dur * SR)
    t = np.arange(n) / SR
    shape = lambda u: u ** 2.4
    x = filtered_sweep(dur, 300, 7000, shape, 1.0) * 0.35
    f = 220 * (4 ** (t / dur))
    x += np.sin(2 * np.pi * np.cumsum(f) / SR) * shape(t / dur) * 0.08
    return x * v


def counter(v=1.0):
    out = np.zeros(int(1.2 * SR))
    k, tt = 0, 0.0
    while tt < 1.05:  # accelerating then settling ticks, like a number rolling up
        place(out, tick(0.6 + 0.4 * (1 - tt)), tt)
        tt += 0.03 + 0.09 * (tt / 1.05) ** 2
        k += 1
    return out * v


def reverb_ir(dur=2.2, decay=3.2):
    t = t_axis(dur)
    ir = noise(len(t)) * np.exp(-decay * t)
    ir = lp(ir, 5000)
    ir[: int(0.012 * SR)] = 0
    return ir / np.sqrt(np.sum(ir ** 2))


# ------------------------------------------------------------------ score
def build():
    cues = json.loads((OUT / "events.json").read_text())
    dur = cues["duration"] + 0.6
    n = int(dur * SR)
    drums, music, sfx, verb_send = (np.zeros(n) for _ in range(4))
    side = np.ones(n)  # sidechain gain for music

    beat = 0.5
    groove0, groove1 = 7.6, 48.3  # drums from the first wipe to the end card
    # chords per bar (2 s): Am F C G
    chords = [
        (55.0, (220.0, 261.6, 329.6)),
        (43.65, (174.6, 220.0, 261.6)),
        (65.41, (196.0, 261.6, 329.6)),
        (49.0, (196.0, 246.9, 293.7)),
    ]

    # intro drone (0 - 3.9): low A pad swelling under the words
    t_int = t_axis(4.2)
    drone = pad_chord((110.0, 164.8, 220.0), 4.2, 0.55) * np.clip(t_int / 3.4, 0, 1) ** 1.5
    place(music, drone, 0)

    # title (3.9 - 7.6): pad + bass, no drums
    bar = 0
    tt = 3.9
    while tt < groove1 + 4:
        root, triad = chords[bar % 4]
        v = 0.9 if tt < groove0 else 1.0
        if tt >= groove1:
            root, triad = chords[0]
            v = 1.0
        d = 2.0 if tt < groove1 else 4.7
        place(music, pad_chord(triad, d + 0.3, v), tt)
        place(verb_send, pad_chord(triad, d + 0.3, v) * 0.5, tt)
        if tt < groove1:
            if tt >= groove0:  # 8th-note bass pluck in the groove
                for k in range(8):
                    if k in (3, 7) and bar % 2:
                        continue
                    place(music, bass_note(root * (2 if k in (3, 6) else 1), 0.22), tt + k * 0.25)
            else:
                place(music, bass_note(root, 1.9, 0.9), tt)
        else:
            place(music, bass_note(root, 4.0, 1.0), tt)
        if tt >= groove1:
            break
        tt += 2.0
        bar += 1

    # drums
    k = 0
    tt = groove0
    while tt < groove1 - 0.01:
        results = tt >= 44.0
        place(drums, kick(1.0 if results else 0.9), tt)
        ke = int(tt * SR)
        dip = 1 - 0.55 * env(int(0.32 * SR), 0.0, 0.32, 4)
        side[ke: ke + len(dip)] = np.minimum(side[ke: ke + len(dip)], dip[: len(side[ke: ke + len(dip)])])
        if k % 2 == 1:
            place(drums, clap(0.9 if not results else 1.1), tt)
        in_sitrep = 39.4 <= tt < 44.0
        place(drums, hat(0.55 if in_sitrep else 0.9), tt + beat / 2)
        if k % 4 == 3 and not in_sitrep:
            place(drums, hat(0.5), tt + beat * 0.75)
        tt += beat
        k += 1
    # drop out briefly before the end card for the impact to land
    fade_i = int((groove1 - 0.35) * SR)

    # sfx
    for e in cues["events"]:
        typ, t0, v = e["type"], e["t"], e["v"]
        if typ == "riser":
            x = riser(1.3, v)
            place(sfx, x, t0)
            continue
        fn = {"slam": slam, "impact": impact, "pop": pop, "tick": tick, "whoosh": whoosh, "swish": swish,
              "sweep": sweep, "ping": ping, "counter": counter, "snap": snap, "buzz": buzz, "confirm": confirm,
              "switch": switch, "thud": thud, "stamp": stamp}[typ]
        x = fn(v)
        off = -0.375 if typ == "whoosh" else (-0.2 if typ in ("swish",) else 0)
        place(sfx, x, t0 + off)
        if typ in ("ping", "confirm", "impact", "slam", "stamp", "pop"):
            place(verb_send, x * 0.6, t0 + off)

    music *= side
    verb = fftconvolve(verb_send, reverb_ir(), mode="full")[:n] * 0.22

    # stereo: light width on music and reverb
    L = drums + music * 0.92 + sfx + verb
    R = drums + music + sfx + np.roll(verb, int(0.011 * SR))
    mix = np.stack([L, R])
    mix = hp(mix, 28)
    mix[:, fade_i:] *= 1.0
    # end fade
    tail = np.clip((dur - 0.6 - t_axis(dur)) / 0.9, 0, 1)
    mix *= tail[: mix.shape[1]]
    # soft limit, then normalise to -1 dBFS
    mix = np.tanh(mix * 1.4 / max(1e-6, np.percentile(np.abs(mix), 99.9)))
    mix *= 10 ** (-1 / 20) / np.max(np.abs(mix))
    return mix


def write_wav(path, x):
    pcm = (np.clip(x.T, -1, 1) * 32767).astype("<i2")
    with wave.open(str(path), "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(pcm.tobytes())


if __name__ == "__main__":
    mix = build()
    write_wav(OUT / "soundtrack.wav", mix)
    rms = np.sqrt(np.mean(mix ** 2))
    print(f"out/soundtrack.wav  {mix.shape[1] / SR:.1f}s  peak {20 * np.log10(np.max(np.abs(mix))):.1f} dBFS  rms {20 * np.log10(rms):.1f} dBFS")
